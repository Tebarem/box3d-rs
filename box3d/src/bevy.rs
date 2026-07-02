//! Optional Bevy integration.
//!
//! This module is available with the `bevy_ecs` feature. The `bevy` feature
//! enables [`Box3dPlugin`] and transform syncing.

use std::collections::HashMap;

use bevy_ecs::prelude::{Component, Entity, Resource};
#[cfg(feature = "bevy")]
use bevy_ecs::schedule::IntoScheduleConfigs;
use box3d_sys as sys;

use crate::{
    handle, BodyDef, BodyId, BodyType, Capacity, Quat, ShapeDef, ShapeId, SurfaceMaterial, Vec3,
    World,
};

/// Bevy minor version supported by this integration.
///
/// Bevy 0.19 currently requires a newer Rust compiler than this workspace uses,
/// so the feature is pinned to the latest compatible 0.18 release.
pub const SUPPORTED_BEVY_VERSION: &str = "0.18";

/// How the plugin advances the Box3D world.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Box3dTimestep {
    /// Step once per Bevy update using this simulation delta.
    Fixed(f32),
    /// Step once per Bevy update using elapsed wall time, capped by `max_delta`.
    Variable { max_delta: f32 },
}

impl Default for Box3dTimestep {
    fn default() -> Self {
        Self::Fixed(1.0 / 60.0)
    }
}

/// Settings for the Box3D world owned by a Bevy app.
#[derive(Clone, Copy, Debug, PartialEq, Resource)]
pub struct Box3dConfig {
    pub gravity: Vec3,
    pub timestep: Box3dTimestep,
    pub sub_steps: i32,
    pub capacity: Capacity,
    pub sleeping_enabled: bool,
    pub continuous_enabled: bool,
}

impl Default for Box3dConfig {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.8, 0.0),
            timestep: Box3dTimestep::default(),
            sub_steps: 4,
            capacity: Capacity::default(),
            sleeping_enabled: true,
            continuous_enabled: true,
        }
    }
}

/// Bevy plugin for Box3D world ownership, body creation, stepping, and transform sync.
#[cfg(feature = "bevy")]
#[derive(Clone, Copy, Debug, Default)]
pub struct Box3dPlugin {
    pub config: Box3dConfig,
}

#[cfg(feature = "bevy")]
impl bevy_app::Plugin for Box3dPlugin {
    fn build(&self, app: &mut bevy_app::App) {
        app.insert_resource(self.config)
            .insert_non_send_resource(Box3dWorld::new(self.config))
            .add_systems(
                bevy_app::Update,
                (
                    create_box3d_bodies,
                    sync_velocity_to_box3d,
                    sync_static_transforms_to_box3d,
                    step_box3d_world,
                    sync_box3d_to_transforms,
                    cleanup_box3d_bodies,
                )
                    .chain(),
            );
    }
}

/// Bevy rigid-body marker.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Component)]
pub enum RigidBody {
    Static,
    Kinematic,
    Dynamic,
}

impl From<RigidBody> for BodyType {
    fn from(value: RigidBody) -> Self {
        match value {
            RigidBody::Static => Self::Static,
            RigidBody::Kinematic => Self::Kinematic,
            RigidBody::Dynamic => Self::Dynamic,
        }
    }
}

/// Bevy collider component. One collider per entity for the first plugin pass.
#[derive(Clone, Copy, Debug, Component)]
pub struct Collider {
    shape: ColliderShape,
    def: ShapeDef,
    material: Option<SurfaceMaterial>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum ColliderShape {
    Cuboid { half_extents: Vec3 },
    Sphere { radius: f32 },
}

impl Collider {
    pub fn cuboid(half_extents: Vec3) -> Self {
        Self {
            shape: ColliderShape::Cuboid { half_extents },
            def: ShapeDef::default(),
            material: None,
        }
    }

    pub fn sphere(radius: f32) -> Self {
        Self {
            shape: ColliderShape::Sphere { radius },
            def: ShapeDef::default(),
            material: None,
        }
    }

    pub fn with_density(mut self, density: f32) -> Self {
        self.def.density = density;
        self
    }

    pub fn with_friction(mut self, friction: f32) -> Self {
        self.def.friction = friction;
        self
    }

    pub fn sensor(mut self, enabled: bool) -> Self {
        self.def.is_sensor = enabled;
        self
    }

    pub fn with_surface_material(mut self, material: SurfaceMaterial) -> Self {
        self.material = Some(material);
        self
    }
}

/// Linear and angular velocity synced into Box3D.
#[derive(Clone, Copy, Debug, Default, PartialEq, Component)]
pub struct Velocity {
    pub linear: Vec3,
    pub angular: Vec3,
}

impl Velocity {
    pub const fn linear(linear: Vec3) -> Self {
        Self {
            linear,
            angular: Vec3::ZERO,
        }
    }
}

/// Native Box3D body created for an entity.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Component)]
pub struct Box3dBody {
    pub id: BodyId,
}

/// Native Box3D shape created for an entity.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Component)]
pub struct Box3dShape {
    pub id: ShapeId,
}

/// Non-send Box3D world resource owned by the plugin.
pub struct Box3dWorld {
    world: World,
    bodies: HashMap<Entity, sys::b3BodyId>,
    last_step: std::time::Instant,
}

impl Box3dWorld {
    pub fn new(config: Box3dConfig) -> Self {
        let world = World::with_capacity(config.gravity, config.capacity);
        world.set_sleeping_enabled(config.sleeping_enabled);
        world.set_continuous_enabled(config.continuous_enabled);

        Self {
            world,
            bodies: HashMap::new(),
            last_step: std::time::Instant::now(),
        }
    }

    pub fn world(&self) -> &World {
        &self.world
    }

    fn body(&self, entity: Entity) -> Option<sys::b3BodyId> {
        self.bodies.get(&entity).copied()
    }

    fn remove_body(&mut self, entity: Entity) {
        if let Some(raw) = self.bodies.remove(&entity) {
            handle::destroy_body(raw);
        }
    }
}

impl Drop for Box3dWorld {
    fn drop(&mut self) {
        for (_, raw) in self.bodies.drain() {
            handle::destroy_body(raw);
        }
    }
}

#[cfg(feature = "bevy")]
#[allow(clippy::type_complexity)]
fn create_box3d_bodies(
    mut commands: bevy_ecs::prelude::Commands,
    mut physics: bevy_ecs::prelude::NonSendMut<Box3dWorld>,
    query: bevy_ecs::prelude::Query<
        (
            Entity,
            &RigidBody,
            Option<&Collider>,
            Option<&bevy_transform::prelude::Transform>,
            Option<&Velocity>,
        ),
        bevy_ecs::prelude::Without<Box3dBody>,
    >,
) {
    for (entity, rigid_body, collider, transform, velocity) in &query {
        let start = transform
            .map(bevy_transform_to_box3d)
            .unwrap_or(crate::Transform::IDENTITY);

        let body = physics.world.create_body(BodyDef {
            body_type: (*rigid_body).into(),
            position: start.p,
        });
        body.set_transform(start.p, start.q);

        if let Some(velocity) = velocity {
            body.set_linear_velocity(velocity.linear);
            body.set_angular_velocity(velocity.angular);
        }

        let body_id = body.id();
        let raw_body = body.raw();
        let shape_id = collider.map(|collider| {
            let shape = match collider.shape {
                ColliderShape::Cuboid { half_extents } => {
                    body.create_box(half_extents, collider.def)
                }
                ColliderShape::Sphere { radius } => {
                    body.create_sphere(Vec3::ZERO, radius, collider.def)
                }
            };

            if let Some(material) = collider.material {
                shape.set_surface_material(material);
            }

            let id = shape.id();
            std::mem::forget(shape);
            id
        });

        std::mem::forget(body);
        physics.bodies.insert(entity, raw_body);

        let mut entity_commands = commands.entity(entity);
        entity_commands.insert(Box3dBody { id: body_id });
        if let Some(id) = shape_id {
            entity_commands.insert(Box3dShape { id });
        }
    }
}

#[cfg(feature = "bevy")]
#[allow(clippy::type_complexity)]
fn sync_velocity_to_box3d(
    physics: bevy_ecs::prelude::NonSend<Box3dWorld>,
    query: bevy_ecs::prelude::Query<
        (Entity, &Velocity),
        (
            bevy_ecs::prelude::With<Box3dBody>,
            bevy_ecs::prelude::Changed<Velocity>,
        ),
    >,
) {
    for (entity, velocity) in &query {
        let Some(raw) = physics.body(entity) else {
            continue;
        };

        unsafe {
            sys::b3Body_SetLinearVelocity(raw, velocity.linear.into());
            sys::b3Body_SetAngularVelocity(raw, velocity.angular.into());
        }
    }
}

#[cfg(feature = "bevy")]
#[allow(clippy::type_complexity)]
fn sync_static_transforms_to_box3d(
    physics: bevy_ecs::prelude::NonSend<Box3dWorld>,
    query: bevy_ecs::prelude::Query<
        (Entity, &RigidBody, &bevy_transform::prelude::Transform),
        (
            bevy_ecs::prelude::With<Box3dBody>,
            bevy_ecs::prelude::Changed<bevy_transform::prelude::Transform>,
        ),
    >,
) {
    for (entity, rigid_body, transform) in &query {
        if *rigid_body == RigidBody::Dynamic {
            continue;
        }

        let Some(raw) = physics.body(entity) else {
            continue;
        };
        let transform = bevy_transform_to_box3d(transform);
        unsafe { sys::b3Body_SetTransform(raw, transform.p.into(), transform.q.into()) };
    }
}

#[cfg(feature = "bevy")]
fn step_box3d_world(
    config: bevy_ecs::prelude::Res<Box3dConfig>,
    mut physics: bevy_ecs::prelude::NonSendMut<Box3dWorld>,
) {
    physics.world.set_gravity(config.gravity);
    physics.world.set_sleeping_enabled(config.sleeping_enabled);
    physics
        .world
        .set_continuous_enabled(config.continuous_enabled);

    let time_step = match config.timestep {
        Box3dTimestep::Fixed(time_step) => time_step,
        Box3dTimestep::Variable { max_delta } => {
            let now = std::time::Instant::now();
            let elapsed = now.duration_since(physics.last_step).as_secs_f32();
            physics.last_step = now;
            elapsed.min(max_delta)
        }
    };

    if time_step > 0.0 {
        physics.world.step(time_step, config.sub_steps);
    }
}

#[cfg(feature = "bevy")]
fn sync_box3d_to_transforms(
    physics: bevy_ecs::prelude::NonSend<Box3dWorld>,
    mut query: bevy_ecs::prelude::Query<(
        Entity,
        &RigidBody,
        &mut bevy_transform::prelude::Transform,
    )>,
) {
    for (entity, rigid_body, mut transform) in &mut query {
        if *rigid_body != RigidBody::Dynamic {
            continue;
        }

        let Some(raw) = physics.body(entity) else {
            continue;
        };
        let raw_transform: crate::Transform = unsafe { sys::b3Body_GetTransform(raw) }.into();
        transform.translation = to_bevy_vec3(raw_transform.p);
        transform.rotation = to_bevy_quat(raw_transform.q);
    }
}

#[cfg(feature = "bevy")]
fn cleanup_box3d_bodies(
    mut commands: bevy_ecs::prelude::Commands,
    entities: &bevy_ecs::entity::Entities,
    mut physics: bevy_ecs::prelude::NonSendMut<Box3dWorld>,
    removed_rigid_bodies: bevy_ecs::prelude::Query<
        Entity,
        (
            bevy_ecs::prelude::With<Box3dBody>,
            bevy_ecs::prelude::Without<RigidBody>,
        ),
    >,
) {
    let removed: Vec<_> = physics
        .bodies
        .keys()
        .copied()
        .filter(|entity| !entities.contains(*entity) || removed_rigid_bodies.get(*entity).is_ok())
        .collect();

    for entity in removed {
        physics.remove_body(entity);
        if entities.contains(entity) {
            commands.entity(entity).remove::<(Box3dBody, Box3dShape)>();
        }
    }
}

#[cfg(feature = "bevy")]
fn bevy_transform_to_box3d(transform: &bevy_transform::prelude::Transform) -> crate::Transform {
    crate::Transform {
        p: Vec3::new(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ),
        q: Quat::new(
            Vec3::new(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
            ),
            transform.rotation.w,
        ),
    }
}

#[cfg(feature = "bevy")]
fn to_bevy_vec3(value: Vec3) -> bevy_math::Vec3 {
    bevy_math::Vec3::new(value.x, value.y, value.z)
}

#[cfg(feature = "bevy")]
fn to_bevy_quat(value: Quat) -> bevy_math::Quat {
    bevy_math::Quat::from_xyzw(value.v.x, value.v.y, value.v.z, value.s)
}

#[cfg(test)]
mod tests {
    use bevy_ecs::world::World as EcsWorld;

    use super::*;

    #[test]
    fn config_is_a_bevy_resource() {
        let mut world = EcsWorld::new();
        world.insert_resource(Box3dConfig::default());

        assert_eq!(world.resource::<Box3dConfig>().sub_steps, 4);
    }

    #[test]
    fn collider_builders_keep_shape_settings() {
        let collider = Collider::sphere(0.5).with_density(2.0).with_friction(0.8);

        assert_eq!(collider.def.density, 2.0);
        assert_eq!(collider.def.friction, 0.8);
    }

    #[cfg(feature = "bevy")]
    #[test]
    fn plugin_creates_body_and_syncs_dynamic_transform() {
        let mut app = bevy_app::App::new();
        app.add_plugins(Box3dPlugin {
            config: Box3dConfig {
                timestep: Box3dTimestep::Fixed(1.0 / 60.0),
                sub_steps: 4,
                ..Box3dConfig::default()
            },
        });

        let entity = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::cuboid(Vec3::new(0.5, 0.5, 0.5)).with_density(1.0),
                bevy_transform::prelude::Transform::from_xyz(0.0, 4.0, 0.0),
            ))
            .id();

        for _ in 0..10 {
            app.update();
        }

        let entity_ref = app.world().entity(entity);
        assert!(entity_ref.contains::<Box3dBody>());
        assert!(
            entity_ref
                .get::<bevy_transform::prelude::Transform>()
                .unwrap()
                .translation
                .y
                < 4.0
        );
    }
}
