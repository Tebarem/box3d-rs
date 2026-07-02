//! Bevy integration for Box3D.

use bevy_app::{FixedUpdate, RunFixedMainLoop, RunFixedMainLoopSystems};
use bevy_ecs::change_detection::DetectChanges;
use bevy_ecs::prelude::Entity;
use bevy_ecs::prelude::{Component, Message, Resource};
use bevy_ecs::schedule::{IntoScheduleConfigs, SystemSet};
use bevy_time::{Fixed, Time};

use box3d::Vec3 as BoxVec3;
use box3d::{
    BodyDef, BodyId, BodyType, Capacity, ContactId, Quat, ShapeDef, ShapeId, SurfaceMaterial,
    Transform as BoxTransform, World,
};
use std::collections::HashMap;

pub use bevy_math::Vec3;

/// Bevy minor version supported by this integration.
///
/// Bevy 0.19 currently requires a newer Rust compiler than this workspace uses,
/// so the feature is pinned to the latest compatible 0.18 release.
pub const SUPPORTED_BEVY_VERSION: &str = "0.18";

/// Public system sets for ordering gameplay around Box3D.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq, SystemSet)]
pub enum Box3dSet {
    Sync,
    Step,
    Writeback,
}

/// Settings for the Box3D world owned by a Bevy app.
#[derive(Clone, Copy, Debug, PartialEq, Resource)]
pub struct Box3dConfig {
    pub gravity: Vec3,
    pub fixed_hz: f64,
    pub sub_steps: i32,
    pub capacity: Capacity,
    pub sleeping_enabled: bool,
    pub continuous_enabled: bool,
}

impl Default for Box3dConfig {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.8, 0.0),
            fixed_hz: 60.0,
            sub_steps: 4,
            capacity: Capacity::default(),
            sleeping_enabled: true,
            continuous_enabled: true,
        }
    }
}

/// Last plugin step statistics.
#[derive(Clone, Copy, Debug, Default, PartialEq, Resource)]
pub struct Box3dStats {
    pub body_count: usize,
    pub step_count: u32,
    pub step_ms: f64,
    pub time_step: f32,
    pub interpolation_alpha: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Message)]
pub struct Box3dContactStarted {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub shape_a: ShapeId,
    pub shape_b: ShapeId,
    pub contact: ContactId,
}

#[derive(Clone, Copy, Debug, PartialEq, Message)]
pub struct Box3dContactEnded {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub shape_a: ShapeId,
    pub shape_b: ShapeId,
    pub contact: ContactId,
}

#[derive(Clone, Copy, Debug, PartialEq, Message)]
pub struct Box3dContactHit {
    pub entity_a: Entity,
    pub entity_b: Entity,
    pub shape_a: ShapeId,
    pub shape_b: ShapeId,
    pub contact: ContactId,
    pub point: Vec3,
    pub normal: Vec3,
    pub approach_speed: f32,
    pub user_material_id_a: u64,
    pub user_material_id_b: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Message)]
pub struct Box3dSensorStarted {
    pub sensor_entity: Entity,
    pub visitor_entity: Entity,
    pub sensor: ShapeId,
    pub visitor: ShapeId,
}

#[derive(Clone, Copy, Debug, PartialEq, Message)]
pub struct Box3dSensorEnded {
    pub sensor_entity: Entity,
    pub visitor_entity: Entity,
    pub sensor: ShapeId,
    pub visitor: ShapeId,
}

/// Bevy plugin for Box3D world ownership, body creation, stepping, and transform sync.
#[derive(Clone, Copy, Debug, Default)]
pub struct Box3dPlugin {
    pub config: Box3dConfig,
}

impl bevy_app::Plugin for Box3dPlugin {
    fn build(&self, app: &mut bevy_app::App) {
        app.add_message::<Box3dContactStarted>()
            .add_message::<Box3dContactEnded>()
            .add_message::<Box3dContactHit>()
            .add_message::<Box3dSensorStarted>()
            .add_message::<Box3dSensorEnded>()
            .insert_resource(self.config)
            .insert_resource(Time::<Fixed>::from_hz(fixed_hz(self.config.fixed_hz)))
            .insert_resource(Box3dStats::default())
            .insert_non_send_resource(Box3dWorld::new(self.config))
            .configure_sets(FixedUpdate, (Box3dSet::Sync, Box3dSet::Step).chain())
            .configure_sets(
                RunFixedMainLoop,
                Box3dSet::Writeback.in_set(RunFixedMainLoopSystems::AfterFixedMainLoop),
            )
            .add_systems(
                RunFixedMainLoop,
                sync_fixed_timestep.in_set(RunFixedMainLoopSystems::BeforeFixedMainLoop),
            )
            .add_systems(
                FixedUpdate,
                (
                    (
                        cleanup_box3d_shapes,
                        cleanup_box3d_bodies,
                        create_box3d_bodies,
                        create_box3d_shapes,
                        sync_velocity_to_box3d,
                        sync_damping_to_box3d,
                        sync_static_transforms_to_box3d,
                    )
                        .chain()
                        .in_set(Box3dSet::Sync),
                    step_box3d_world.in_set(Box3dSet::Step),
                ),
            );
        app.add_systems(
            RunFixedMainLoop,
            sync_box3d_to_transforms.in_set(Box3dSet::Writeback),
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

/// Bevy collider component.
#[derive(Clone, Copy, Debug, Component)]
pub struct Collider {
    shape: ColliderShape,
    def: ShapeDef,
    material: Option<SurfaceMaterial>,
}

/// Attach this collider entity to a different rigid-body entity.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Component)]
pub struct ColliderParent(pub Entity);

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

    pub fn contact_events(mut self, enabled: bool) -> Self {
        self.def.enable_contact_events = enabled;
        self
    }

    pub fn sensor_events(mut self, enabled: bool) -> Self {
        self.def.enable_sensor_events = enabled;
        self
    }

    pub fn hit_events(mut self, enabled: bool) -> Self {
        self.def.enable_hit_events = enabled;
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

/// Linear and angular damping synced into Box3D.
#[derive(Clone, Copy, Debug, Default, PartialEq, Component)]
pub struct Damping {
    pub linear: f32,
    pub angular: f32,
}

/// Native Box3D body created for an entity.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Component)]
pub struct Box3dBody {
    pub id: BodyId,
}

/// Native Box3D shape created for a collider entity.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Component)]
pub struct Box3dShape {
    pub id: ShapeId,
}

/// Non-send Box3D world resource owned by the plugin.
pub struct Box3dWorld {
    world: World,
    bodies: HashMap<Entity, BodyId>,
    shapes: HashMap<Entity, ShapeId>,
    shape_bodies: HashMap<Entity, Entity>,
    shape_entities: HashMap<u64, Entity>,
    transforms: HashMap<Entity, InterpolatedTransform>,
}

#[derive(Clone, Copy, Debug)]
struct InterpolatedTransform {
    previous: BoxTransform,
    current: BoxTransform,
}

impl Box3dWorld {
    pub fn new(config: Box3dConfig) -> Self {
        let world = World::with_capacity(to_box3d_vec3(config.gravity), config.capacity);
        world.set_sleeping_enabled(config.sleeping_enabled);
        world.set_continuous_enabled(config.continuous_enabled);

        Self {
            world,
            bodies: HashMap::new(),
            shapes: HashMap::new(),
            shape_bodies: HashMap::new(),
            shape_entities: HashMap::new(),
            transforms: HashMap::new(),
        }
    }

    pub fn world(&self) -> &World {
        &self.world
    }

    fn body(&self, entity: Entity) -> Option<BodyId> {
        self.bodies.get(&entity).copied()
    }

    fn shape_pair(&self, shape_a: ShapeId, shape_b: ShapeId) -> Option<(Entity, Entity)> {
        Some((
            *self.shape_entities.get(&shape_a.to_bits())?,
            *self.shape_entities.get(&shape_b.to_bits())?,
        ))
    }

    fn remove_body(&mut self, entity: Entity) -> Vec<Entity> {
        let shapes = self.remove_body_shapes(entity, false);
        if let Some(body) = self.bodies.remove(&entity) {
            body.destroy();
        }
        self.transforms.remove(&entity);
        shapes
    }

    fn remove_shape(&mut self, entity: Entity, destroy: bool) {
        if let Some(shape) = self.shapes.remove(&entity) {
            self.shape_entities.remove(&shape.to_bits());
            if destroy {
                shape.destroy(true);
            }
        }
        self.shape_bodies.remove(&entity);
    }

    fn remove_body_shapes(&mut self, body_entity: Entity, destroy: bool) -> Vec<Entity> {
        let shapes: Vec<_> = self
            .shape_bodies
            .iter()
            .filter_map(|(shape_entity, owner)| (*owner == body_entity).then_some(*shape_entity))
            .collect();
        for shape_entity in &shapes {
            self.remove_shape(*shape_entity, destroy);
        }
        shapes
    }
}

impl Drop for Box3dWorld {
    fn drop(&mut self) {
        for (_, body) in self.bodies.drain() {
            body.destroy();
        }
        self.shapes.clear();
        self.shape_bodies.clear();
        self.shape_entities.clear();
    }
}

#[allow(clippy::type_complexity)]
fn create_box3d_bodies(
    mut commands: bevy_ecs::prelude::Commands,
    mut physics: bevy_ecs::prelude::NonSendMut<Box3dWorld>,
    query: bevy_ecs::prelude::Query<
        (
            Entity,
            &RigidBody,
            Option<&bevy_transform::prelude::Transform>,
            Option<&Velocity>,
            Option<&Damping>,
        ),
        bevy_ecs::prelude::Without<Box3dBody>,
    >,
) {
    for (entity, rigid_body, transform, velocity, damping) in &query {
        let start = transform
            .map(bevy_transform_to_box3d)
            .unwrap_or(BoxTransform::IDENTITY);

        let body = physics.world.create_body(BodyDef {
            body_type: (*rigid_body).into(),
            position: start.p,
        });
        body.set_transform(start.p, start.q);
        let body_id = body.id();

        if let Some(velocity) = velocity {
            body.set_linear_velocity(to_box3d_vec3(velocity.linear));
            body.set_angular_velocity(to_box3d_vec3(velocity.angular));
        }
        if let Some(damping) = damping {
            body.set_linear_damping(damping.linear);
            body.set_angular_damping(damping.angular);
        }

        std::mem::forget(body);
        physics.bodies.insert(entity, body_id);
        physics.transforms.insert(
            entity,
            InterpolatedTransform {
                previous: start,
                current: start,
            },
        );

        commands.entity(entity).insert(Box3dBody { id: body_id });
    }
}

#[allow(clippy::type_complexity)]
fn create_box3d_shapes(
    mut commands: bevy_ecs::prelude::Commands,
    mut physics: bevy_ecs::prelude::NonSendMut<Box3dWorld>,
    query: bevy_ecs::prelude::Query<
        (
            Entity,
            &Collider,
            Option<&ColliderParent>,
            Option<&bevy_transform::prelude::Transform>,
        ),
        bevy_ecs::prelude::Without<Box3dShape>,
    >,
) {
    for (entity, collider, parent, transform) in &query {
        let body_entity = parent.map(|parent| parent.0).unwrap_or(entity);
        let Some(body) = physics.body(body_entity) else {
            continue;
        };

        let local_transform = if entity == body_entity {
            BoxTransform::IDENTITY
        } else {
            transform
                .map(bevy_transform_to_box3d)
                .unwrap_or(BoxTransform::IDENTITY)
        };

        let shape = match collider.shape {
            ColliderShape::Cuboid { half_extents } => {
                if entity == body_entity {
                    body.create_box(to_box3d_vec3(half_extents), collider.def)
                } else {
                    body.create_transformed_box(
                        to_box3d_vec3(half_extents),
                        local_transform,
                        collider.def,
                    )
                }
            }
            ColliderShape::Sphere { radius } => {
                body.create_sphere(local_transform.p, radius, collider.def)
            }
        };

        if let Some(material) = collider.material {
            shape.set_surface_material(material);
        }

        physics.shapes.insert(entity, shape);
        physics.shape_bodies.insert(entity, body_entity);
        physics.shape_entities.insert(shape.to_bits(), entity);
        commands.entity(entity).insert(Box3dShape { id: shape });
    }
}

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
        let Some(body) = physics.body(entity) else {
            continue;
        };

        body.set_linear_velocity(to_box3d_vec3(velocity.linear));
        body.set_angular_velocity(to_box3d_vec3(velocity.angular));
    }
}

#[allow(clippy::type_complexity)]
fn sync_damping_to_box3d(
    physics: bevy_ecs::prelude::NonSend<Box3dWorld>,
    query: bevy_ecs::prelude::Query<
        (Entity, &Damping),
        (
            bevy_ecs::prelude::With<Box3dBody>,
            bevy_ecs::prelude::Changed<Damping>,
        ),
    >,
) {
    for (entity, damping) in &query {
        let Some(body) = physics.body(entity) else {
            continue;
        };

        body.set_linear_damping(damping.linear);
        body.set_angular_damping(damping.angular);
    }
}

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

        let Some(body) = physics.body(entity) else {
            continue;
        };
        let transform = bevy_transform_to_box3d(transform);
        body.set_transform(transform.p, transform.q);
    }
}

fn step_box3d_world(
    fixed_time: bevy_ecs::prelude::Res<Time<Fixed>>,
    config: bevy_ecs::prelude::Res<Box3dConfig>,
    mut stats: bevy_ecs::prelude::ResMut<Box3dStats>,
    mut contact_started: bevy_ecs::prelude::MessageWriter<Box3dContactStarted>,
    mut contact_ended: bevy_ecs::prelude::MessageWriter<Box3dContactEnded>,
    mut contact_hit: bevy_ecs::prelude::MessageWriter<Box3dContactHit>,
    mut sensor_started: bevy_ecs::prelude::MessageWriter<Box3dSensorStarted>,
    mut sensor_ended: bevy_ecs::prelude::MessageWriter<Box3dSensorEnded>,
    mut physics: bevy_ecs::prelude::NonSendMut<Box3dWorld>,
) {
    physics.world.set_gravity(to_box3d_vec3(config.gravity));
    physics.world.set_sleeping_enabled(config.sleeping_enabled);
    physics
        .world
        .set_continuous_enabled(config.continuous_enabled);

    let started = std::time::Instant::now();
    let time_step = fixed_time.delta_secs();
    if time_step > 0.0 {
        store_previous_transforms(&mut physics);
        physics.world.step(time_step, config.sub_steps);
        store_current_transforms(&mut physics);
        emit_box3d_messages(
            &physics,
            &mut contact_started,
            &mut contact_ended,
            &mut contact_hit,
            &mut sensor_started,
            &mut sensor_ended,
        );
        update_stats(&mut stats, &physics, 1, time_step, started);
    } else {
        update_stats(&mut stats, &physics, 0, time_step, started);
    }
}

fn emit_box3d_messages(
    physics: &Box3dWorld,
    contact_started: &mut bevy_ecs::prelude::MessageWriter<Box3dContactStarted>,
    contact_ended: &mut bevy_ecs::prelude::MessageWriter<Box3dContactEnded>,
    contact_hit: &mut bevy_ecs::prelude::MessageWriter<Box3dContactHit>,
    sensor_started: &mut bevy_ecs::prelude::MessageWriter<Box3dSensorStarted>,
    sensor_ended: &mut bevy_ecs::prelude::MessageWriter<Box3dSensorEnded>,
) {
    let contacts = physics.world.contact_events();
    for event in contacts.begins() {
        let Some((entity_a, entity_b)) = physics.shape_pair(event.shape_a, event.shape_b) else {
            continue;
        };
        contact_started.write(Box3dContactStarted {
            entity_a,
            entity_b,
            shape_a: event.shape_a,
            shape_b: event.shape_b,
            contact: event.contact,
        });
    }

    for event in contacts.ends() {
        let Some((entity_a, entity_b)) = physics.shape_pair(event.shape_a, event.shape_b) else {
            continue;
        };
        contact_ended.write(Box3dContactEnded {
            entity_a,
            entity_b,
            shape_a: event.shape_a,
            shape_b: event.shape_b,
            contact: event.contact,
        });
    }

    for event in contacts.hits() {
        let Some((entity_a, entity_b)) = physics.shape_pair(event.shape_a, event.shape_b) else {
            continue;
        };
        contact_hit.write(Box3dContactHit {
            entity_a,
            entity_b,
            shape_a: event.shape_a,
            shape_b: event.shape_b,
            contact: event.contact,
            point: to_bevy_vec3(event.point),
            normal: to_bevy_vec3(event.normal),
            approach_speed: event.approach_speed,
            user_material_id_a: event.user_material_id_a,
            user_material_id_b: event.user_material_id_b,
        });
    }

    let sensors = physics.world.sensor_events();
    for event in sensors.begins() {
        let Some((sensor_entity, visitor_entity)) = physics.shape_pair(event.sensor, event.visitor)
        else {
            continue;
        };
        sensor_started.write(Box3dSensorStarted {
            sensor_entity,
            visitor_entity,
            sensor: event.sensor,
            visitor: event.visitor,
        });
    }

    for event in sensors.ends() {
        let Some((sensor_entity, visitor_entity)) = physics.shape_pair(event.sensor, event.visitor)
        else {
            continue;
        };
        sensor_ended.write(Box3dSensorEnded {
            sensor_entity,
            visitor_entity,
            sensor: event.sensor,
            visitor: event.visitor,
        });
    }
}

fn sync_fixed_timestep(
    config: bevy_ecs::prelude::Res<Box3dConfig>,
    mut fixed_time: bevy_ecs::prelude::ResMut<Time<Fixed>>,
) {
    if config.is_changed() {
        fixed_time.set_timestep_hz(fixed_hz(config.fixed_hz));
    }
}

fn fixed_hz(hz: f64) -> f64 {
    if hz.is_finite() && hz > 0.0 {
        hz
    } else {
        60.0
    }
}

fn update_stats(
    stats: &mut Box3dStats,
    physics: &Box3dWorld,
    step_count: u32,
    time_step: f32,
    started: std::time::Instant,
) {
    stats.body_count = physics.bodies.len();
    stats.step_count = step_count;
    stats.step_ms = started.elapsed().as_secs_f64() * 1000.0;
    stats.time_step = time_step.max(0.0);
}

fn store_previous_transforms(physics: &mut Box3dWorld) {
    for transform in physics.transforms.values_mut() {
        transform.previous = transform.current;
    }
}

fn store_current_transforms(physics: &mut Box3dWorld) {
    let bodies: Vec<_> = physics
        .bodies
        .iter()
        .map(|(entity, body)| (*entity, *body))
        .collect();
    for (entity, body) in bodies {
        if let (Some(transform), Some(entry)) =
            (body.transform(), physics.transforms.get_mut(&entity))
        {
            entry.current = transform;
        }
    }
}

fn sync_box3d_to_transforms(
    fixed_time: bevy_ecs::prelude::Res<Time<Fixed>>,
    mut stats: bevy_ecs::prelude::ResMut<Box3dStats>,
    physics: bevy_ecs::prelude::NonSend<Box3dWorld>,
    mut query: bevy_ecs::prelude::Query<(
        Entity,
        &RigidBody,
        &mut bevy_transform::prelude::Transform,
    )>,
) {
    let alpha = fixed_time.overstep_fraction();
    stats.body_count = physics.bodies.len();
    stats.interpolation_alpha = alpha;

    for (entity, rigid_body, mut transform) in &mut query {
        if *rigid_body != RigidBody::Dynamic {
            continue;
        }

        let Some(interpolated) = physics.transforms.get(&entity) else {
            continue;
        };

        transform.translation = lerp_vec3(interpolated.previous.p, interpolated.current.p, alpha);
        transform.rotation = to_bevy_quat(interpolated.previous.q)
            .slerp(to_bevy_quat(interpolated.current.q), alpha);
    }
}

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
        let removed_shapes = physics.remove_body(entity);
        for shape_entity in removed_shapes {
            if entities.contains(shape_entity) {
                commands.entity(shape_entity).remove::<Box3dShape>();
            }
        }
        if entities.contains(entity) {
            commands.entity(entity).remove::<Box3dBody>();
        }
    }
}

#[allow(clippy::type_complexity)]
fn cleanup_box3d_shapes(
    mut commands: bevy_ecs::prelude::Commands,
    entities: &bevy_ecs::entity::Entities,
    mut physics: bevy_ecs::prelude::NonSendMut<Box3dWorld>,
    removed_colliders: bevy_ecs::prelude::Query<
        Entity,
        (
            bevy_ecs::prelude::With<Box3dShape>,
            bevy_ecs::prelude::Without<Collider>,
        ),
    >,
    changed_colliders: bevy_ecs::prelude::Query<
        Entity,
        (
            bevy_ecs::prelude::With<Box3dShape>,
            bevy_ecs::prelude::Or<(
                bevy_ecs::prelude::Changed<Collider>,
                bevy_ecs::prelude::Changed<ColliderParent>,
            )>,
        ),
    >,
) {
    let removed: Vec<_> = physics
        .shapes
        .keys()
        .copied()
        .filter(|entity| {
            let missing_body = physics
                .shape_bodies
                .get(entity)
                .is_some_and(|body| !physics.bodies.contains_key(body));
            !entities.contains(*entity)
                || removed_colliders.get(*entity).is_ok()
                || changed_colliders.get(*entity).is_ok()
                || missing_body
        })
        .collect();

    for entity in removed {
        physics.remove_shape(entity, true);
        if entities.contains(entity) {
            commands.entity(entity).remove::<Box3dShape>();
        }
    }
}

fn bevy_transform_to_box3d(transform: &bevy_transform::prelude::Transform) -> BoxTransform {
    BoxTransform {
        p: BoxVec3::new(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ),
        q: Quat::new(
            BoxVec3::new(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
            ),
            transform.rotation.w,
        ),
    }
}

fn to_box3d_vec3(value: Vec3) -> BoxVec3 {
    BoxVec3::new(value.x, value.y, value.z)
}

fn to_bevy_vec3(value: BoxVec3) -> Vec3 {
    Vec3::new(value.x, value.y, value.z)
}

fn lerp_vec3(from: BoxVec3, to: BoxVec3, alpha: f32) -> Vec3 {
    to_bevy_vec3(BoxVec3::new(
        from.x + (to.x - from.x) * alpha,
        from.y + (to.y - from.y) * alpha,
        from.z + (to.z - from.z) * alpha,
    ))
}

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
        let collider = Collider::sphere(0.5)
            .with_density(2.0)
            .with_friction(0.8)
            .sensor(true)
            .contact_events(true)
            .sensor_events(true)
            .hit_events(true);

        assert_eq!(collider.def.density, 2.0);
        assert_eq!(collider.def.friction, 0.8);
        assert!(collider.def.is_sensor);
        assert!(collider.def.enable_contact_events);
        assert!(collider.def.enable_sensor_events);
        assert!(collider.def.enable_hit_events);
    }

    #[test]
    fn invalid_fixed_hz_falls_back_to_default() {
        assert_eq!(fixed_hz(120.0), 120.0);
        assert_eq!(fixed_hz(0.0), 60.0);
        assert_eq!(fixed_hz(f64::NAN), 60.0);
    }

    #[test]
    fn plugin_creates_body_and_syncs_dynamic_transform() {
        let mut app = bevy_app::App::new();
        app.add_plugins(bevy_time::TimePlugin)
            .insert_resource(bevy_time::TimeUpdateStrategy::FixedTimesteps(1));
        app.add_plugins(Box3dPlugin {
            config: Box3dConfig {
                fixed_hz: 60.0,
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
        assert_eq!(app.world().resource::<Box3dStats>().body_count, 1);
        assert_eq!(app.world().resource::<Box3dStats>().time_step, 1.0 / 60.0);
    }

    #[test]
    fn collider_parent_adds_shape_to_existing_body() {
        let mut app = bevy_app::App::new();
        app.add_plugins(bevy_time::TimePlugin)
            .insert_resource(bevy_time::TimeUpdateStrategy::FixedTimesteps(1));
        app.add_plugins(Box3dPlugin::default());

        let body = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::cuboid(Vec3::new(0.5, 0.5, 0.5)).with_density(1.0),
                bevy_transform::prelude::Transform::from_xyz(0.0, 4.0, 0.0),
            ))
            .id();
        let child = app
            .world_mut()
            .spawn((
                ColliderParent(body),
                Collider::sphere(0.25).with_density(1.0),
                bevy_transform::prelude::Transform::from_xyz(0.4, 0.0, 0.0),
            ))
            .id();

        for _ in 0..3 {
            app.update();
        }

        assert!(app.world().entity(body).contains::<Box3dShape>());
        assert!(app.world().entity(child).contains::<Box3dShape>());

        let physics = app.world().non_send_resource::<Box3dWorld>();
        assert_eq!(physics.bodies.len(), 1);
        assert_eq!(physics.shapes.len(), 2);
        assert_eq!(physics.shape_bodies.get(&child), Some(&body));
    }

    #[test]
    fn contact_messages_map_shape_ids_to_entities() {
        let mut app = bevy_app::App::new();
        app.add_plugins(bevy_time::TimePlugin)
            .insert_resource(bevy_time::TimeUpdateStrategy::FixedTimesteps(1));
        app.add_plugins(Box3dPlugin::default());

        let ground = app
            .world_mut()
            .spawn((
                RigidBody::Static,
                Collider::cuboid(Vec3::new(10.0, 0.5, 10.0)).contact_events(true),
                bevy_transform::prelude::Transform::from_xyz(0.0, -0.5, 0.0),
            ))
            .id();
        let ball = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::sphere(0.5)
                    .with_density(1.0)
                    .with_friction(0.3)
                    .contact_events(true),
                bevy_transform::prelude::Transform::from_xyz(0.0, 4.0, 0.0),
            ))
            .id();

        let mut saw_contact = false;
        for _ in 0..180 {
            app.update();
            saw_contact |= app
                .world()
                .resource::<bevy_ecs::message::Messages<Box3dContactStarted>>()
                .iter_current_update_messages()
                .any(|message| {
                    (message.entity_a == ground && message.entity_b == ball)
                        || (message.entity_a == ball && message.entity_b == ground)
                });
            if saw_contact {
                break;
            }
        }

        assert!(saw_contact);
    }

    #[test]
    fn sensor_messages_map_shape_ids_to_entities() {
        let mut app = bevy_app::App::new();
        app.add_plugins(bevy_time::TimePlugin)
            .insert_resource(bevy_time::TimeUpdateStrategy::FixedTimesteps(1));
        app.add_plugins(Box3dPlugin {
            config: Box3dConfig {
                gravity: Vec3::ZERO,
                ..Box3dConfig::default()
            },
        });

        let sensor = app
            .world_mut()
            .spawn((
                RigidBody::Static,
                Collider::cuboid(Vec3::new(1.0, 1.0, 1.0))
                    .sensor(true)
                    .sensor_events(true),
                bevy_transform::prelude::Transform::default(),
            ))
            .id();
        let visitor = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::sphere(0.25)
                    .with_density(1.0)
                    .with_friction(0.3)
                    .sensor_events(true),
                bevy_transform::prelude::Transform::default(),
            ))
            .id();

        let mut saw_sensor = false;
        for _ in 0..10 {
            app.update();
            saw_sensor |= app
                .world()
                .resource::<bevy_ecs::message::Messages<Box3dSensorStarted>>()
                .iter_current_update_messages()
                .any(|message| {
                    message.sensor_entity == sensor && message.visitor_entity == visitor
                });
            if saw_sensor {
                break;
            }
        }

        assert!(saw_sensor);
    }
}
