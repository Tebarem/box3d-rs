//! Bevy integration for Box3D.

use bevy_app::{FixedUpdate, RunFixedMainLoop, RunFixedMainLoopSystems};
use bevy_color::Color;
use bevy_ecs::change_detection::DetectChanges;
use bevy_ecs::prelude::Entity;
use bevy_ecs::prelude::{Component, Message, Resource};
use bevy_ecs::schedule::{IntoScheduleConfigs, SingleThreadedExecutor, SystemSet};
use bevy_gizmos::prelude::Gizmos;
use bevy_math::Isometry3d;
use bevy_time::{Fixed, Time};

use box3d::Vec3 as BoxVec3;
use box3d::{
    BodyCreateOptions, BodyDef, BodyId, BodyType, Capacity, ContactId, ContactTuning, Filter,
    Mesh as BoxMesh, MeshCreateOptions, Quat, ShapeDef, ShapeId, SurfaceMaterial,
    Transform as BoxTransform, World,
};
use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

pub use bevy_math::Vec3;

/// Bevy minor version supported by this integration.
pub const SUPPORTED_BEVY_VERSION: &str = "0.19";

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
    /// Box3D worker count. Use `1` for serial stepping.
    pub worker_count: u32,
    pub capacity: Capacity,
    pub sleeping_enabled: bool,
    pub continuous_enabled: bool,
    pub contact_tuning: Option<ContactTuning>,
    pub contact_recycle_distance: Option<f32>,
    pub restitution_threshold: Option<f32>,
    pub hit_event_threshold: Option<f32>,
    pub maximum_linear_speed: Option<f32>,
    pub warm_starting_enabled: bool,
    pub speculative_enabled: bool,
}

impl Default for Box3dConfig {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.8, 0.0),
            fixed_hz: 60.0,
            sub_steps: 4,
            worker_count: default_worker_count(),
            capacity: Capacity::default(),
            sleeping_enabled: true,
            continuous_enabled: true,
            contact_tuning: Some(default_contact_tuning()),
            contact_recycle_distance: None,
            restitution_threshold: None,
            hit_event_threshold: None,
            maximum_linear_speed: None,
            warm_starting_enabled: true,
            speculative_enabled: true,
        }
    }
}

/// Last plugin step statistics.
#[derive(Clone, Copy, Debug, Default, PartialEq, Resource)]
pub struct Box3dStats {
    pub body_count: usize,
    pub worker_count: usize,
    pub task_count: usize,
    pub byte_count: usize,
    pub move_event_count: usize,
    pub awake_body_count: usize,
    pub contact_count: usize,
    pub awake_contact_count: usize,
    pub recycled_contact_count: usize,
    pub manifold_count: usize,
    pub island_count: usize,
    pub sat_call_count: usize,
    pub sat_cache_hit_count: usize,
    pub constraint_color_counts: [i32; 24],
    pub overflow_constraint_count: usize,
    pub step_count: u32,
    pub step_ms: f64,
    pub native_step_ms: f32,
    pub native_pairs_ms: f32,
    pub native_collide_ms: f32,
    pub native_solve_ms: f32,
    pub native_refit_ms: f32,
    pub native_sleep_ms: f32,
    pub native_profile: box3d::Profile,
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
#[derive(Clone, Copy, Debug)]
pub struct Box3dPlugin {
    pub config: Box3dConfig,
    /// Run Bevy's fixed schedules on one thread so Box3D's native worker pool is not starved.
    pub single_threaded_schedules: bool,
}

impl Default for Box3dPlugin {
    fn default() -> Self {
        Self {
            config: Box3dConfig::default(),
            single_threaded_schedules: true,
        }
    }
}

impl bevy_app::Plugin for Box3dPlugin {
    fn build(&self, app: &mut bevy_app::App) {
        if self.single_threaded_schedules {
            app.edit_schedule(FixedUpdate, |schedule| {
                schedule.set_executor(SingleThreadedExecutor::new());
            });
            app.edit_schedule(RunFixedMainLoop, |schedule| {
                schedule.set_executor(SingleThreadedExecutor::new());
            });
        }

        app.add_message::<Box3dContactStarted>()
            .add_message::<Box3dContactEnded>()
            .add_message::<Box3dContactHit>()
            .add_message::<Box3dSensorStarted>()
            .add_message::<Box3dSensorEnded>()
            .insert_resource(self.config)
            .insert_resource(Time::<Fixed>::from_hz(fixed_hz(self.config.fixed_hz)))
            .insert_resource(Box3dStats::default())
            .insert_non_send(Box3dWorld::new(self.config))
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
                        sync_sleep_threshold_to_box3d,
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

#[derive(Clone, Copy, Debug, Resource)]
pub struct Box3dDebugConfig {
    pub enabled: bool,
    pub collider_color: Color,
    pub sensor_color: Color,
}

impl Default for Box3dDebugConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            collider_color: Color::srgb(0.1, 0.85, 1.0),
            sensor_color: Color::srgb(1.0, 0.8, 0.1),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Box3dDebugPlugin {
    pub config: Box3dDebugConfig,
}

impl bevy_app::Plugin for Box3dDebugPlugin {
    fn build(&self, app: &mut bevy_app::App) {
        app.insert_resource(self.config).add_systems(
            RunFixedMainLoop,
            draw_box3d_colliders
                .after(sync_box3d_to_transforms)
                .in_set(Box3dSet::Writeback),
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
#[derive(Clone, Debug, Component)]
pub struct Collider {
    shape: ColliderShape,
    def: ShapeDef,
}

/// Attach this collider entity to a different rigid-body entity.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Component)]
pub struct ColliderParent(pub Entity);

#[derive(Clone, Debug)]
enum ColliderShape {
    Cuboid { half_extents: Vec3 },
    Sphere { radius: f32 },
    Mesh { data: Arc<MeshColliderData> },
}

#[derive(Clone, Debug)]
struct MeshColliderData {
    vertices: Arc<[BoxVec3]>,
    indices: Arc<[u32]>,
    scale: Vec3,
    options: MeshCreateOptions,
}

impl Collider {
    pub fn cuboid(half_extents: Vec3) -> Self {
        Self {
            shape: ColliderShape::Cuboid { half_extents },
            def: ShapeDef::default(),
        }
    }

    pub fn sphere(radius: f32) -> Self {
        Self {
            shape: ColliderShape::Sphere { radius },
            def: ShapeDef::default(),
        }
    }

    pub fn mesh(vertices: Vec<Vec3>, indices: Vec<u32>) -> Self {
        Self::mesh_with_options(vertices, indices, Vec3::ONE, MeshCreateOptions::default())
    }

    pub fn mesh_with_options(
        vertices: Vec<Vec3>,
        indices: Vec<u32>,
        scale: Vec3,
        options: MeshCreateOptions,
    ) -> Self {
        let vertices = vertices
            .into_iter()
            .map(to_box3d_vec3)
            .collect::<Vec<_>>()
            .into();
        Self {
            shape: ColliderShape::Mesh {
                data: Arc::new(MeshColliderData {
                    vertices,
                    indices: indices.into(),
                    scale,
                    options,
                }),
            },
            def: ShapeDef::default(),
        }
    }

    pub fn with_density(mut self, density: f32) -> Self {
        self.def.density = density;
        self
    }

    pub fn with_friction(mut self, friction: f32) -> Self {
        self.def.friction = friction;
        if let Some(material) = &mut self.def.surface_material {
            material.friction = friction;
        }
        self
    }

    pub fn with_filter(mut self, filter: Filter) -> Self {
        self.def.filter = filter;
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

    pub fn invoke_contact_creation(mut self, enabled: bool) -> Self {
        self.def.invoke_contact_creation = enabled;
        self
    }

    pub fn with_surface_material(mut self, material: SurfaceMaterial) -> Self {
        self.def.friction = material.friction;
        self.def.surface_material = Some(material);
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

/// Allow higher angular velocity for small dynamic bodies such as vehicle wheels.
#[derive(Clone, Copy, Debug, Default, Component)]
pub struct FastRotation;

/// Sleep speed threshold synced into Box3D for this body.
#[derive(Clone, Copy, Debug, PartialEq, Component)]
pub struct SleepThreshold(pub f32);

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

#[derive(Clone, Copy, Debug, Default, Component)]
struct Box3dStaticBody;

/// Non-send Box3D world resource owned by the plugin.
pub struct Box3dWorld {
    world: World,
    bodies: HashMap<Entity, BodyId>,
    body_entities: HashMap<u64, Entity>,
    shapes: HashMap<Entity, ShapeId>,
    shape_bodies: HashMap<Entity, Entity>,
    shape_entities: HashMap<u64, Entity>,
    event_shapes: HashSet<Entity>,
    mesh_colliders: HashMap<Entity, BoxMesh>,
    interpolating: Vec<(Entity, InterpolatedTransform)>,
}

#[derive(Clone, Copy, Debug, Component)]
struct InterpolatedTransform {
    previous: BoxTransform,
    current: BoxTransform,
}

impl Box3dWorld {
    pub fn new(config: Box3dConfig) -> Self {
        let world = World::with_capacity_and_workers(
            to_box3d_vec3(config.gravity),
            config.capacity,
            config.worker_count,
        );
        world.set_sleeping_enabled(config.sleeping_enabled);
        world.set_continuous_enabled(config.continuous_enabled);
        world.set_warm_starting_enabled(config.warm_starting_enabled);
        world.set_speculative_enabled(config.speculative_enabled);
        if let Some(tuning) = valid_contact_tuning(config.contact_tuning) {
            world.set_contact_tuning(tuning);
        }
        if let Some(distance) = valid_contact_recycle_distance(config.contact_recycle_distance) {
            world.set_contact_recycle_distance(distance);
        }
        if let Some(threshold) = valid_non_negative(config.restitution_threshold) {
            world.set_restitution_threshold(threshold);
        }
        if let Some(threshold) = valid_non_negative(config.hit_event_threshold) {
            world.set_hit_event_threshold(threshold);
        }
        if let Some(speed) = valid_positive(config.maximum_linear_speed) {
            world.set_maximum_linear_speed(speed);
        }
        let body_capacity = capacity_len(config.capacity.static_body_count)
            + capacity_len(config.capacity.dynamic_body_count);
        let shape_capacity = capacity_len(config.capacity.static_shape_count)
            + capacity_len(config.capacity.dynamic_shape_count);
        let dynamic_body_capacity = capacity_len(config.capacity.dynamic_body_count);

        Self {
            world,
            bodies: HashMap::with_capacity(body_capacity),
            body_entities: HashMap::with_capacity(body_capacity),
            shapes: HashMap::with_capacity(shape_capacity),
            shape_bodies: HashMap::with_capacity(shape_capacity),
            shape_entities: HashMap::with_capacity(shape_capacity),
            event_shapes: HashSet::new(),
            mesh_colliders: HashMap::new(),
            interpolating: Vec::with_capacity(dynamic_body_capacity),
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
            self.body_entities.remove(&body.to_bits());
            body.destroy();
        }
        self.interpolating
            .retain(|(moved_entity, _)| *moved_entity != entity);
        shapes
    }

    fn remove_shape(&mut self, entity: Entity, destroy: bool) {
        if let Some(shape) = self.shapes.remove(&entity) {
            self.shape_entities.remove(&shape.to_bits());
            if destroy {
                shape.destroy(true);
            }
        }
        self.event_shapes.remove(&entity);
        self.mesh_colliders.remove(&entity);
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
        self.body_entities.clear();
        self.shapes.clear();
        self.shape_bodies.clear();
        self.shape_entities.clear();
        self.event_shapes.clear();
        self.mesh_colliders.clear();
        self.interpolating.clear();
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
            Option<&FastRotation>,
            Option<&SleepThreshold>,
            Option<&Collider>,
            Option<&ColliderParent>,
        ),
        bevy_ecs::prelude::Without<Box3dBody>,
    >,
) {
    for (
        entity,
        rigid_body,
        transform,
        velocity,
        damping,
        fast_rotation,
        sleep_threshold,
        collider,
        parent,
    ) in &query
    {
        let start = transform
            .map(bevy_transform_to_box3d)
            .unwrap_or(BoxTransform::IDENTITY);

        let body_id = physics.world.spawn_body_with_options(
            BodyDef {
                body_type: (*rigid_body).into(),
                position: start.p,
                rotation: start.q,
                linear_velocity: velocity
                    .map(|velocity| to_box3d_vec3(velocity.linear))
                    .unwrap_or(BoxVec3::ZERO),
                angular_velocity: velocity
                    .map(|velocity| to_box3d_vec3(velocity.angular))
                    .unwrap_or(BoxVec3::ZERO),
                linear_damping: damping.map(|damping| damping.linear).unwrap_or(0.0),
                angular_damping: damping.map(|damping| damping.angular).unwrap_or(0.0),
                sleep_threshold: sleep_threshold.and_then(valid_sleep_threshold),
                user_data: entity_user_data(entity),
            },
            BodyCreateOptions {
                allow_fast_rotation: fast_rotation.is_some(),
            },
        );
        physics.bodies.insert(entity, body_id);
        physics.body_entities.insert(body_id.to_bits(), entity);
        let mut entity_commands = commands.entity(entity);
        entity_commands.insert(Box3dBody { id: body_id });
        if *rigid_body == RigidBody::Static {
            entity_commands.insert(Box3dStaticBody);
        } else {
            entity_commands.insert(InterpolatedTransform {
                previous: start,
                current: start,
            });
        }

        if let (Some(collider), None) = (collider, parent) {
            let shape = create_box3d_shape(
                &mut physics,
                entity,
                body_id,
                collider,
                BoxTransform::IDENTITY,
            );
            track_box3d_shape(&mut physics, entity, entity, shape, collider);
            entity_commands.insert(Box3dShape { id: shape });
        }
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
        if physics.shapes.contains_key(&entity) {
            continue;
        }

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

        let shape = create_box3d_shape(&mut physics, entity, body, collider, local_transform);
        track_box3d_shape(&mut physics, entity, body_entity, shape, collider);
        commands.entity(entity).insert(Box3dShape { id: shape });
    }
}

fn create_box3d_shape(
    physics: &mut Box3dWorld,
    entity: Entity,
    body: BodyId,
    collider: &Collider,
    local_transform: BoxTransform,
) -> ShapeId {
    match &collider.shape {
        ColliderShape::Cuboid { half_extents } => {
            if local_transform != BoxTransform::IDENTITY {
                body.create_transformed_box(
                    to_box3d_vec3(*half_extents),
                    local_transform,
                    collider.def,
                )
            } else {
                body.create_box(to_box3d_vec3(*half_extents), collider.def)
            }
        }
        ColliderShape::Sphere { radius } => {
            body.create_sphere(local_transform.p, *radius, collider.def)
        }
        ColliderShape::Mesh { data } => {
            let mesh = BoxMesh::from_triangles_with_options(
                &data.vertices,
                &data.indices,
                None,
                data.options,
            )
            .expect("invalid box3d mesh collider");
            physics.mesh_colliders.insert(entity, mesh);
            let mesh = physics
                .mesh_colliders
                .get(&entity)
                .expect("box3d mesh collider was just inserted");
            body.create_mesh(mesh, to_box3d_vec3(data.scale), collider.def)
        }
    }
}

fn track_box3d_shape(
    physics: &mut Box3dWorld,
    entity: Entity,
    body_entity: Entity,
    shape: ShapeId,
    collider: &Collider,
) {
    physics.shapes.insert(entity, shape);
    physics.shape_bodies.insert(entity, body_entity);
    physics.shape_entities.insert(shape.to_bits(), entity);
    if collider_events_enabled(collider) {
        physics.event_shapes.insert(entity);
    }
}

fn collider_events_enabled(collider: &Collider) -> bool {
    collider.def.enable_contact_events
        || collider.def.enable_hit_events
        || collider.def.enable_sensor_events
}

#[allow(clippy::type_complexity)]
fn sync_velocity_to_box3d(
    query: bevy_ecs::prelude::Query<
        (
            bevy_ecs::prelude::Ref<Velocity>,
            bevy_ecs::prelude::Ref<Box3dBody>,
        ),
        (bevy_ecs::prelude::Changed<Velocity>,),
    >,
) {
    for (velocity, body) in &query {
        if body.is_added() {
            continue;
        }

        body.id.set_linear_velocity(to_box3d_vec3(velocity.linear));
        body.id
            .set_angular_velocity(to_box3d_vec3(velocity.angular));
    }
}

#[allow(clippy::type_complexity)]
fn sync_damping_to_box3d(
    query: bevy_ecs::prelude::Query<
        (
            bevy_ecs::prelude::Ref<Damping>,
            bevy_ecs::prelude::Ref<Box3dBody>,
        ),
        (bevy_ecs::prelude::Changed<Damping>,),
    >,
) {
    for (damping, body) in &query {
        if body.is_added() {
            continue;
        }

        body.id.set_linear_damping(damping.linear);
        body.id.set_angular_damping(damping.angular);
    }
}

#[allow(clippy::type_complexity)]
fn sync_sleep_threshold_to_box3d(
    query: bevy_ecs::prelude::Query<
        (
            bevy_ecs::prelude::Ref<SleepThreshold>,
            bevy_ecs::prelude::Ref<Box3dBody>,
        ),
        (bevy_ecs::prelude::Changed<SleepThreshold>,),
    >,
) {
    for (threshold, body) in &query {
        if body.is_added() {
            continue;
        }

        if let Some(threshold) = valid_sleep_threshold(&threshold) {
            body.id.set_sleep_threshold(threshold);
        }
    }
}

#[allow(clippy::type_complexity)]
fn sync_static_transforms_to_box3d(
    query: bevy_ecs::prelude::Query<
        (&Box3dBody, &RigidBody, &bevy_transform::prelude::Transform),
        (
            bevy_ecs::prelude::With<Box3dStaticBody>,
            bevy_ecs::prelude::Changed<bevy_transform::prelude::Transform>,
        ),
    >,
) {
    for (body, rigid_body, transform) in &query {
        if *rigid_body != RigidBody::Static {
            continue;
        }

        let transform = bevy_transform_to_box3d(transform);
        body.id.set_transform(transform.p, transform.q);
    }
}

#[allow(clippy::too_many_arguments)]
fn step_box3d_world(
    fixed_time: bevy_ecs::prelude::Res<Time<Fixed>>,
    config: bevy_ecs::prelude::Res<Box3dConfig>,
    mut stats: bevy_ecs::prelude::ResMut<Box3dStats>,
    mut contact_started: bevy_ecs::prelude::MessageWriter<Box3dContactStarted>,
    mut contact_ended: bevy_ecs::prelude::MessageWriter<Box3dContactEnded>,
    mut contact_hit: bevy_ecs::prelude::MessageWriter<Box3dContactHit>,
    mut sensor_started: bevy_ecs::prelude::MessageWriter<Box3dSensorStarted>,
    mut sensor_ended: bevy_ecs::prelude::MessageWriter<Box3dSensorEnded>,
    mut transforms: bevy_ecs::prelude::Query<&mut InterpolatedTransform>,
    mut physics: bevy_ecs::prelude::NonSendMut<Box3dWorld>,
) {
    if config.is_changed() {
        physics.world.set_gravity(to_box3d_vec3(config.gravity));
        physics.world.set_worker_count(config.worker_count);
        physics.world.set_sleeping_enabled(config.sleeping_enabled);
        physics
            .world
            .set_continuous_enabled(config.continuous_enabled);
        physics
            .world
            .set_warm_starting_enabled(config.warm_starting_enabled);
        physics
            .world
            .set_speculative_enabled(config.speculative_enabled);
        if let Some(tuning) = valid_contact_tuning(config.contact_tuning) {
            physics.world.set_contact_tuning(tuning);
        }
        if let Some(distance) = valid_contact_recycle_distance(config.contact_recycle_distance) {
            physics.world.set_contact_recycle_distance(distance);
        }
        if let Some(threshold) = valid_non_negative(config.restitution_threshold) {
            physics.world.set_restitution_threshold(threshold);
        }
        if let Some(threshold) = valid_non_negative(config.hit_event_threshold) {
            physics.world.set_hit_event_threshold(threshold);
        }
        if let Some(speed) = valid_positive(config.maximum_linear_speed) {
            physics.world.set_maximum_linear_speed(speed);
        }
    }

    let started = std::time::Instant::now();
    let time_step = fixed_time.delta_secs();
    if time_step > 0.0 {
        physics.world.step(time_step, config.sub_steps);
        let move_event_count = store_current_transforms(&mut physics, &mut transforms);
        if !physics.event_shapes.is_empty() {
            emit_box3d_messages(
                &physics,
                &mut contact_started,
                &mut contact_ended,
                &mut contact_hit,
                &mut sensor_started,
                &mut sensor_ended,
            );
        }
        update_stats(
            &mut stats,
            &physics,
            1,
            move_event_count,
            time_step,
            started,
        );
    } else {
        update_stats(&mut stats, &physics, 0, 0, time_step, started);
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

fn default_contact_tuning() -> ContactTuning {
    ContactTuning {
        hertz: 20.0,
        damping_ratio: 10.0,
        contact_speed: 2.0,
    }
}

fn valid_contact_recycle_distance(distance: Option<f32>) -> Option<f32> {
    distance.filter(|distance| distance.is_finite() && *distance >= 0.0)
}

fn valid_non_negative(value: Option<f32>) -> Option<f32> {
    value.filter(|value| value.is_finite() && *value >= 0.0)
}

fn valid_positive(value: Option<f32>) -> Option<f32> {
    value.filter(|value| value.is_finite() && *value > 0.0)
}

fn valid_contact_tuning(tuning: Option<ContactTuning>) -> Option<ContactTuning> {
    tuning.filter(|tuning| {
        tuning.hertz.is_finite()
            && tuning.hertz >= 0.0
            && tuning.damping_ratio.is_finite()
            && tuning.damping_ratio >= 0.0
            && tuning.contact_speed.is_finite()
            && tuning.contact_speed >= 0.0
    })
}

fn valid_sleep_threshold(threshold: &SleepThreshold) -> Option<f32> {
    (threshold.0.is_finite() && threshold.0 >= 0.0).then_some(threshold.0)
}

fn capacity_len(value: i32) -> usize {
    value.max(0) as usize
}

fn manifold_count(buckets: [i32; 8]) -> usize {
    buckets
        .into_iter()
        .enumerate()
        .map(|(index, count)| (index + 1) * count.max(0) as usize)
        .sum()
}

fn overflow_constraint_count(color_counts: [i32; 24]) -> usize {
    color_counts.last().copied().unwrap_or(0).max(0) as usize
}

fn default_worker_count() -> u32 {
    let max = 8usize.min(box3d::MAX_WORKERS as usize).max(1);
    std::thread::available_parallelism()
        .map(|count| (count.get() / 2).clamp(1, max) as u32)
        .unwrap_or(1)
}

fn entity_user_data(entity: Entity) -> usize {
    usize::try_from(entity.to_bits()).unwrap_or(0)
}

fn entity_from_user_data(user_data: usize) -> Option<Entity> {
    if user_data == 0 {
        return None;
    }

    Entity::try_from_bits(u64::try_from(user_data).ok()?)
}

fn update_stats(
    stats: &mut Box3dStats,
    physics: &Box3dWorld,
    step_count: u32,
    move_event_count: usize,
    time_step: f32,
    started: std::time::Instant,
) {
    let counters = physics.world.counters();
    let profile = physics.world.profile();
    stats.body_count = counters.body_count.max(0) as usize;
    stats.worker_count = physics.world.worker_count() as usize;
    stats.task_count = counters.task_count.max(0) as usize;
    stats.byte_count = counters.byte_count.max(0) as usize;
    stats.move_event_count = move_event_count;
    stats.awake_body_count = physics.world.awake_body_count().max(0) as usize;
    stats.contact_count = counters.contact_count.max(0) as usize;
    stats.awake_contact_count = counters.awake_contact_count.max(0) as usize;
    stats.recycled_contact_count = counters.recycled_contact_count.max(0) as usize;
    stats.manifold_count = manifold_count(counters.manifold_counts);
    stats.island_count = counters.island_count.max(0) as usize;
    stats.sat_call_count = counters.sat_call_count.max(0) as usize;
    stats.sat_cache_hit_count = counters.sat_cache_hit_count.max(0) as usize;
    stats.constraint_color_counts = counters.color_counts;
    stats.overflow_constraint_count = overflow_constraint_count(counters.color_counts);
    stats.step_count = step_count;
    stats.step_ms = started.elapsed().as_secs_f64() * 1000.0;
    stats.native_step_ms = profile.step;
    stats.native_pairs_ms = profile.pairs;
    stats.native_collide_ms = profile.collide;
    stats.native_solve_ms = profile.solve;
    stats.native_refit_ms = profile.refit;
    stats.native_sleep_ms = profile.sleep_islands;
    stats.native_profile = profile;
    stats.time_step = time_step.max(0.0);
}

fn store_current_transforms(
    physics: &mut Box3dWorld,
    transforms: &mut bevy_ecs::prelude::Query<&mut InterpolatedTransform>,
) -> usize {
    let mut count = 0;
    physics.interpolating.clear();
    let body_events = physics.world.body_events();
    let move_events = body_events.moves();
    physics.interpolating.reserve(move_events.size_hint().0);
    for event in move_events {
        count += 1;
        let entity = entity_from_user_data(event.user_data)
            .or_else(|| physics.body_entities.get(&event.body.to_bits()).copied());
        let Some(entity) = entity else {
            continue;
        };
        if let Ok(mut entry) = transforms.get_mut(entity) {
            entry.previous = entry.current;
            entry.current = event.transform;
            physics.interpolating.push((entity, *entry));
        }
    }
    count
}

fn sync_box3d_to_transforms(
    fixed_time: bevy_ecs::prelude::Res<Time<Fixed>>,
    mut stats: bevy_ecs::prelude::ResMut<Box3dStats>,
    physics: bevy_ecs::prelude::NonSend<Box3dWorld>,
    mut query: bevy_ecs::prelude::Query<
        &mut bevy_transform::prelude::Transform,
        (
            bevy_ecs::prelude::With<Box3dBody>,
            bevy_ecs::prelude::Without<Box3dStaticBody>,
        ),
    >,
) {
    let alpha = fixed_time.overstep_fraction();
    stats.interpolation_alpha = alpha;

    for (entity, interpolated) in &physics.interpolating {
        let Ok(mut transform) = query.get_mut(*entity) else {
            continue;
        };

        transform.translation = lerp_vec3(interpolated.previous.p, interpolated.current.p, alpha);
        transform.rotation = to_bevy_quat(interpolated.previous.q)
            .slerp(to_bevy_quat(interpolated.current.q), alpha);
    }
}

#[allow(clippy::type_complexity)]
fn draw_box3d_colliders(
    config: bevy_ecs::prelude::Res<Box3dDebugConfig>,
    mut gizmos: Gizmos,
    bodies: bevy_ecs::prelude::Query<
        Option<&bevy_transform::prelude::Transform>,
        bevy_ecs::prelude::With<Box3dBody>,
    >,
    colliders: bevy_ecs::prelude::Query<(
        Entity,
        &Collider,
        Option<&ColliderParent>,
        Option<&bevy_transform::prelude::Transform>,
    )>,
) {
    if !config.enabled {
        return;
    }

    for (entity, collider, parent, local_transform) in &colliders {
        let body_entity = parent.map(|parent| parent.0).unwrap_or(entity);
        let Ok(body_transform) = bodies.get(body_entity) else {
            continue;
        };
        let body_transform = body_transform.copied().unwrap_or_default();
        let transform = collider_debug_transform(
            body_transform,
            (entity != body_entity)
                .then_some(local_transform)
                .flatten()
                .copied()
                .unwrap_or_default(),
        );
        let color = if collider.def.is_sensor {
            config.sensor_color
        } else {
            config.collider_color
        };

        match &collider.shape {
            ColliderShape::Cuboid { half_extents } => {
                let mut cube = transform;
                cube.scale = *half_extents * 2.0;
                gizmos.cube(cube, color);
            }
            ColliderShape::Sphere { radius } => {
                gizmos.sphere(
                    Isometry3d::new(transform.translation, transform.rotation),
                    *radius,
                    color,
                );
            }
            ColliderShape::Mesh { .. } => {}
        }
    }
}

fn cleanup_box3d_bodies(
    mut commands: bevy_ecs::prelude::Commands,
    entities: &bevy_ecs::entity::Entities,
    mut physics: bevy_ecs::prelude::NonSendMut<Box3dWorld>,
    mut removed_rigid_bodies: bevy_ecs::lifecycle::RemovedComponents<RigidBody>,
    current_rigid_bodies: bevy_ecs::prelude::Query<(), bevy_ecs::prelude::With<RigidBody>>,
) {
    let removed: Vec<_> = removed_rigid_bodies
        .read()
        .filter(|entity| physics.bodies.contains_key(entity))
        .filter(|entity| !entities.contains(*entity) || current_rigid_bodies.get(*entity).is_err())
        .collect();

    for entity in removed {
        let removed_shapes = physics.remove_body(entity);
        for shape_entity in removed_shapes {
            if entities.contains(shape_entity) {
                commands.entity(shape_entity).remove::<Box3dShape>();
            }
        }
        if entities.contains(entity) {
            commands
                .entity(entity)
                .remove::<(Box3dBody, Box3dStaticBody, InterpolatedTransform)>();
        }
    }
}

#[allow(clippy::type_complexity)]
fn cleanup_box3d_shapes(
    mut commands: bevy_ecs::prelude::Commands,
    entities: &bevy_ecs::entity::Entities,
    mut physics: bevy_ecs::prelude::NonSendMut<Box3dWorld>,
    mut removed_colliders: bevy_ecs::lifecycle::RemovedComponents<Collider>,
    current_colliders: bevy_ecs::prelude::Query<(), bevy_ecs::prelude::With<Collider>>,
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
    let mut removed: Vec<_> = removed_colliders
        .read()
        .filter(|entity| physics.shapes.contains_key(entity))
        .filter(|entity| !entities.contains(*entity) || current_colliders.get(*entity).is_err())
        .collect();
    removed.extend(changed_colliders.iter());

    for entity in removed {
        physics.remove_shape(entity, true);
        if entities.contains(entity) {
            commands.entity(entity).remove::<Box3dShape>();
        }
    }
}

fn collider_debug_transform(
    body: bevy_transform::prelude::Transform,
    local: bevy_transform::prelude::Transform,
) -> bevy_transform::prelude::Transform {
    bevy_transform::prelude::Transform {
        translation: body.translation + body.rotation * local.translation,
        rotation: body.rotation * local.rotation,
        scale: Vec3::ONE,
    }
}

fn bevy_transform_to_box3d(transform: &bevy_transform::prelude::Transform) -> BoxTransform {
    let rotation = normalized_bevy_rotation(transform.rotation);
    BoxTransform {
        p: BoxVec3::new(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ),
        q: Quat::new(BoxVec3::new(rotation.x, rotation.y, rotation.z), rotation.w),
    }
}

fn normalized_bevy_rotation(rotation: bevy_math::Quat) -> bevy_math::Quat {
    let length_squared = rotation.length_squared();
    if rotation.is_finite() && length_squared.is_finite() && length_squared > f32::MIN_POSITIVE {
        rotation.normalize()
    } else {
        bevy_math::Quat::IDENTITY
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
        assert!(world.resource::<Box3dConfig>().worker_count >= 1);
        assert_eq!(
            world.resource::<Box3dConfig>().contact_recycle_distance,
            None
        );
        assert_eq!(
            world.resource::<Box3dConfig>().contact_tuning,
            Some(default_contact_tuning())
        );
        assert_eq!(world.resource::<Box3dConfig>().restitution_threshold, None);
        assert_eq!(world.resource::<Box3dConfig>().hit_event_threshold, None);
        assert_eq!(world.resource::<Box3dConfig>().maximum_linear_speed, None);
        assert!(world.resource::<Box3dConfig>().warm_starting_enabled);
        assert!(world.resource::<Box3dConfig>().speculative_enabled);
        assert!(Box3dPlugin::default().single_threaded_schedules);
    }

    #[test]
    fn collider_builders_keep_shape_settings() {
        let material = SurfaceMaterial {
            friction: 0.65,
            restitution: 0.2,
            rolling_resistance: 0.04,
            ..SurfaceMaterial::default()
        };
        let collider = Collider::sphere(0.5)
            .with_density(2.0)
            .with_friction(0.8)
            .with_filter(Filter {
                category_bits: 0x2,
                mask_bits: 0x4,
                group_index: -3,
            })
            .sensor(true)
            .contact_events(true)
            .sensor_events(true)
            .hit_events(true)
            .with_surface_material(material)
            .invoke_contact_creation(false);

        assert_eq!(collider.def.density, 2.0);
        assert_eq!(collider.def.friction, material.friction);
        assert_eq!(collider.def.surface_material, Some(material));
        assert_eq!(
            collider.def.filter,
            Filter {
                category_bits: 0x2,
                mask_bits: 0x4,
                group_index: -3,
            }
        );
        assert!(collider.def.is_sensor);
        assert!(collider.def.enable_contact_events);
        assert!(collider.def.enable_sensor_events);
        assert!(collider.def.enable_hit_events);
        assert!(!collider.def.invoke_contact_creation);
    }

    #[test]
    fn invalid_fixed_hz_falls_back_to_default() {
        assert_eq!(fixed_hz(120.0), 120.0);
        assert_eq!(fixed_hz(0.0), 60.0);
        assert_eq!(fixed_hz(f64::NAN), 60.0);
    }

    #[test]
    fn contact_recycle_distance_ignores_invalid_values() {
        assert_eq!(valid_contact_recycle_distance(Some(0.25)), Some(0.25));
        assert_eq!(valid_contact_recycle_distance(Some(-0.25)), None);
        assert_eq!(valid_contact_recycle_distance(Some(f32::NAN)), None);
        assert_eq!(valid_contact_recycle_distance(None), None);
        assert_eq!(valid_non_negative(Some(0.0)), Some(0.0));
        assert_eq!(valid_non_negative(Some(-1.0)), None);
        assert_eq!(valid_positive(Some(1.0)), Some(1.0));
        assert_eq!(valid_positive(Some(0.0)), None);
        assert_eq!(
            valid_contact_tuning(Some(ContactTuning::default())),
            Some(ContactTuning::default())
        );
        assert_eq!(
            valid_contact_tuning(Some(ContactTuning {
                hertz: f32::NAN,
                ..ContactTuning::default()
            })),
            None
        );
        assert_eq!(valid_sleep_threshold(&SleepThreshold(0.25)), Some(0.25));
        assert_eq!(valid_sleep_threshold(&SleepThreshold(f32::NAN)), None);
    }

    #[test]
    fn world_config_applies_native_tuning_knobs() {
        let physics = Box3dWorld::new(Box3dConfig {
            restitution_threshold: Some(1.25),
            hit_event_threshold: Some(2.5),
            maximum_linear_speed: Some(60.0),
            warm_starting_enabled: false,
            speculative_enabled: false,
            ..Box3dConfig::default()
        });

        assert_eq!(physics.world.restitution_threshold(), 1.25);
        assert_eq!(physics.world.hit_event_threshold(), 2.5);
        assert_eq!(physics.world.maximum_linear_speed(), 60.0);
        assert!(!physics.world.is_warm_starting_enabled());
    }

    #[test]
    fn manifold_count_weights_native_buckets() {
        assert_eq!(manifold_count([2, 1, 0, -1, 0, 0, 0, 1]), 12);
        assert_eq!(overflow_constraint_count([0; 24]), 0);
        let mut colors = [0; 24];
        colors[23] = 7;
        assert_eq!(overflow_constraint_count(colors), 7);
    }

    #[test]
    fn sleep_threshold_syncs_to_body() {
        let mut app = bevy_app::App::new();
        app.add_plugins(bevy_time::TimePlugin)
            .insert_resource(bevy_time::TimeUpdateStrategy::FixedTimesteps(1));
        app.add_plugins(Box3dPlugin::default());

        let entity = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::sphere(0.5).with_density(1.0),
                SleepThreshold(0.25),
                bevy_transform::prelude::Transform::default(),
            ))
            .id();

        for _ in 0..3 {
            app.update();
        }
        {
            let physics = app.world().non_send::<Box3dWorld>();
            assert_eq!(physics.body(entity).unwrap().sleep_threshold(), 0.25);
        }

        app.world_mut()
            .entity_mut(entity)
            .insert(SleepThreshold(0.5));
        for _ in 0..3 {
            app.update();
        }

        let physics = app.world().non_send::<Box3dWorld>();
        assert_eq!(physics.body(entity).unwrap().sleep_threshold(), 0.5);
    }

    #[test]
    fn transform_conversion_normalizes_rotation_for_box3d() {
        let mut transform = bevy_transform::prelude::Transform::default();
        transform.rotation = bevy_math::Quat::from_xyzw(0.0, 0.2, 0.0, 1.0);

        let converted = bevy_transform_to_box3d(&transform);
        let length = converted.q.v.x * converted.q.v.x
            + converted.q.v.y * converted.q.v.y
            + converted.q.v.z * converted.q.v.z
            + converted.q.s * converted.q.s;

        assert!((length - 1.0).abs() < f32::EPSILON);
        assert_eq!(
            normalized_bevy_rotation(bevy_math::Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0)),
            bevy_math::Quat::IDENTITY
        );
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
            ..Box3dPlugin::default()
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
        assert!(entity_ref.contains::<Box3dShape>());
        assert_eq!(app.world().resource::<Box3dStats>().body_count, 1);
        assert!(app.world().resource::<Box3dStats>().worker_count >= 1);
        assert_eq!(app.world().resource::<Box3dStats>().time_step, 1.0 / 60.0);
    }

    #[test]
    fn kinematic_velocity_writes_back_transform() {
        let mut app = bevy_app::App::new();
        app.add_plugins(bevy_time::TimePlugin)
            .insert_resource(bevy_time::TimeUpdateStrategy::FixedTimesteps(1));
        app.add_plugins(Box3dPlugin::default());

        let entity = app
            .world_mut()
            .spawn((
                RigidBody::Kinematic,
                Velocity::linear(Vec3::X),
                bevy_transform::prelude::Transform::default(),
            ))
            .id();

        for _ in 0..3 {
            app.update();
        }

        let transform = app
            .world()
            .entity(entity)
            .get::<bevy_transform::prelude::Transform>()
            .unwrap();
        assert!(transform.translation.x > 0.0, "{transform:?}");
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

        let physics = app.world().non_send::<Box3dWorld>();
        assert_eq!(physics.bodies.len(), 1);
        assert_eq!(physics.shapes.len(), 2);
        assert_eq!(physics.shape_bodies.get(&child), Some(&body));
    }

    #[test]
    fn mesh_collider_keeps_native_mesh_alive() {
        let mut app = bevy_app::App::new();
        app.add_plugins(bevy_time::TimePlugin)
            .insert_resource(bevy_time::TimeUpdateStrategy::FixedTimesteps(1));
        app.add_plugins(Box3dPlugin::default());

        let terrain = app
            .world_mut()
            .spawn((
                RigidBody::Static,
                Collider::mesh(
                    vec![
                        Vec3::new(-1.0, 0.0, -1.0),
                        Vec3::new(1.0, 0.0, -1.0),
                        Vec3::new(-1.0, 0.0, 1.0),
                        Vec3::new(1.0, 0.0, 1.0),
                    ],
                    vec![0, 2, 1, 1, 2, 3],
                ),
                bevy_transform::prelude::Transform::default(),
            ))
            .id();

        for _ in 0..3 {
            app.update();
        }

        let physics = app.world().non_send::<Box3dWorld>();
        assert_eq!(physics.mesh_colliders.len(), 1);
        assert!(physics.shapes.get(&terrain).unwrap().is_valid());
    }

    #[test]
    fn removed_components_destroy_native_body_and_shape() {
        let mut app = bevy_app::App::new();
        app.add_plugins(bevy_time::TimePlugin)
            .insert_resource(bevy_time::TimeUpdateStrategy::FixedTimesteps(1));
        app.add_plugins(Box3dPlugin::default());

        let entity = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::sphere(0.5).with_density(1.0),
                bevy_transform::prelude::Transform::from_xyz(0.0, 4.0, 0.0),
            ))
            .id();

        for _ in 0..3 {
            app.update();
        }
        assert!(app.world().entity(entity).contains::<Box3dBody>());
        assert!(app.world().entity(entity).contains::<Box3dShape>());

        app.world_mut().entity_mut(entity).remove::<Collider>();
        for _ in 0..3 {
            app.update();
        }
        assert!(!app.world().entity(entity).contains::<Box3dShape>());
        assert_eq!(app.world().non_send::<Box3dWorld>().shapes.len(), 0);

        app.world_mut().entity_mut(entity).remove::<RigidBody>();
        for _ in 0..3 {
            app.update();
        }
        assert!(!app.world().entity(entity).contains::<Box3dBody>());
        assert_eq!(app.world().non_send::<Box3dWorld>().bodies.len(), 0);
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
            ..Box3dPlugin::default()
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

    #[test]
    fn debug_transform_composes_child_offset_without_scale() {
        let mut body = bevy_transform::prelude::Transform::from_xyz(1.0, 2.0, 3.0);
        body.rotation = bevy_math::Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
        body.scale = Vec3::splat(10.0);

        let local = bevy_transform::prelude::Transform::from_xyz(1.0, 0.0, 0.0);
        let transform = collider_debug_transform(body, local);

        assert!((transform.translation - Vec3::new(1.0, 3.0, 3.0)).length() < 0.0001);
        assert_eq!(transform.scale, Vec3::ONE);
    }
}
