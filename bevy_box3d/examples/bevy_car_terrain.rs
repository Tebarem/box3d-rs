use std::time::{SystemTime, UNIX_EPOCH};

use bevy::{
    asset::RenderAssetUsages,
    diagnostic::{DiagnosticsStore, EntityCountDiagnosticsPlugin, FrameTimeDiagnosticsPlugin},
    math::primitives::{Cuboid, Sphere},
    mesh::Indices,
    prelude::*,
    render::render_resource::PrimitiveTopology,
};
use bevy_box3d::{
    Box3dBody, Box3dConfig, Box3dPlugin, Box3dSet, Box3dStats, Box3dWorld, Collider, FastRotation,
    RigidBody,
};
use box3d::{
    compute_quat_between_unit_vectors, Capacity, JointId, MeshCreateOptions, ParallelJointIdDef,
    Quat as BoxQuat, SurfaceMaterial, Transform as BoxTransform, Vec3 as BoxVec3, WheelJointIdDef,
};

const PHYSICS_TICK_RATE: f32 = 60.0;
const PHYSICS_SUB_STEPS: i32 = 4;
const TERRAIN_POINTS: usize = 73;
const TERRAIN_CELL: f32 = 1.25;
const TERRAIN_AMPLITUDE: f32 = 2.6;
const CHASSIS_HALF_EXTENTS: Vec3 = Vec3::new(2.0, 0.5, 1.0);
const WHEEL_RADIUS: f32 = 0.4;
const SPIN_SPEED: f32 = 30.0;
const MAX_SPIN_TORQUE: f32 = 5.0;
const SUSPENSION_HERTZ: f32 = 4.0;
const SUSPENSION_DAMPING_RATIO: f32 = 0.7;
const SUSPENSION_LOWER: f32 = -0.2;
const SUSPENSION_UPPER: f32 = 0.2;
const STEERING_HERTZ: f32 = 10.0;
const STEERING_DAMPING_RATIO: f32 = 0.7;
const MAX_STEERING_TORQUE: f32 = 5.0;
const MAX_STEERING_ANGLE: f32 = std::f32::consts::FRAC_PI_4;

#[derive(Component)]
struct TerrainBody;

#[derive(Component)]
struct VehicleChassis;

#[derive(Clone, Copy, Component)]
struct VehicleWheel {
    local_anchor: Vec3,
    front: bool,
}

#[derive(Component)]
struct FollowCamera;

#[derive(Component)]
struct StatsText;

#[derive(Resource)]
struct TerrainSeed(u64);

#[derive(Default, Resource)]
struct VehicleJoints {
    upright: Option<JointId>,
    wheels: Vec<VehicleWheelJoint>,
}

struct VehicleWheelJoint {
    id: JointId,
    front: bool,
}

impl Drop for VehicleJoints {
    fn drop(&mut self) {
        for wheel in &self.wheels {
            wheel.id.destroy(true);
        }
        if let Some(upright) = self.upright {
            upright.destroy(true);
        }
    }
}

struct TerrainData {
    vertices: Vec<Vec3>,
    indices: Vec<u32>,
    mesh: Mesh,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "box3d Bevy wheel-joint car".to_owned(),
                resolution: (1280, 720).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins((
            FrameTimeDiagnosticsPlugin::default(),
            EntityCountDiagnosticsPlugin::default(),
            Box3dPlugin {
                config: Box3dConfig {
                    fixed_hz: PHYSICS_TICK_RATE as f64,
                    sub_steps: PHYSICS_SUB_STEPS,
                    capacity: Capacity {
                        static_shape_count: 1,
                        dynamic_shape_count: 5,
                        static_body_count: 1,
                        dynamic_body_count: 5,
                        contact_count: 512,
                    },
                    ..default()
                },
                ..default()
            },
        ))
        .insert_resource(TerrainSeed(new_seed()))
        .insert_resource(VehicleJoints::default())
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            (setup_vehicle_joints, drive_vehicle)
                .chain()
                .after(Box3dSet::Sync)
                .before(Box3dSet::Step),
        )
        .add_systems(Update, (follow_camera, update_stats))
        .run();
}

fn setup(
    mut commands: Commands,
    seed: Res<TerrainSeed>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-8.0, 6.0, 10.0).looking_at(Vec3::Y * 1.5, Vec3::Y),
        FollowCamera,
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 18_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.1, -0.75, 0.0)),
    ));

    let terrain = generate_terrain(seed.0);
    commands.spawn((
        RigidBody::Static,
        Collider::mesh_with_options(
            terrain.vertices.clone(),
            terrain.indices.clone(),
            Vec3::ONE,
            MeshCreateOptions {
                use_median_split: true,
                ..default()
            },
        )
        .with_surface_material(SurfaceMaterial {
            friction: 1.0,
            restitution: 0.02,
            rolling_resistance: 0.04,
            ..default()
        }),
        Mesh3d(meshes.add(terrain.mesh)),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.24, 0.45, 0.25),
            perceptual_roughness: 0.95,
            ..default()
        })),
        Transform::default(),
        TerrainBody,
    ));

    let start = Vec3::new(0.0, terrain_height(seed.0, 0.0, 0.0) + 2.5, 0.0);
    commands.spawn((
        RigidBody::Dynamic,
        Collider::cuboid(CHASSIS_HALF_EXTENTS).with_density(0.5),
        Mesh3d(meshes.add(Cuboid::new(
            CHASSIS_HALF_EXTENTS.x * 2.0,
            CHASSIS_HALF_EXTENTS.y * 2.0,
            CHASSIS_HALF_EXTENTS.z * 2.0,
        ))),
        MeshMaterial3d(materials.add(Color::srgb(0.86, 0.12, 0.08))),
        Transform::from_translation(start),
        VehicleChassis,
    ));

    let wheel_mesh = meshes.add(Sphere::new(WHEEL_RADIUS));
    let wheel_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.02, 0.02, 0.025),
        perceptual_roughness: 0.9,
        ..default()
    });

    for (front, x) in [(true, 1.5), (false, -1.5)] {
        for z in [0.8, -0.8] {
            let local_anchor = Vec3::new(x, -0.5, z);
            commands.spawn((
                RigidBody::Dynamic,
                Collider::sphere(WHEEL_RADIUS)
                    .with_density(2.0)
                    .with_surface_material(SurfaceMaterial {
                        friction: 3.0,
                        restitution: 0.0,
                        rolling_resistance: 0.03,
                        ..default()
                    }),
                FastRotation,
                Mesh3d(wheel_mesh.clone()),
                MeshMaterial3d(wheel_material.clone()),
                Transform::from_translation(start + local_anchor)
                    .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
                VehicleWheel {
                    local_anchor,
                    front,
                },
            ));
        }
    }

    commands.spawn((
        Text::new("stats"),
        TextFont {
            font_size: FontSize::Px(16.0),
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
        StatsText,
    ));
}

fn setup_vehicle_joints(
    mut joints: ResMut<VehicleJoints>,
    physics: NonSend<Box3dWorld>,
    ground: Query<&Box3dBody, With<TerrainBody>>,
    chassis: Query<&Box3dBody, With<VehicleChassis>>,
    wheels: Query<(&Box3dBody, &VehicleWheel)>,
) {
    if joints.upright.is_some() {
        return;
    }

    let Ok(ground) = ground.single() else {
        return;
    };
    let Ok(chassis) = chassis.single() else {
        return;
    };
    if wheels.iter().count() != 4 {
        return;
    }

    let upright_rotation = axis_rotation(Vec3::Z, Vec3::Y);
    let mut upright_def = ParallelJointIdDef::new(ground.id, chassis.id);
    upright_def.local_frame_a.q = upright_rotation;
    upright_def.local_frame_b.q = upright_rotation;
    upright_def.collide_connected = true;
    upright_def.hertz = 0.5;
    upright_def.damping_ratio = 1.0;
    joints.upright = Some(physics.world().create_parallel_joint_id(upright_def));

    let frame_a_rotation = axis_rotation(Vec3::X, Vec3::Y);
    let frame_b_rotation = axis_rotation(Vec3::Z, Vec3::Y);
    for (body, wheel) in &wheels {
        let mut def = WheelJointIdDef::new(chassis.id, body.id);
        def.local_frame_a = BoxTransform::new(box_vec3(wheel.local_anchor), frame_a_rotation);
        def.local_frame_b = BoxTransform::new(BoxVec3::ZERO, frame_b_rotation);
        def.enable_suspension_limit = true;
        def.lower_suspension_limit = SUSPENSION_LOWER;
        def.upper_suspension_limit = SUSPENSION_UPPER;
        def.enable_suspension_spring = true;
        def.suspension_hertz = SUSPENSION_HERTZ;
        def.suspension_damping_ratio = SUSPENSION_DAMPING_RATIO;
        def.enable_spin_motor = !wheel.front;
        def.max_spin_torque = MAX_SPIN_TORQUE;
        def.enable_steering = wheel.front;
        def.steering_hertz = STEERING_HERTZ;
        def.steering_damping_ratio = STEERING_DAMPING_RATIO;
        def.max_steering_torque = MAX_STEERING_TORQUE;
        def.enable_steering_limit = true;
        def.lower_steering_limit = -MAX_STEERING_ANGLE;
        def.upper_steering_limit = MAX_STEERING_ANGLE;

        joints.wheels.push(VehicleWheelJoint {
            id: physics.world().create_wheel_joint_id(def),
            front: wheel.front,
        });
    }
}

fn drive_vehicle(
    keyboard: Res<ButtonInput<KeyCode>>,
    joints: Res<VehicleJoints>,
    chassis: Query<&Box3dBody, With<VehicleChassis>>,
) {
    let Ok(chassis) = chassis.single() else {
        return;
    };
    if joints.wheels.len() != 4 {
        return;
    }

    let mut throttle = 0.0;
    if keyboard.pressed(KeyCode::KeyW) || keyboard.pressed(KeyCode::ArrowUp) {
        throttle += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyS) || keyboard.pressed(KeyCode::ArrowDown) {
        throttle -= 1.0;
    }

    let mut steering = 0.0;
    if keyboard.pressed(KeyCode::KeyA) || keyboard.pressed(KeyCode::ArrowLeft) {
        steering += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyD) || keyboard.pressed(KeyCode::ArrowRight) {
        steering -= 1.0;
    }

    if throttle != 0.0 || steering != 0.0 {
        chassis.id.wake();
    }

    for wheel in &joints.wheels {
        if wheel.front {
            wheel
                .id
                .set_wheel_target_steering_angle(MAX_STEERING_ANGLE * steering);
        } else {
            wheel.id.set_wheel_spin_motor_speed(-SPIN_SPEED * throttle);
        }
    }
}

fn follow_camera(
    time: Res<Time>,
    chassis: Query<&Transform, With<VehicleChassis>>,
    mut camera: Query<&mut Transform, (With<FollowCamera>, Without<VehicleChassis>)>,
) {
    let Ok(chassis) = chassis.single() else {
        return;
    };
    let Ok(mut camera) = camera.single_mut() else {
        return;
    };

    let forward = chassis.rotation * Vec3::X;
    let target = chassis.translation - forward * 10.0 + Vec3::Y * 4.5;
    let alpha = 1.0 - (-8.0 * time.delta_secs()).exp();
    camera.translation = camera.translation.lerp(target, alpha);
    camera.look_at(chassis.translation + Vec3::Y * 0.8, Vec3::Y);
}

fn update_stats(
    seed: Res<TerrainSeed>,
    diagnostics: Res<DiagnosticsStore>,
    physics: Res<Box3dStats>,
    joints: Res<VehicleJoints>,
    mut text: Single<&mut Text, With<StatsText>>,
) {
    let fps = diagnostic_average(&diagnostics, &FrameTimeDiagnosticsPlugin::FPS);
    let frame_ms = diagnostic_average(&diagnostics, &FrameTimeDiagnosticsPlugin::FRAME_TIME);
    let entities = diagnostic_average(&diagnostics, &EntityCountDiagnosticsPlugin::ENTITY_COUNT);
    text.0 = format!(
        "seed: {}\nfps: {fps:.0}\nrender frame: {frame_ms:.2} ms\nphysics: {:.2} ms native {:.2} ms\nbodies: {} contacts: {}\njoints: {}\nentities: {entities:.0}",
        seed.0,
        physics.step_ms,
        physics.native_step_ms,
        physics.body_count,
        physics.contact_count,
        joints.wheels.len() + usize::from(joints.upright.is_some()),
    );
}

fn axis_rotation(from: Vec3, to: Vec3) -> BoxQuat {
    compute_quat_between_unit_vectors(box_vec3(from.normalize()), box_vec3(to.normalize()))
}

fn box_vec3(value: Vec3) -> BoxVec3 {
    BoxVec3::new(value.x, value.y, value.z)
}

fn generate_terrain(seed: u64) -> TerrainData {
    let half = (TERRAIN_POINTS as f32 - 1.0) * TERRAIN_CELL * 0.5;
    let mut vertices = Vec::with_capacity(TERRAIN_POINTS * TERRAIN_POINTS);
    let mut uvs = Vec::with_capacity(TERRAIN_POINTS * TERRAIN_POINTS);

    for z in 0..TERRAIN_POINTS {
        for x in 0..TERRAIN_POINTS {
            let world_x = x as f32 * TERRAIN_CELL - half;
            let world_z = z as f32 * TERRAIN_CELL - half;
            vertices.push(Vec3::new(
                world_x,
                terrain_height(seed, world_x, world_z),
                world_z,
            ));
            uvs.push([
                x as f32 / (TERRAIN_POINTS - 1) as f32,
                z as f32 / (TERRAIN_POINTS - 1) as f32,
            ]);
        }
    }

    let mut indices = Vec::with_capacity((TERRAIN_POINTS - 1) * (TERRAIN_POINTS - 1) * 6);
    for z in 0..TERRAIN_POINTS - 1 {
        for x in 0..TERRAIN_POINTS - 1 {
            let i0 = (z * TERRAIN_POINTS + x) as u32;
            let i1 = i0 + 1;
            let i2 = i0 + TERRAIN_POINTS as u32;
            let i3 = i2 + 1;
            indices.extend([i0, i2, i1, i1, i2, i3]);
        }
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vertices.clone());
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_indices(Indices::U32(indices.clone()));
    mesh.compute_smooth_normals();

    TerrainData {
        vertices,
        indices,
        mesh,
    }
}

fn terrain_height(seed: u64, x: f32, z: f32) -> f32 {
    let mut total = 0.0;
    let mut amplitude = 1.0;
    let mut frequency = 0.045;
    let mut norm = 0.0;

    for octave in 0..5 {
        total += value_noise(seed.wrapping_add(octave), x * frequency, z * frequency) * amplitude;
        norm += amplitude;
        amplitude *= 0.52;
        frequency *= 2.05;
    }

    let ridge = (value_noise(seed ^ 0x9e37_79b9, x * 0.025, z * 0.025).abs() * -1.4) + 0.7;
    (total / norm + ridge * 0.35) * TERRAIN_AMPLITUDE
}

fn value_noise(seed: u64, x: f32, z: f32) -> f32 {
    let x0 = x.floor() as i32;
    let z0 = z.floor() as i32;
    let tx = smoothstep(x - x.floor());
    let tz = smoothstep(z - z.floor());
    let a = hash_noise(seed, x0, z0);
    let b = hash_noise(seed, x0 + 1, z0);
    let c = hash_noise(seed, x0, z0 + 1);
    let d = hash_noise(seed, x0 + 1, z0 + 1);
    lerp(lerp(a, b, tx), lerp(c, d, tx), tz)
}

fn hash_noise(seed: u64, x: i32, z: i32) -> f32 {
    let mut value = seed ^ (x as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15);
    value ^= (z as u64).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value ^= value >> 30;
    value = value.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    value ^= value >> 27;
    value = value.wrapping_mul(0x94d0_49bb_1331_11eb);
    value ^= value >> 31;
    (value as f64 / u64::MAX as f64) as f32 * 2.0 - 1.0
}

fn smoothstep(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

fn new_seed() -> u64 {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_nanos() as u64)
        .unwrap_or(0);
    nanos ^ u64::from(std::process::id())
}

fn diagnostic_average(
    diagnostics: &DiagnosticsStore,
    path: &bevy::diagnostic::DiagnosticPath,
) -> f64 {
    diagnostics
        .get(path)
        .and_then(|diagnostic| diagnostic.average())
        .unwrap_or(0.0)
}
