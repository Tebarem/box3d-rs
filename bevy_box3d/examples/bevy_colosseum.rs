use bevy::{
    diagnostic::{DiagnosticsStore, EntityCountDiagnosticsPlugin, FrameTimeDiagnosticsPlugin},
    math::primitives::{Cuboid, Sphere},
    prelude::*,
};
use bevy_box3d::{
    Box3dConfig, Box3dPlugin, Box3dStats, Collider, ColliderParent, Damping, RigidBody, Velocity,
};
use box3d::{Capacity, SurfaceMaterial};

const PHYSICS_TICK_RATE: f32 = 60.0;
const PHYSICS_SUB_STEPS: i32 = 1;
const PHYSICS_WORKERS: u32 = 8;
const EXPECTED_DYNAMIC_BODIES: i32 = 8_096;
const CONTACTS_PER_BODY: i32 = 16;
const EXPECTED_CONTACTS: i32 = EXPECTED_DYNAMIC_BODIES * CONTACTS_PER_BODY;
const WALL_SEGMENTS: usize = 32;
const WALL_RADIUS: f32 = 7.0;
const WALL_HEIGHT: f32 = 2.6;
const BALLS_PER_CLICK: usize = 100;
const BALL_RADIUS: f32 = 0.16;
const BALL_DENSITY: f32 = 3.0;
const BALL_GRID_WIDTH: usize = 10;
const BALL_SPACING: f32 = BALL_RADIUS * 2.4;
const BALL_SPAWN_HEIGHT: f32 = 4.0;
const BALL_RESTITUTION: f32 = 0.05;
const PROPELLER_SPEED: f32 = 9.0;

#[derive(Component)]
struct StatsText;

#[derive(Resource)]
struct BallAssets {
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "box3d Bevy colosseum".to_owned(),
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
                    worker_count: PHYSICS_WORKERS,
                    capacity: Capacity {
                        static_shape_count: WALL_SEGMENTS as i32 + 4,
                        dynamic_shape_count: EXPECTED_DYNAMIC_BODIES,
                        static_body_count: WALL_SEGMENTS as i32 + 2,
                        dynamic_body_count: EXPECTED_DYNAMIC_BODIES,
                        contact_count: EXPECTED_CONTACTS,
                    },
                    sleeping_enabled: true,
                    continuous_enabled: true,
                    ..default()
                },
                ..default()
            },
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, (spawn_balls, update_stats))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(10.0, 8.0, 12.0).looking_at(Vec3::new(0.0, 1.5, 0.0), Vec3::Y),
    ));
    commands.spawn((
        PointLight {
            intensity: 7_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(0.0, 9.0, 4.0),
    ));

    spawn_colosseum(&mut commands, &mut meshes, &mut materials);
    spawn_propeller(&mut commands, &mut meshes, &mut materials);
    commands.insert_resource(BallAssets {
        mesh: meshes.add(Sphere::new(BALL_RADIUS)),
        material: materials.add(Color::srgb(0.16, 0.56, 0.95)),
    });
    spawn_stats(&mut commands);
}

fn spawn_stats(commands: &mut Commands) {
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

fn spawn_colosseum(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(Vec3::new(8.0, 0.25, 8.0)).with_surface_material(SurfaceMaterial {
            friction: 1.2,
            restitution: 0.05,
            ..default()
        }),
        Mesh3d(meshes.add(Cuboid::new(16.0, 0.5, 16.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.58, 0.47, 0.34))),
        Transform::from_xyz(0.0, -0.25, 0.0),
    ));

    let segment_length = std::f32::consts::TAU * WALL_RADIUS / WALL_SEGMENTS as f32 * 0.92;
    let wall_mesh = meshes.add(Cuboid::new(segment_length, WALL_HEIGHT, 0.5));
    let wall_material = materials.add(Color::srgb(0.68, 0.61, 0.52));
    let accent_material = materials.add(Color::srgb(0.42, 0.38, 0.34));

    for i in 0..WALL_SEGMENTS {
        let angle = i as f32 / WALL_SEGMENTS as f32 * std::f32::consts::TAU;
        let x = angle.cos() * WALL_RADIUS;
        let z = angle.sin() * WALL_RADIUS;
        let rotation = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2 - angle);
        let material = if i % 4 == 0 {
            accent_material.clone()
        } else {
            wall_material.clone()
        };

        commands.spawn((
            RigidBody::Static,
            Collider::cuboid(Vec3::new(segment_length * 0.5, WALL_HEIGHT * 0.5, 0.25))
                .with_surface_material(SurfaceMaterial {
                    friction: 0.9,
                    restitution: 0.1,
                    ..default()
                }),
            Mesh3d(wall_mesh.clone()),
            MeshMaterial3d(material),
            Transform {
                translation: Vec3::new(x, WALL_HEIGHT * 0.5, z),
                rotation,
                ..default()
            },
        ));
    }
}

fn update_stats(
    diagnostics: Res<DiagnosticsStore>,
    physics: Res<Box3dStats>,
    mut text: Single<&mut Text, With<StatsText>>,
) {
    let fps = diagnostic_average(&diagnostics, &FrameTimeDiagnosticsPlugin::FPS);
    let frame_ms = diagnostic_average(&diagnostics, &FrameTimeDiagnosticsPlugin::FRAME_TIME);
    let entities = diagnostic_average(&diagnostics, &EntityCountDiagnosticsPlugin::ENTITY_COUNT);
    let profile = physics.native_profile;

    text.0 = format!(
        "fps: {fps:.0}\nrender frame: {frame_ms:.2} ms\nphysics tick: {:.2} ms native {:.2} ms\nprofile: pairs {:.2} collide {:.2} solve {:.2} refit {:.2} sleep {:.2}\nsolver: setup {:.2} constraints {:.2} impulses {:.2} relax {:.2}\nintegrate: vel {:.2} pos {:.2} warm {:.2} rest {:.2}\nworkers/tasks: {}/{} moves: {}\nfixed tick: {PHYSICS_TICK_RATE:.0} Hz x {PHYSICS_SUB_STEPS} substeps\ninterpolation: {:.2}\nbodies: {} awake {}\ncontact pairs: {} touched {} recycled {}\nmanifolds touched: {} overflow constraints: {}\nislands: {} sat: {}/{}\nalloc: {} KB entities: {entities:.0}",
        physics.step_ms,
        physics.native_step_ms,
        profile.pairs,
        profile.collide,
        profile.solve,
        profile.refit,
        profile.sleep_islands,
        profile.solver_setup,
        profile.constraints,
        profile.solve_impulses,
        profile.relax_impulses,
        profile.integrate_velocities,
        profile.integrate_positions,
        profile.warm_start,
        profile.apply_restitution,
        physics.worker_count,
        physics.task_count,
        physics.move_event_count,
        physics.interpolation_alpha,
        physics.body_count,
        physics.awake_body_count,
        physics.contact_count,
        physics.awake_contact_count,
        physics.recycled_contact_count,
        physics.manifold_count,
        physics.overflow_constraint_count,
        physics.island_count,
        physics.sat_cache_hit_count,
        physics.sat_call_count,
        physics.byte_count / 1024
    );
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

fn spawn_propeller(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let blade_mesh = meshes.add(Cuboid::new(5.2, 0.25, 0.42));
    let blade_material = materials.add(Color::srgb(0.84, 0.12, 0.08));
    let hub_mesh = meshes.add(Sphere::new(0.38));
    let hub_material = materials.add(Color::srgb(0.08, 0.08, 0.09));
    let propeller = commands
        .spawn((
            RigidBody::Kinematic,
            Velocity {
                linear: Vec3::ZERO,
                angular: Vec3::Y * PROPELLER_SPEED,
            },
            Transform::from_xyz(0.0, 0.55, 0.0),
            GlobalTransform::default(),
            Visibility::default(),
        ))
        .id();

    commands.entity(propeller).with_children(|parent| {
        for rotation in [0.0, std::f32::consts::FRAC_PI_2] {
            parent.spawn((
                ColliderParent(propeller),
                Collider::cuboid(Vec3::new(2.6, 0.125, 0.21)).with_surface_material(
                    SurfaceMaterial {
                        friction: 0.7,
                        restitution: 0.25,
                        ..default()
                    },
                ),
                Mesh3d(blade_mesh.clone()),
                MeshMaterial3d(blade_material.clone()),
                Transform {
                    rotation: Quat::from_rotation_y(rotation),
                    ..default()
                },
            ));
        }

        parent.spawn((
            ColliderParent(propeller),
            Collider::sphere(0.38).with_surface_material(SurfaceMaterial {
                friction: 0.8,
                restitution: 0.2,
                ..default()
            }),
            Mesh3d(hub_mesh),
            MeshMaterial3d(hub_material),
            Transform::default(),
        ));
    });
}

fn spawn_balls(
    mouse: Res<ButtonInput<MouseButton>>,
    mut commands: Commands,
    ball_assets: Res<BallAssets>,
) {
    if !mouse.just_pressed(MouseButton::Left) {
        return;
    }

    let collider = Collider::sphere(BALL_RADIUS)
        .with_density(BALL_DENSITY)
        .with_surface_material(SurfaceMaterial {
            friction: 0.75,
            restitution: BALL_RESTITUTION,
            rolling_resistance: 0.03,
            ..default()
        });
    let damping = Damping {
        linear: 0.02,
        angular: 0.02,
    };
    let ball_mesh = ball_assets.mesh.clone();
    let ball_material = ball_assets.material.clone();

    commands.spawn_batch((0..BALLS_PER_CLICK).map(move |i| {
        (
            RigidBody::Dynamic,
            collider.clone(),
            damping,
            Mesh3d(ball_mesh.clone()),
            MeshMaterial3d(ball_material.clone()),
            Transform::from_translation(ball_spawn_position(i)),
        )
    }));
}

fn ball_spawn_position(index: usize) -> Vec3 {
    let x = index % BALL_GRID_WIDTH;
    let z = (index / BALL_GRID_WIDTH) % BALL_GRID_WIDTH;
    let layer = index / (BALL_GRID_WIDTH * BALL_GRID_WIDTH);
    let offset = (BALL_GRID_WIDTH as f32 - 1.0) * 0.5;

    Vec3::new(
        (x as f32 - offset) * BALL_SPACING,
        BALL_SPAWN_HEIGHT + layer as f32 * BALL_SPACING,
        (z as f32 - offset) * BALL_SPACING,
    )
}
