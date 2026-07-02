use bevy::{
    diagnostic::{DiagnosticsStore, EntityCountDiagnosticsPlugin, FrameTimeDiagnosticsPlugin},
    math::primitives::{Cuboid, Sphere},
    prelude::*,
};
use bevy_box3d::{
    Box3dConfig, Box3dDebugPlugin, Box3dPlugin, Box3dStats, Collider, Damping, RigidBody, Velocity,
};
use box3d::SurfaceMaterial;

const HALF_EXTENTS: Vec3 = Vec3::new(0.5, 0.5, 0.5);
const PHYSICS_TICK_RATE: f32 = 60.0;
const PHYSICS_SUB_STEPS: i32 = 4;
const BALL_RADIUS: f32 = 0.3;
const BALL_DENSITY: f32 = 8.0;
const BALL_SPEED: f32 = 24.0;
const BALL_FRICTION: f32 = 0.9;
const BALL_RESTITUTION: f32 = 0.45;
const BALL_ROLLING_RESISTANCE: f32 = 0.08;
const BOX_FRICTION: f32 = 1.1;
const GROUND_FRICTION: f32 = 1.4;

#[derive(Component)]
struct StatsText;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "box3d Bevy stack".to_owned(),
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
                    continuous_enabled: false,
                    ..default()
                },
            },
            Box3dDebugPlugin::default(),
        ))
        .add_systems(Startup, setup)
        .add_systems(PreUpdate, throw_ball)
        .add_systems(Update, update_stats)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(8.0, 8.0, 12.0).looking_at(Vec3::new(0.0, 4.0, 0.0), Vec3::Y),
    ));
    commands.spawn((
        PointLight {
            intensity: 5_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 10.0, 6.0),
    ));

    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(Vec3::new(8.0, 0.5, 8.0)).with_surface_material(SurfaceMaterial {
            friction: GROUND_FRICTION,
            restitution: 0.0,
            ..default()
        }),
        Mesh3d(meshes.add(Cuboid::new(16.0, 1.0, 16.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.22, 0.24, 0.26))),
        Transform::from_xyz(0.0, -0.5, 0.0),
    ));

    let cube_mesh = meshes.add(Cuboid::from_length(1.0));
    let cube_material = materials.add(Color::srgb(0.2, 0.55, 0.95));

    for row in 0..10 {
        let y = 0.5 + row as f32 * 1.05;
        let x_offset = if row % 2 == 0 { -0.25 } else { 0.25 };
        for col in 0..4 {
            let x = (col as f32 - 1.5) * 1.05 + x_offset;
            let z = (row as f32 * 0.17).sin() * 0.2;

            commands.spawn((
                RigidBody::Dynamic,
                Collider::cuboid(HALF_EXTENTS)
                    .with_density(1.0)
                    .with_surface_material(SurfaceMaterial {
                        friction: BOX_FRICTION,
                        restitution: 0.0,
                        ..default()
                    }),
                Mesh3d(cube_mesh.clone()),
                MeshMaterial3d(cube_material.clone()),
                Transform::from_xyz(x, y, z),
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

fn throw_ball(
    mouse: Res<ButtonInput<MouseButton>>,
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform)>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !mouse.just_pressed(MouseButton::Left) {
        return;
    }

    let Some(cursor_position) = window.cursor_position() else {
        return;
    };

    let (camera, camera_transform) = *camera;
    let Ok(ray) = camera.viewport_to_world(camera_transform, cursor_position) else {
        return;
    };

    let start = ray.origin + *ray.direction * 1.5;
    let direction = *ray.direction;

    commands.spawn((
        RigidBody::Dynamic,
        Collider::sphere(BALL_RADIUS)
            .with_density(BALL_DENSITY)
            .with_surface_material(SurfaceMaterial {
                friction: BALL_FRICTION,
                restitution: BALL_RESTITUTION,
                rolling_resistance: BALL_ROLLING_RESISTANCE,
                ..default()
            }),
        Velocity::linear(direction * BALL_SPEED),
        Damping {
            linear: 0.08,
            angular: 0.06,
        },
        Mesh3d(meshes.add(Sphere::new(BALL_RADIUS))),
        MeshMaterial3d(materials.add(Color::srgb(0.95, 0.25, 0.18))),
        Transform::from_translation(start),
    ));
}

fn update_stats(
    diagnostics: Res<DiagnosticsStore>,
    physics: Res<Box3dStats>,
    mut text: Single<&mut Text, With<StatsText>>,
) {
    let fps = diagnostic_average(&diagnostics, &FrameTimeDiagnosticsPlugin::FPS);
    let frame_ms = diagnostic_average(&diagnostics, &FrameTimeDiagnosticsPlugin::FRAME_TIME);
    let entities = diagnostic_average(&diagnostics, &EntityCountDiagnosticsPlugin::ENTITY_COUNT);

    text.0 = format!(
        "fps: {fps:.0}\nrender frame: {frame_ms:.2} ms\nphysics: {:.2} ms / {} steps\nfixed tick: {PHYSICS_TICK_RATE:.0} Hz x {PHYSICS_SUB_STEPS} substeps\ninterpolation: {:.2}\nphysics bodies: {}\nentities: {entities:.0}",
        physics.step_ms,
        physics.step_count,
        physics.interpolation_alpha,
        physics.body_count
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
