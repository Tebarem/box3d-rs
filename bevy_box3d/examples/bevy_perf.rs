use std::{env, time::Instant};

use bevy_app::App;
use bevy_box3d::{
    Box3dConfig, Box3dPlugin, Box3dStats, Collider, ColliderParent, Damping, RigidBody,
    SleepThreshold, Velocity,
};
use bevy_math::{Quat, Vec3};
use bevy_time::{TimePlugin, TimeUpdateStrategy};
use bevy_transform::prelude::Transform;
use box3d::{Capacity, SurfaceMaterial};

const PHYSICS_TICK_RATE: f32 = 60.0;
const PHYSICS_SUB_STEPS: i32 = 4;
const PHYSICS_WORKERS: u32 = 8;
const CONTACTS_PER_BODY: usize = 16;
const BALL_RADIUS: f32 = 0.16;
const BALL_DENSITY: f32 = 3.0;
const BALL_RESTITUTION: f32 = 0.05;
const BALL_GRID_WIDTH: usize = 10;
const BALL_SPACING: f32 = BALL_RADIUS * 2.4;
const BALL_SPAWN_HEIGHT: f32 = 8.0;
const BALLS_PER_BATCH: usize = 100;
const WALL_SEGMENTS: usize = 32;
const WALL_RADIUS: f32 = 7.0;
const WALL_HEIGHT: f32 = 2.6;
const WAVE_STEPS: usize = 30;
const WARMUP_STEPS: usize = 600;
const MEASURE_STEPS: usize = 120;

fn main() {
    let ball_count = arg_usize(1).unwrap_or(2_000);
    let measure_steps = arg_usize(2).unwrap_or(MEASURE_STEPS);
    let warmup_steps = arg_usize(3).unwrap_or(WARMUP_STEPS);
    let wave_steps = arg_usize(4).unwrap_or(WAVE_STEPS).max(1);
    let worker_count = arg_usize(5)
        .map(|count| count.clamp(1, box3d::MAX_WORKERS as usize) as u32)
        .unwrap_or(PHYSICS_WORKERS);
    let contact_recycle_distance =
        arg_f32(6).filter(|distance| distance.is_finite() && *distance >= 0.0);
    let sleep_threshold = arg_f32(7).filter(|threshold| threshold.is_finite() && *threshold >= 0.0);
    let ball_restitution = arg_f32(8)
        .filter(|restitution| restitution.is_finite() && *restitution >= 0.0)
        .unwrap_or(BALL_RESTITUTION);
    let invoke_contact_creation = arg_bool(9).unwrap_or(false);
    let sub_steps = arg_i32(10)
        .filter(|sub_steps| *sub_steps >= 1)
        .unwrap_or(PHYSICS_SUB_STEPS);
    let same_spawn = arg_bool(11).unwrap_or(false);

    println!(
        "balls: {ball_count}, measured steps: {measure_steps}, warmup: {warmup_steps}, wave steps: {wave_steps}, workers: {worker_count}, substeps: {sub_steps}, same spawn: {same_spawn}, contact recycle: {}, sleep threshold: {}, restitution: {ball_restitution:.3}, invoke contact creation: {invoke_contact_creation}",
        recycle_label(contact_recycle_distance),
        threshold_label(sleep_threshold),
    );

    let active = run_active_waves(
        ball_count,
        wave_steps,
        worker_count,
        contact_recycle_distance,
        sleep_threshold,
        ball_restitution,
        invoke_contact_creation,
        sub_steps,
        same_spawn,
    );
    println!(
        "active: spawn avg {:.3} ms max {:.3} ms, app avg {:.3} ms max {:.3} ms, step avg {:.3} ms max {:.3} ms, native avg {:.3} ms max {:.3} ms, pairs/collide/solve max {:.3}/{:.3}/{:.3} ms, tasks max {}, moves max {}, bodies {}, contact pairs {}, touched max {}, manifolds max {}, overflow max {}, recycled max {}, sat max {}, alloc {} KB",
        active.spawn_avg_ms(),
        active.max_spawn_ms,
        active.app_avg_ms(),
        active.max_app_ms,
        active.step_avg_ms(),
        active.max_step_ms,
        active.native_avg_ms(),
        active.max_native_ms,
        active.max_pairs_ms,
        active.max_collide_ms,
        active.max_solve_ms,
        active.max_tasks,
        active.max_moves,
        active.bodies,
        active.contacts,
        active.max_awake_contacts,
        active.max_manifolds,
        active.max_overflow_constraints,
        active.max_recycled_contacts,
        active.max_sat_calls,
        active.max_bytes / 1024,
    );
    active.print_profile("active");

    let settled = run_settled(
        ball_count,
        warmup_steps,
        measure_steps,
        worker_count,
        contact_recycle_distance,
        sleep_threshold,
        ball_restitution,
        invoke_contact_creation,
        sub_steps,
        same_spawn,
    );
    println!(
        "settled: spawn avg {:.3} ms max {:.3} ms, app avg {:.3} ms max {:.3} ms, step avg {:.3} ms max {:.3} ms, native avg {:.3} ms max {:.3} ms, pairs/collide/solve max {:.3}/{:.3}/{:.3} ms, tasks max {}, moves max {}, bodies {}, contact pairs {}, touched max {}, manifolds max {}, overflow max {}, recycled max {}, sat max {}, alloc {} KB",
        settled.spawn_avg_ms(),
        settled.max_spawn_ms,
        settled.app_avg_ms(),
        settled.max_app_ms,
        settled.step_avg_ms(),
        settled.max_step_ms,
        settled.native_avg_ms(),
        settled.max_native_ms,
        settled.max_pairs_ms,
        settled.max_collide_ms,
        settled.max_solve_ms,
        settled.max_tasks,
        settled.max_moves,
        settled.bodies,
        settled.contacts,
        settled.max_awake_contacts,
        settled.max_manifolds,
        settled.max_overflow_constraints,
        settled.max_recycled_contacts,
        settled.max_sat_calls,
        settled.max_bytes / 1024,
    );
    settled.print_profile("settled");
}

#[derive(Default)]
struct Timing {
    spawn_batches: usize,
    spawn_ms: f64,
    max_spawn_ms: f64,
    frames: usize,
    app_ms: f64,
    max_app_ms: f64,
    step_ms: f64,
    max_step_ms: f64,
    native_ms: f64,
    max_native_ms: f32,
    max_pairs_ms: f32,
    max_collide_ms: f32,
    max_solve_ms: f32,
    max_refit_ms: f32,
    max_sleep_ms: f32,
    max_solver_setup_ms: f32,
    max_constraints_ms: f32,
    max_solve_impulses_ms: f32,
    max_relax_impulses_ms: f32,
    max_integrate_velocities_ms: f32,
    max_integrate_positions_ms: f32,
    max_warm_start_ms: f32,
    max_restitution_ms: f32,
    max_tasks: usize,
    max_bytes: usize,
    max_moves: usize,
    bodies: usize,
    contacts: usize,
    max_awake_contacts: usize,
    max_manifolds: usize,
    max_overflow_constraints: usize,
    max_recycled_contacts: usize,
    max_sat_calls: usize,
}

impl Timing {
    fn record_spawn(&mut self, spawn_ms: f64) {
        self.spawn_batches += 1;
        self.spawn_ms += spawn_ms;
        self.max_spawn_ms = self.max_spawn_ms.max(spawn_ms);
    }

    fn record(&mut self, app_ms: f64, stats: Box3dStats) {
        self.frames += 1;
        self.app_ms += app_ms;
        self.max_app_ms = self.max_app_ms.max(app_ms);
        self.step_ms += stats.step_ms;
        self.max_step_ms = self.max_step_ms.max(stats.step_ms);
        self.native_ms += f64::from(stats.native_step_ms);
        self.max_native_ms = self.max_native_ms.max(stats.native_step_ms);
        self.max_pairs_ms = self.max_pairs_ms.max(stats.native_pairs_ms);
        self.max_collide_ms = self.max_collide_ms.max(stats.native_collide_ms);
        self.max_solve_ms = self.max_solve_ms.max(stats.native_solve_ms);
        self.max_refit_ms = self.max_refit_ms.max(stats.native_profile.refit);
        self.max_sleep_ms = self.max_sleep_ms.max(stats.native_profile.sleep_islands);
        self.max_solver_setup_ms = self
            .max_solver_setup_ms
            .max(stats.native_profile.solver_setup);
        self.max_constraints_ms = self
            .max_constraints_ms
            .max(stats.native_profile.constraints);
        self.max_solve_impulses_ms = self
            .max_solve_impulses_ms
            .max(stats.native_profile.solve_impulses);
        self.max_relax_impulses_ms = self
            .max_relax_impulses_ms
            .max(stats.native_profile.relax_impulses);
        self.max_integrate_velocities_ms = self
            .max_integrate_velocities_ms
            .max(stats.native_profile.integrate_velocities);
        self.max_integrate_positions_ms = self
            .max_integrate_positions_ms
            .max(stats.native_profile.integrate_positions);
        self.max_warm_start_ms = self.max_warm_start_ms.max(stats.native_profile.warm_start);
        self.max_restitution_ms = self
            .max_restitution_ms
            .max(stats.native_profile.apply_restitution);
        self.max_tasks = self.max_tasks.max(stats.task_count);
        self.max_bytes = self.max_bytes.max(stats.byte_count);
        self.max_moves = self.max_moves.max(stats.move_event_count);
        self.bodies = stats.body_count;
        self.contacts = stats.contact_count;
        self.max_awake_contacts = self.max_awake_contacts.max(stats.awake_contact_count);
        self.max_manifolds = self.max_manifolds.max(stats.manifold_count);
        self.max_overflow_constraints = self
            .max_overflow_constraints
            .max(stats.overflow_constraint_count);
        self.max_recycled_contacts = self.max_recycled_contacts.max(stats.recycled_contact_count);
        self.max_sat_calls = self.max_sat_calls.max(stats.sat_call_count);
    }

    fn app_avg_ms(&self) -> f64 {
        self.app_ms / self.frames.max(1) as f64
    }

    fn step_avg_ms(&self) -> f64 {
        self.step_ms / self.frames.max(1) as f64
    }

    fn native_avg_ms(&self) -> f64 {
        self.native_ms / self.frames.max(1) as f64
    }

    fn spawn_avg_ms(&self) -> f64 {
        self.spawn_ms / self.spawn_batches.max(1) as f64
    }

    fn print_profile(&self, label: &str) {
        println!(
            "{label} profile max: refit/sleep {:.3}/{:.3} ms, solver setup/constraints/impulses/relax {:.3}/{:.3}/{:.3}/{:.3} ms, integrate vel/pos/warm/rest {:.3}/{:.3}/{:.3}/{:.3} ms",
            self.max_refit_ms,
            self.max_sleep_ms,
            self.max_solver_setup_ms,
            self.max_constraints_ms,
            self.max_solve_impulses_ms,
            self.max_relax_impulses_ms,
            self.max_integrate_velocities_ms,
            self.max_integrate_positions_ms,
            self.max_warm_start_ms,
            self.max_restitution_ms,
        );
    }
}

#[allow(clippy::too_many_arguments)]
fn run_active_waves(
    ball_count: usize,
    wave_steps: usize,
    worker_count: u32,
    contact_recycle_distance: Option<f32>,
    sleep_threshold: Option<f32>,
    ball_restitution: f32,
    invoke_contact_creation: bool,
    sub_steps: i32,
    same_spawn: bool,
) -> Timing {
    let mut app = build_app(
        ball_count,
        worker_count,
        contact_recycle_distance,
        sub_steps,
    );
    spawn_static_scene(&mut app);
    app.update();

    let mut timing = Timing::default();
    let mut spawned = 0;
    while spawned < ball_count {
        let count = BALLS_PER_BATCH.min(ball_count - spawned);
        let spawn_started = Instant::now();
        spawn_ball_batch(
            &mut app,
            if same_spawn { 0 } else { spawned },
            count,
            sleep_threshold,
            ball_restitution,
            invoke_contact_creation,
        );
        timing.record_spawn(spawn_started.elapsed().as_secs_f64() * 1000.0);
        spawned += count;

        for _ in 0..wave_steps {
            record_update(&mut app, &mut timing);
        }
    }
    timing
}

#[allow(clippy::too_many_arguments)]
fn run_settled(
    ball_count: usize,
    warmup_steps: usize,
    measure_steps: usize,
    worker_count: u32,
    contact_recycle_distance: Option<f32>,
    sleep_threshold: Option<f32>,
    ball_restitution: f32,
    invoke_contact_creation: bool,
    sub_steps: i32,
    same_spawn: bool,
) -> Timing {
    let mut app = build_app(
        ball_count,
        worker_count,
        contact_recycle_distance,
        sub_steps,
    );
    spawn_static_scene(&mut app);

    let mut timing = Timing::default();
    let mut spawned = 0;
    while spawned < ball_count {
        let count = BALLS_PER_BATCH.min(ball_count - spawned);
        let spawn_started = Instant::now();
        spawn_ball_batch(
            &mut app,
            if same_spawn { 0 } else { spawned },
            count,
            sleep_threshold,
            ball_restitution,
            invoke_contact_creation,
        );
        timing.record_spawn(spawn_started.elapsed().as_secs_f64() * 1000.0);
        spawned += count;
    }

    for _ in 0..warmup_steps {
        app.update();
    }

    for _ in 0..measure_steps {
        record_update(&mut app, &mut timing);
    }
    timing
}

fn record_update(app: &mut App, timing: &mut Timing) {
    let started = Instant::now();
    app.update();
    let app_ms = started.elapsed().as_secs_f64() * 1000.0;
    timing.record(app_ms, *app.world().resource::<Box3dStats>());
}

fn build_app(
    ball_count: usize,
    worker_count: u32,
    contact_recycle_distance: Option<f32>,
    sub_steps: i32,
) -> App {
    let mut app = App::new();
    app.add_plugins(TimePlugin)
        .insert_resource(TimeUpdateStrategy::FixedTimesteps(1));
    let body_capacity = body_capacity(ball_count);
    app.add_plugins(Box3dPlugin {
        config: Box3dConfig {
            fixed_hz: PHYSICS_TICK_RATE as f64,
            sub_steps,
            worker_count,
            capacity: Capacity {
                static_shape_count: WALL_SEGMENTS as i32 + 4,
                dynamic_shape_count: body_capacity,
                static_body_count: WALL_SEGMENTS as i32 + 2,
                dynamic_body_count: body_capacity,
                contact_count: contact_capacity(ball_count),
            },
            sleeping_enabled: true,
            continuous_enabled: true,
            contact_recycle_distance,
            ..Box3dConfig::default()
        },
        ..Box3dPlugin::default()
    });
    app
}

fn spawn_static_scene(app: &mut App) {
    app.world_mut().spawn((
        RigidBody::Static,
        Collider::cuboid(Vec3::new(8.0, 0.25, 8.0)).with_surface_material(SurfaceMaterial {
            friction: 1.0,
            restitution: 0.05,
            rolling_resistance: 0.05,
            ..SurfaceMaterial::default()
        }),
        Transform::from_xyz(0.0, -0.25, 0.0),
    ));

    let segment_length = std::f32::consts::TAU * WALL_RADIUS / WALL_SEGMENTS as f32 * 0.92;
    for i in 0..WALL_SEGMENTS {
        let angle = i as f32 / WALL_SEGMENTS as f32 * std::f32::consts::TAU;
        let x = angle.cos() * WALL_RADIUS;
        let z = angle.sin() * WALL_RADIUS;
        app.world_mut().spawn((
            RigidBody::Static,
            Collider::cuboid(Vec3::new(segment_length * 0.5, WALL_HEIGHT * 0.5, 0.25))
                .with_surface_material(SurfaceMaterial {
                    friction: 0.9,
                    restitution: 0.1,
                    ..SurfaceMaterial::default()
                }),
            Transform {
                translation: Vec3::new(x, WALL_HEIGHT * 0.5, z),
                rotation: Quat::from_rotation_y(std::f32::consts::FRAC_PI_2 - angle),
                ..Transform::default()
            },
        ));
    }

    let propeller = app
        .world_mut()
        .spawn((
            RigidBody::Kinematic,
            Velocity {
                linear: Vec3::ZERO,
                angular: Vec3::Y * 9.0,
            },
            Transform::from_xyz(0.0, 0.55, 0.0),
        ))
        .id();

    for rotation in [0.0, std::f32::consts::FRAC_PI_2] {
        app.world_mut().spawn((
            ColliderParent(propeller),
            Collider::cuboid(Vec3::new(2.6, 0.125, 0.21)).with_surface_material(SurfaceMaterial {
                friction: 0.7,
                restitution: 0.25,
                ..SurfaceMaterial::default()
            }),
            Transform {
                rotation: Quat::from_rotation_y(rotation),
                ..Transform::default()
            },
        ));
    }
}

fn spawn_ball_batch(
    app: &mut App,
    start_index: usize,
    ball_count: usize,
    sleep_threshold: Option<f32>,
    ball_restitution: f32,
    invoke_contact_creation: bool,
) {
    let collider = Collider::sphere(BALL_RADIUS)
        .with_density(BALL_DENSITY)
        .invoke_contact_creation(invoke_contact_creation)
        .with_surface_material(SurfaceMaterial {
            friction: 0.75,
            restitution: ball_restitution,
            rolling_resistance: 0.03,
            ..SurfaceMaterial::default()
        });
    let damping = Damping {
        linear: 0.02,
        angular: 0.02,
    };

    if let Some(threshold) = sleep_threshold {
        app.world_mut().spawn_batch((0..ball_count).map(move |i| {
            (
                RigidBody::Dynamic,
                collider,
                damping,
                SleepThreshold(threshold),
                Transform::from_translation(ball_spawn_position(start_index + i)),
            )
        }));
    } else {
        app.world_mut().spawn_batch((0..ball_count).map(move |i| {
            (
                RigidBody::Dynamic,
                collider,
                damping,
                Transform::from_translation(ball_spawn_position(start_index + i)),
            )
        }));
    }
}

fn arg_usize(index: usize) -> Option<usize> {
    env::args().nth(index)?.parse().ok()
}

fn arg_f32(index: usize) -> Option<f32> {
    env::args().nth(index)?.parse().ok()
}

fn arg_i32(index: usize) -> Option<i32> {
    env::args().nth(index)?.parse().ok()
}

fn arg_bool(index: usize) -> Option<bool> {
    match env::args().nth(index)?.to_ascii_lowercase().as_str() {
        "1" | "true" | "yes" | "on" => Some(true),
        "0" | "false" | "no" | "off" => Some(false),
        _ => None,
    }
}

fn recycle_label(distance: Option<f32>) -> String {
    distance
        .map(|distance| format!("{distance:.3}"))
        .unwrap_or_else(|| "native default".to_string())
}

fn contact_capacity(ball_count: usize) -> i32 {
    ball_count
        .saturating_mul(CONTACTS_PER_BODY)
        .min(i32::MAX as usize) as i32
}

fn body_capacity(ball_count: usize) -> i32 {
    ball_count.saturating_add(2).min(i32::MAX as usize) as i32
}

fn threshold_label(threshold: Option<f32>) -> String {
    threshold
        .map(|threshold| format!("{threshold:.3}"))
        .unwrap_or_else(|| "native default".to_string())
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
