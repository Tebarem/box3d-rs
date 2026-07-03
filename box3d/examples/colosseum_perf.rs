use std::{env, time::Instant};

use box3d::{
    BodyDef, BodyType, Capacity, ContactTuning, Quat, ShapeDef, SurfaceMaterial, Transform, Vec3,
    World,
};

const BALL_RADIUS: f32 = 0.16;
const BALL_DENSITY: f32 = 3.0;
const BALL_RESTITUTION: f32 = 0.05;
const CONTACTS_PER_BODY: usize = 16;
const BALL_GRID_WIDTH: usize = 10;
const BALL_SPACING: f32 = BALL_RADIUS * 2.4;
const BALL_SPAWN_HEIGHT: f32 = 8.0;
const BALLS_PER_BATCH: usize = 100;
const WALL_SEGMENTS: usize = 32;
const WALL_RADIUS: f32 = 7.0;
const WALL_HEIGHT: f32 = 2.6;
const STEPS: usize = 240;
const WARMUP_STEPS: usize = 120;
const WAVE_STEPS: usize = 30;
const SUB_STEPS: i32 = 4;

fn main() {
    let ball_count = arg_usize(1).unwrap_or(2_000);
    let steps = arg_usize(2).unwrap_or(STEPS);
    let warmup_steps = arg_usize(3).unwrap_or(WARMUP_STEPS);
    let wave_steps = arg_usize(4).unwrap_or(WAVE_STEPS).max(1);
    let contact_recycle_distance =
        arg_f32(5).filter(|distance| distance.is_finite() && *distance >= 0.0);
    let sleep_threshold = arg_f32(6).filter(|threshold| threshold.is_finite() && *threshold >= 0.0);
    let single_worker =
        arg_usize(7).map(|count| count.clamp(1, box3d::MAX_WORKERS as usize) as u32);
    let sub_steps = arg_i32(8)
        .filter(|sub_steps| *sub_steps >= 1)
        .unwrap_or(SUB_STEPS);
    let same_spawn = arg_bool(9).unwrap_or(false);

    println!(
        "balls: {ball_count}, measured steps: {steps}, warmup: {warmup_steps}, wave steps: {wave_steps}, substeps: {sub_steps}, same spawn: {same_spawn}, contact recycle: {}, sleep threshold: {}, workers: {}",
        recycle_label(contact_recycle_distance),
        threshold_label(sleep_threshold),
        single_worker
            .map(|count| count.to_string())
            .unwrap_or_else(|| "1,2,4,8,16,32".to_string()),
    );
    for worker_count in [1_u32, 2, 4, 8, 16, 32] {
        if single_worker.is_some_and(|target| target != worker_count) {
            continue;
        }
        if worker_count > box3d::MAX_WORKERS {
            continue;
        }

        let active = run_active_waves(
            ball_count,
            worker_count,
            wave_steps,
            contact_recycle_distance,
            sleep_threshold,
            sub_steps,
            same_spawn,
        );
        println!(
            "{worker_count} workers active: avg {:.3} ms, max {:.3} ms, native avg {:.3} ms, native max {:.3} ms, bodies {}, contact pairs {}, touched max {}, manifolds max {}, overflow max {}, recycled max {}, sat max {}",
            active.measured_ms,
            active.max_ms,
            active.native_ms,
            active.max_native_ms,
            active.bodies,
            active.contacts,
            active.max_awake_contacts,
            active.max_manifolds,
            active.max_overflow_constraints,
            active.max_recycled_contacts,
            active.max_sat_calls,
        );

        let world = build_settled_world(
            ball_count,
            worker_count,
            contact_recycle_distance,
            sleep_threshold,
            same_spawn,
        );
        for _ in 0..warmup_steps {
            world.step(1.0 / 60.0, sub_steps);
        }

        let started = Instant::now();
        for _ in 0..steps {
            world.step(1.0 / 60.0, sub_steps);
        }
        let total_ms = started.elapsed().as_secs_f64() * 1000.0;
        let profile = world.profile();
        let counters = world.counters();

        println!(
            "{worker_count} workers settled: {:.3} ms/step measured, native profile {:.3} ms, bodies {}, contact pairs {}, touched {}, manifolds {}, overflow {}, recycled {}, sat {}",
            total_ms / steps as f64,
            profile.step,
            counters.body_count,
            counters.contact_count,
            counters.awake_contact_count,
            manifold_count(counters.manifold_counts),
            overflow_constraint_count(counters.color_counts),
            counters.recycled_contact_count,
            counters.sat_call_count,
        );
    }
}

struct Timing {
    measured_ms: f64,
    max_ms: f64,
    native_ms: f64,
    max_native_ms: f32,
    bodies: i32,
    contacts: i32,
    max_awake_contacts: i32,
    max_manifolds: i32,
    max_overflow_constraints: i32,
    max_recycled_contacts: i32,
    max_sat_calls: i32,
}

fn run_active_waves(
    ball_count: usize,
    worker_count: u32,
    wave_steps: usize,
    contact_recycle_distance: Option<f32>,
    sleep_threshold: Option<f32>,
    sub_steps: i32,
    same_spawn: bool,
) -> Timing {
    let world = build_empty_world(ball_count, worker_count, contact_recycle_distance);
    let mut measured_ms = 0.0;
    let mut max_ms = 0.0;
    let mut native_ms = 0.0;
    let mut max_native_ms: f32 = 0.0;
    let mut max_awake_contacts = 0;
    let mut max_manifolds = 0;
    let mut max_overflow_constraints = 0;
    let mut max_recycled_contacts = 0;
    let mut max_sat_calls = 0;
    let mut measured_steps = 0;
    let mut spawned = 0;

    while spawned < ball_count {
        let count = BALLS_PER_BATCH.min(ball_count - spawned);
        create_ball_batch(
            &world,
            if same_spawn { 0 } else { spawned },
            count,
            sleep_threshold,
        );
        spawned += count;

        for _ in 0..wave_steps {
            let started = Instant::now();
            world.step(1.0 / 60.0, sub_steps);
            let step_ms = started.elapsed().as_secs_f64() * 1000.0;
            measured_ms += step_ms;
            max_ms = f64::max(max_ms, step_ms);
            let profile = world.profile();
            native_ms += f64::from(profile.step);
            max_native_ms = f32::max(max_native_ms, profile.step);
            let counters = world.counters();
            max_awake_contacts = max_awake_contacts.max(counters.awake_contact_count);
            max_manifolds = max_manifolds.max(manifold_count(counters.manifold_counts));
            max_overflow_constraints =
                max_overflow_constraints.max(overflow_constraint_count(counters.color_counts));
            max_recycled_contacts = max_recycled_contacts.max(counters.recycled_contact_count);
            max_sat_calls = max_sat_calls.max(counters.sat_call_count);
            measured_steps += 1;
        }
    }

    let counters = world.counters();
    Timing {
        measured_ms: measured_ms / measured_steps as f64,
        max_ms,
        native_ms: native_ms / measured_steps as f64,
        max_native_ms,
        bodies: counters.body_count,
        contacts: counters.contact_count,
        max_awake_contacts,
        max_manifolds,
        max_overflow_constraints,
        max_recycled_contacts,
        max_sat_calls,
    }
}

fn manifold_count(buckets: [i32; 8]) -> i32 {
    buckets
        .into_iter()
        .enumerate()
        .map(|(index, count)| (index as i32 + 1) * count.max(0))
        .sum()
}

fn overflow_constraint_count(color_counts: [i32; 24]) -> i32 {
    color_counts.last().copied().unwrap_or(0).max(0)
}

fn build_settled_world(
    ball_count: usize,
    worker_count: u32,
    contact_recycle_distance: Option<f32>,
    sleep_threshold: Option<f32>,
    same_spawn: bool,
) -> World {
    let world = build_empty_world(ball_count, worker_count, contact_recycle_distance);
    let mut spawned = 0;
    while spawned < ball_count {
        let count = BALLS_PER_BATCH.min(ball_count - spawned);
        create_ball_batch(
            &world,
            if same_spawn { 0 } else { spawned },
            count,
            sleep_threshold,
        );
        spawned += count;
    }
    world
}

fn build_empty_world(
    ball_count: usize,
    worker_count: u32,
    contact_recycle_distance: Option<f32>,
) -> World {
    let world = World::with_capacity_and_workers(
        Vec3::new(0.0, -9.8, 0.0),
        Capacity {
            static_shape_count: (WALL_SEGMENTS + 1) as i32,
            dynamic_shape_count: (ball_count + 2) as i32,
            static_body_count: (WALL_SEGMENTS + 2) as i32,
            dynamic_body_count: (ball_count + 1) as i32,
            contact_count: (ball_count * CONTACTS_PER_BODY) as i32,
        },
        worker_count,
    );
    world.set_continuous_enabled(true);
    world.set_contact_tuning(ContactTuning::default());
    if let Some(distance) = contact_recycle_distance {
        world.set_contact_recycle_distance(distance);
    }

    create_static_scene(&world);
    world
}

fn create_static_scene(world: &World) {
    let ground = world.spawn_body(BodyDef::static_at(Vec3::new(0.0, -0.25, 0.0)));
    let ground_shape = ground.create_box(Vec3::new(8.0, 0.25, 8.0), ShapeDef::default());
    ground_shape.set_surface_material(SurfaceMaterial {
        friction: 1.0,
        restitution: 0.05,
        rolling_resistance: 0.05,
        ..SurfaceMaterial::default()
    });

    let segment_length = std::f32::consts::TAU * WALL_RADIUS / WALL_SEGMENTS as f32 * 0.92;
    for i in 0..WALL_SEGMENTS {
        let angle = i as f32 / WALL_SEGMENTS as f32 * std::f32::consts::TAU;
        let position = Vec3::new(
            angle.cos() * WALL_RADIUS,
            WALL_HEIGHT * 0.5,
            angle.sin() * WALL_RADIUS,
        );
        let wall = world.spawn_body(BodyDef {
            body_type: BodyType::Static,
            position,
            rotation: rotation_y(std::f32::consts::FRAC_PI_2 - angle),
            ..BodyDef::default()
        });
        let shape = wall.create_box(
            Vec3::new(segment_length * 0.5, WALL_HEIGHT * 0.5, 0.25),
            ShapeDef::default(),
        );
        shape.set_surface_material(SurfaceMaterial {
            friction: 0.9,
            restitution: 0.1,
            ..SurfaceMaterial::default()
        });
    }

    let propeller = world.spawn_body(BodyDef {
        body_type: BodyType::Kinematic,
        position: Vec3::new(0.0, 0.55, 0.0),
        angular_velocity: Vec3::new(0.0, 9.0, 0.0),
        ..BodyDef::default()
    });

    for rotation in [0.0, std::f32::consts::FRAC_PI_2] {
        let shape = propeller.create_transformed_box(
            Vec3::new(2.6, 0.125, 0.21),
            Transform::new(Vec3::ZERO, rotation_y(rotation)),
            ShapeDef::default(),
        );
        shape.set_surface_material(SurfaceMaterial {
            friction: 0.7,
            restitution: 0.25,
            ..SurfaceMaterial::default()
        });
    }
}

fn create_ball_batch(
    world: &World,
    start_index: usize,
    ball_count: usize,
    sleep_threshold: Option<f32>,
) {
    let ball_material = SurfaceMaterial {
        friction: 0.75,
        restitution: BALL_RESTITUTION,
        rolling_resistance: 0.03,
        ..SurfaceMaterial::default()
    };

    for i in 0..ball_count {
        let position = ball_spawn_position(start_index + i);

        let body = world.spawn_body(BodyDef {
            body_type: BodyType::Dynamic,
            position,
            linear_damping: 0.02,
            angular_damping: 0.02,
            sleep_threshold,
            ..BodyDef::default()
        });

        let shape = body.create_sphere(
            Vec3::ZERO,
            BALL_RADIUS,
            ShapeDef {
                density: BALL_DENSITY,
                friction: 0.75,
                ..ShapeDef::default()
            },
        );
        shape.set_surface_material(ball_material);
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
    match env::args().nth(index)?.as_str() {
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

fn threshold_label(threshold: Option<f32>) -> String {
    threshold
        .map(|threshold| format!("{threshold:.3}"))
        .unwrap_or_else(|| "native default".to_string())
}

fn rotation_y(angle: f32) -> Quat {
    let half = angle * 0.5;
    Quat::new(Vec3::new(0.0, half.sin(), 0.0), half.cos())
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
