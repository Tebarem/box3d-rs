use std::{env, time::Instant};

use box3d::{BodyDef, Capacity, ShapeDef, Vec3, World};

const BASE_COUNT: usize = 90;
const STEPS: usize = 200;
const SUB_STEPS: i32 = 4;

fn main() {
    let worker_count = arg_u32(1).unwrap_or(8).clamp(1, box3d::MAX_WORKERS);
    let repeats = arg_usize(2).unwrap_or(3).max(1);

    println!(
        "benchmark: large_pyramid_rust, steps: {STEPS}, substeps: {SUB_STEPS}, workers: {worker_count}, repeats: {repeats}"
    );

    for run in 0..repeats {
        let world = create_large_pyramid(worker_count);
        world.step(1.0 / 60.0, SUB_STEPS);

        let started = Instant::now();
        for _ in 0..STEPS {
            world.step(1.0 / 60.0, SUB_STEPS);
        }
        let elapsed_ms = started.elapsed().as_secs_f64() * 1000.0;
        let counters = world.counters();
        println!(
            "run {run}: {elapsed_ms:.3} ms total, {:.3} ms/step, bodies {}, shapes {}, contacts {}, stack {}",
            elapsed_ms / STEPS as f64,
            counters.body_count,
            counters.shape_count,
            counters.contact_count,
            counters.stack_used,
        );
    }
}

fn create_large_pyramid(worker_count: u32) -> World {
    let dynamic_count = BASE_COUNT * (BASE_COUNT + 1) / 2;
    let world = World::with_capacity_and_workers(
        Vec3::new(0.0, -9.8, 0.0),
        Capacity {
            static_shape_count: 1,
            dynamic_shape_count: dynamic_count as i32,
            static_body_count: 1,
            dynamic_body_count: dynamic_count as i32,
            contact_count: (dynamic_count * 4) as i32,
        },
        worker_count,
    );
    world.set_sleeping_enabled(false);

    let ground = world.spawn_body(BodyDef::static_at(Vec3::new(0.0, -1.0, 0.0)));
    ground.create_box(Vec3::new(400.0, 1.0, 400.0), ShapeDef::default());

    let shape = ShapeDef {
        density: 100.0,
        ..ShapeDef::default()
    };
    let h = 0.5;
    let shift = h;

    for i in 0..BASE_COUNT {
        let y = (2.0 * i as f32 + 1.0) * shift;
        for j in i..BASE_COUNT {
            let x = (i as f32 + 1.0) * shift + 2.0 * (j - i) as f32 * shift - h * BASE_COUNT as f32;
            let body = world.spawn_body(BodyDef::dynamic_at(Vec3::new(x, y, 0.0)));
            body.create_box(Vec3::new(h, h, h), shape);
        }
    }

    world
}

fn arg_u32(index: usize) -> Option<u32> {
    env::args().nth(index)?.parse().ok()
}

fn arg_usize(index: usize) -> Option<usize> {
    env::args().nth(index)?.parse().ok()
}
