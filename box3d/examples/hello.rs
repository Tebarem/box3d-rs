use box3d::{BodyDef, ShapeDef, Vec3, World};

fn main() {
    let world = World::default();

    let ground = world.create_body(BodyDef::static_at(Vec3::new(0.0, -10.0, 0.0)));
    let _ground_shape = ground.create_box(Vec3::new(50.0, 10.0, 50.0), ShapeDef::default());

    let body = world.create_body(BodyDef::dynamic_at(Vec3::new(0.0, 4.0, 0.0)));
    let _shape = body.create_box(
        Vec3::new(0.5, 0.5, 0.5),
        ShapeDef {
            density: 1.0,
            friction: 0.3,
        },
    );

    for _ in 0..90 {
        world.step(1.0 / 60.0, 4);
    }

    println!("{:?}", body.position());
}
