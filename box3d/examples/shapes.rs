use box3d::{BodyDef, BoxShape, Capsule, ShapeDef, Sphere, Transform, Vec3, World};

fn main() {
    let sphere = Sphere::new(Vec3::ZERO, 0.5);
    let capsule = Capsule::new(Vec3::new(0.0, -0.5, 0.0), Vec3::new(0.0, 0.5, 0.0), 0.25);
    let box_shape = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));

    println!("{:?}", sphere.compute_aabb(Transform::IDENTITY));
    println!("{:?}", capsule.compute_mass(1.0));
    println!("{:?}", box_shape.compute_mass(1.0));

    let world = World::new(Vec3::ZERO);
    let body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
    let _sphere = body.create_sphere(Vec3::ZERO, 0.5, ShapeDef::default());
    let _capsule = body.create_capsule(
        Vec3::new(0.0, -0.5, 0.0),
        Vec3::new(0.0, 0.5, 0.0),
        0.25,
        ShapeDef::default(),
    );
    let _box = body.create_box(Vec3::new(0.5, 0.5, 0.5), ShapeDef::default());
}
