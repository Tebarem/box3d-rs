use box3d::{Aabb, BodyDef, QueryFilter, ShapeDef, ShapeProxy, Vec3, World};

fn main() {
    let world = World::new(Vec3::ZERO);
    let wall = world.create_body(BodyDef::static_at(Vec3::new(2.0, 0.0, 0.0)));
    let _wall_shape = wall.create_box(Vec3::new(0.5, 2.0, 2.0), ShapeDef::default());

    let ray = world.cast_ray_closest(Vec3::ZERO, Vec3::new(4.0, 0.0, 0.0), QueryFilter::default());
    println!("{ray:?}");

    let mut overlaps = 0;
    world.overlap_aabb(
        Aabb {
            lower_bound: Vec3::new(1.0, -3.0, -3.0),
            upper_bound: Vec3::new(3.0, 3.0, 3.0),
        },
        QueryFilter::default(),
        |shape| {
            overlaps += usize::from(shape.is_valid());
            true
        },
    );

    let points = [Vec3::ZERO];
    let proxy = ShapeProxy::new(&points, 0.25).unwrap();
    let mut casts = 0;
    world.cast_shape(
        Vec3::ZERO,
        proxy,
        Vec3::new(4.0, 0.0, 0.0),
        QueryFilter::default(),
        |hit| {
            casts += usize::from(hit.shape.is_valid());
            hit.fraction
        },
    );

    println!("overlaps={overlaps} casts={casts}");
}
