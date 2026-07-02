mod body;
mod error;
mod handle;
mod hull;
mod math;
mod mesh;
mod query;
mod shape;
mod world;

pub use body::{Body, BodyDef, BodyType};
pub use error::{Error, Result};
pub use hull::{BoxHull, Hull, HullRef};
pub use math::{Aabb, Filter, MassData, Matrix3, Quat, SurfaceMaterial, Transform, Vec3};
pub use mesh::{HeightField, Mesh};
pub use query::{QueryFilter, RayHit};
pub use shape::{Shape, ShapeDef};
pub use world::{Counters, Profile, World};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dynamic_box_falls_onto_ground() {
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

        let position = body.position();
        assert!((position.y - 0.5).abs() < 0.05, "{position:?}");
    }
}
