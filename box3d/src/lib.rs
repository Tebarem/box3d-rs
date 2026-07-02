mod body;
mod character;
mod collision;
mod compound;
mod debug_draw;
mod error;
mod events;
mod handle;
mod hull;
mod joint;
mod math;
mod mesh;
mod query;
mod shape;
mod world;

pub use body::{Body, BodyDef, BodyType};
pub use character::{clip_vector, solve_planes, CollisionPlane, MoverCapsule};
pub use collision::{
    compute_aabb, compute_mass, shape_cast, shape_distance, BoxShape, Capsule, DistanceOutput,
    ShapeCastOutput, SimpleShape, Sphere,
};
pub use compound::{Compound, CompoundPart};
pub use debug_draw::{DebugDraw, DEFAULT_DEBUG_MASK};
pub use error::{Error, Result};
pub use events::{
    BodyEvents, BodyId, BodyMoveEvent, ContactEvents, ContactHitEvent, ContactId,
    ContactTouchEvent, JointEvent, JointEvents, JointId, SensorEvents, SensorTouchEvent, ShapeId,
};
pub use hull::{BoxHull, Hull, HullRef};
pub use joint::{
    DistanceJoint, DistanceJointDef, FilterJoint, FilterJointDef, Joint, JointDef, JointType,
    MotorJoint, MotorJointDef, ParallelJoint, ParallelJointDef, PrismaticJoint, PrismaticJointDef,
    RevoluteJoint, RevoluteJointDef, SphericalJoint, SphericalJointDef, WeldJoint, WeldJointDef,
    WheelJoint, WheelJointDef,
};
pub use math::{Aabb, Filter, MassData, Matrix3, Quat, SurfaceMaterial, Transform, Vec3};
pub use mesh::{HeightField, Mesh};
pub use query::{
    CastHit, MoverPlane, Plane, QueryFilter, QueryStats, RayHit, ShapeProxy, ShapeRef,
};
pub use shape::{Shape, ShapeDef, ShapeType};
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
                ..ShapeDef::default()
            },
        );

        for _ in 0..90 {
            world.step(1.0 / 60.0, 4);
        }

        let position = body.position();
        assert!((position.y - 0.5).abs() < 0.05, "{position:?}");
    }
}
