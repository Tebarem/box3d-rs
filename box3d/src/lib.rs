#![doc = include_str!("../README.md")]

mod base;
mod body;
mod callbacks;
mod character;
mod collision;
mod compound;
mod contact;
mod debug_draw;
mod dynamic_tree;
mod error;
mod events;
mod handle;
mod hull;
mod joint;
mod math;
mod mesh;
mod query;
mod recording;
mod shape;
mod tasks;
mod world;

pub use base::{
    byte_count, hash, is_double_precision, length_units_per_meter, milliseconds,
    milliseconds_and_reset, set_length_units_per_meter, set_stall_threshold, sleep,
    stall_threshold, ticks, version, yield_now, Version, HASH_INIT,
};
pub use body::{Body, BodyDef, BodyType, MotionLocks};
pub use callbacks::{FrictionCallback, PreSolveContact, RestitutionCallback};
pub use character::{clip_vector, solve_planes, CollisionPlane, MoverCapsule};
pub use collision::{
    collide_capsule_and_sphere, collide_capsule_and_triangle, collide_capsules,
    collide_hull_and_capsule, collide_hull_and_sphere, collide_hull_and_triangle, collide_hulls,
    collide_sphere_and_triangle, collide_spheres, compute_aabb, compute_capsule_aabb,
    compute_capsule_mass, compute_compound_aabb, compute_height_field_aabb, compute_hull_aabb,
    compute_hull_mass, compute_mass, compute_mesh_aabb, compute_sphere_aabb, compute_sphere_mass,
    get_sweep_transform, is_valid_ray, overlap_capsule, overlap_compound, overlap_height_field,
    overlap_hull, overlap_mesh, overlap_sphere, ray_cast_capsule, ray_cast_compound,
    ray_cast_height_field, ray_cast_hollow_sphere, ray_cast_hull, ray_cast_mesh, ray_cast_sphere,
    shape_cast, shape_cast_capsule, shape_cast_compound, shape_cast_height_field, shape_cast_hull,
    shape_cast_mesh, shape_cast_sphere, shape_distance, time_of_impact, BoxShape, Capsule,
    DistanceOutput, FeaturePair, LocalManifold, LocalManifoldPoint, MeshEdgeFlags, RayCastInput,
    ShapeCastInput, ShapeCastOutput, SimpleShape, Sphere, Sweep, TimeOfImpactInput,
    TimeOfImpactOutput, TimeOfImpactState, TriangleFeature,
};
pub use compound::{
    Compound, CompoundCapsule, CompoundChild, CompoundHull, CompoundMesh, CompoundPart,
    CompoundSphere,
};
pub use contact::{ContactData, ContactManifold, ContactPoint};
pub use debug_draw::{
    graph_color, make_debug_color, DebugDraw, DebugDrawOptions, DebugMaterial, DebugShapeHandle,
    DEFAULT_DEBUG_MASK,
};
pub use dynamic_tree::{
    DynamicTree, TreeBoxCastInput, TreeCastHit, TreeClosestHit, TreeHit, TreeProxy,
    TreeRayCastInput,
};
pub use error::{Error, Result};
pub use events::{
    BodyEvents, BodyId, BodyMoveEvent, ContactEvents, ContactHitEvent, ContactId,
    ContactTouchEvent, JointEvent, JointEvents, JointId, SensorEvents, SensorTouchEvent, ShapeId,
    WorldId,
};
pub use hull::{BoxHull, Hull, HullRef};
pub use joint::{
    DistanceJoint, DistanceJointDef, FilterJoint, FilterJointDef, Joint, JointDef, JointType,
    MotorJoint, MotorJointDef, ParallelJoint, ParallelJointDef, PrismaticJoint, PrismaticJointDef,
    RevoluteJoint, RevoluteJointDef, SphericalJoint, SphericalJointDef, WeldJoint, WeldJointDef,
    WheelJoint, WheelJointDef,
};
pub use math::{
    compute_cos_sin, compute_quat_between_unit_vectors, deterministic_atan2, is_bounded_aabb,
    is_sane_aabb, is_valid_aabb, is_valid_float, is_valid_matrix3, is_valid_plane,
    is_valid_position, is_valid_quat, is_valid_transform, is_valid_vec3, is_valid_world_transform,
    line_distance, make_quat_from_matrix, point_to_segment_distance, segment_distance, steiner,
    Aabb, CosSin, Filter, MassData, Matrix3, Plane, Quat, SegmentDistance, SurfaceMaterial,
    Transform, Vec3,
};
pub use mesh::{HeightField, Mesh, MeshCreateOptions, MeshQueryTriangle, MeshTriangle};
pub use query::{
    BodyCastHit, BodyClosestPoint, BodyPlane, CastHit, MoverPlane, QueryFilter, QueryStats, RayHit,
    ShapeProxy, ShapeRef,
};
pub use recording::{
    validate_replay, RecPlayer, RecPlayerInfo, RecQueryHit, RecQueryInfo, RecQueryType, Recording,
};
pub use shape::{Shape, ShapeDef, ShapeType};
pub use tasks::MAX_WORKERS;
pub use world::{
    max_world_count, world_count, Capacity, ContactTuning, Counters, ExplosionDef, Profile, World,
};

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
