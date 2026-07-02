use crate::math::{Plane, Vec3};
use box3d_sys as sys;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MoverCapsule {
    pub point1: Vec3,
    pub point2: Vec3,
    pub radius: f32,
}

impl MoverCapsule {
    pub fn new(point1: Vec3, point2: Vec3, radius: f32) -> Self {
        assert!(radius > 0.0);
        Self {
            point1,
            point2,
            radius,
        }
    }

    pub fn points(self) -> [Vec3; 2] {
        [self.point1, self.point2]
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CollisionPlane {
    raw: sys::b3CollisionPlane,
}

impl CollisionPlane {
    pub fn new(plane: Plane, push_limit: f32, clip_velocity: bool) -> Self {
        assert!(push_limit >= 0.0 && push_limit.is_finite());
        Self {
            raw: sys::b3CollisionPlane {
                plane: raw_plane(plane),
                pushLimit: push_limit,
                push: 0.0,
                clipVelocity: clip_velocity,
            },
        }
    }

    pub fn rigid(plane: Plane) -> Self {
        Self::new(plane, f32::MAX, true)
    }

    pub fn plane(self) -> Plane {
        self.raw.plane.into()
    }

    pub fn push_limit(self) -> f32 {
        self.raw.pushLimit
    }

    pub fn push(self) -> f32 {
        self.raw.push
    }

    pub fn clip_velocity(self) -> bool {
        self.raw.clipVelocity
    }
}

pub fn solve_planes(target_delta: Vec3, planes: &mut [CollisionPlane]) -> Vec3 {
    assert!(planes.len() <= i32::MAX as usize);
    unsafe {
        sys::b3SolvePlanes(
            target_delta.into(),
            planes.as_mut_ptr().cast::<sys::b3CollisionPlane>(),
            planes.len() as i32,
        )
        .delta
        .into()
    }
}

pub fn clip_vector(vector: Vec3, planes: &[CollisionPlane]) -> Vec3 {
    assert!(planes.len() <= i32::MAX as usize);
    unsafe {
        sys::b3ClipVector(
            vector.into(),
            planes.as_ptr().cast::<sys::b3CollisionPlane>(),
            planes.len() as i32,
        )
        .into()
    }
}

fn raw_plane(plane: Plane) -> sys::b3Plane {
    sys::b3Plane {
        normal: plane.normal.into(),
        offset: plane.offset,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, QueryFilter, ShapeDef, World};

    fn assert_close(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < 1.0e-4, "{actual} != {expected}");
    }

    #[test]
    fn mover_capsule_keeps_points_and_radius() {
        let mover = MoverCapsule::new(Vec3::new(0.0, -0.5, 0.0), Vec3::new(0.0, 0.5, 0.0), 0.25);

        assert_eq!(mover.points(), [mover.point1, mover.point2]);
        assert_close(mover.radius, 0.25);
        assert_close(mover.point1.y, -0.5);
        assert_close(mover.point2.y, 0.5);
    }

    #[test]
    fn solve_planes_updates_planes_and_clip_vector_uses_push() {
        let mut planes = [CollisionPlane::rigid(Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            offset: 0.5,
        })];

        let delta = solve_planes(Vec3::ZERO, &mut planes);
        assert!(delta.z > 0.49 && delta.z < 0.51, "{delta:?}");
        assert!(planes[0].push() > 0.49, "{:?}", planes[0]);

        let clipped = clip_vector(Vec3::new(1.0, 0.0, -2.0), &planes);
        assert_close(clipped.x, 1.0);
        assert_close(clipped.y, 0.0);
        assert_close(clipped.z, 0.0);
    }

    #[test]
    fn mover_capsule_cast_toward_wall_stops_before_full_translation() {
        let world = World::new(Vec3::ZERO);
        let wall = world.create_body(BodyDef::static_at(Vec3::new(2.0, 0.0, 0.0)));
        let _wall_shape = wall.create_box(Vec3::new(0.5, 2.0, 2.0), ShapeDef::default());
        let mover = MoverCapsule::new(Vec3::new(0.0, -0.5, 0.0), Vec3::new(0.0, 0.5, 0.0), 0.25);

        let fraction = world.cast_mover(
            Vec3::ZERO,
            mover.points(),
            mover.radius,
            Vec3::new(4.0, 0.0, 0.0),
            QueryFilter::default(),
        );

        assert!(fraction > 0.0 && fraction < 1.0, "{fraction}");
    }
}
