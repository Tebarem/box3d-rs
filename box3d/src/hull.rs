use std::ptr::NonNull;

use box3d_sys as sys;

use crate::{
    body::Body,
    collision::{compute_hull_aabb, compute_hull_mass},
    handle,
    math::{Aabb, MassData, Transform, Vec3},
    shape::{raw_shape_def, Shape, ShapeDef},
    Error, Result,
};

pub struct BoxHull {
    raw: sys::b3BoxHull,
}

impl BoxHull {
    pub fn cube(half_width: f32) -> Self {
        assert!(half_width > 0.0);
        Self {
            raw: unsafe { sys::b3MakeCubeHull(half_width) },
        }
    }

    pub fn new(half_extents: Vec3) -> Self {
        assert!(half_extents.x > 0.0 && half_extents.y > 0.0 && half_extents.z > 0.0);
        Self {
            raw: unsafe { sys::b3MakeBoxHull(half_extents.x, half_extents.y, half_extents.z) },
        }
    }

    pub fn offset(half_extents: Vec3, offset: Vec3) -> Self {
        assert!(half_extents.x > 0.0 && half_extents.y > 0.0 && half_extents.z > 0.0);
        Self {
            raw: unsafe {
                sys::b3MakeOffsetBoxHull(
                    half_extents.x,
                    half_extents.y,
                    half_extents.z,
                    offset.into(),
                )
            },
        }
    }

    pub fn transformed(half_extents: Vec3, transform: Transform) -> Self {
        assert!(half_extents.x > 0.0 && half_extents.y > 0.0 && half_extents.z > 0.0);
        Self {
            raw: unsafe {
                sys::b3MakeTransformedBoxHull(
                    half_extents.x,
                    half_extents.y,
                    half_extents.z,
                    transform.into(),
                )
            },
        }
    }

    pub fn scaled(half_extents: Vec3, transform: Transform, post_scale: Vec3) -> Self {
        assert!(half_extents.x > 0.0 && half_extents.y > 0.0 && half_extents.z > 0.0);
        assert_valid_transform(transform);
        assert_valid_vec3(post_scale);
        Self {
            raw: unsafe {
                sys::b3MakeScaledBoxHull(half_extents.into(), transform.into(), post_scale.into())
            },
        }
    }

    pub fn scale_box(
        half_extents: Vec3,
        transform: Transform,
        post_scale: Vec3,
        min_half_extent: f32,
    ) -> (Vec3, Transform) {
        assert!(half_extents.x > 0.0 && half_extents.y > 0.0 && half_extents.z > 0.0);
        assert_valid_transform(transform);
        assert_valid_vec3(post_scale);
        assert!(min_half_extent > 0.0 && min_half_extent.is_finite());
        let mut raw_half_extents = half_extents.into();
        let mut raw_transform = transform.into();
        unsafe {
            sys::b3ScaleBox(
                &mut raw_half_extents,
                &mut raw_transform,
                post_scale.into(),
                min_half_extent,
            )
        };
        (raw_half_extents.into(), raw_transform.into())
    }

    pub fn compute_aabb(&self, transform: Transform) -> Aabb {
        compute_hull_aabb(self, transform)
    }

    pub fn compute_mass(&self, density: f32) -> MassData {
        compute_hull_mass(self, density)
    }

    pub fn vertex_count(&self) -> i32 {
        self.raw.base.vertexCount
    }

    pub fn face_count(&self) -> i32 {
        self.raw.base.faceCount
    }
}

pub struct Hull {
    raw: NonNull<sys::b3HullData>,
}

impl Hull {
    pub fn cylinder(height: f32, radius: f32, y_offset: f32, sides: i32) -> Result<Self> {
        if height <= 0.0 || radius <= 0.0 || !y_offset.is_finite() || !(3..=32).contains(&sides) {
            return Err(Error::InvalidInput);
        }
        Self::from_raw(unsafe { sys::b3CreateCylinder(height, radius, y_offset, sides) })
    }

    pub fn cone(height: f32, radius1: f32, radius2: f32, slices: i32) -> Result<Self> {
        if height <= 0.0 || radius1 <= 0.0 || radius2 <= 0.0 || !(4..=32).contains(&slices) {
            return Err(Error::InvalidInput);
        }
        Self::from_raw(unsafe { sys::b3CreateCone(height, radius1, radius2, slices) })
    }

    pub fn rock(radius: f32) -> Result<Self> {
        if radius <= 0.0 {
            return Err(Error::InvalidInput);
        }
        Self::from_raw(unsafe { sys::b3CreateRock(radius) })
    }

    pub fn from_points(points: &[Vec3], max_vertices: i32) -> Result<Self> {
        let point_count = i32::try_from(points.len()).map_err(|_| Error::Null)?;
        let raw_points = points.iter().copied().map(Into::into).collect::<Vec<_>>();
        let raw = unsafe { sys::b3CreateHull(raw_points.as_ptr(), point_count, max_vertices) };
        Self::from_raw(raw)
    }

    pub fn clone_transformed(&self, transform: Transform, scale: Vec3) -> Result<Self> {
        assert_valid_transform(transform);
        assert_valid_vec3(scale);
        Self::from_raw(unsafe {
            sys::b3CloneAndTransformHull(self.raw.as_ptr(), transform.into(), scale.into())
        })
    }

    pub fn compute_aabb(&self, transform: Transform) -> Aabb {
        compute_hull_aabb(self, transform)
    }

    pub fn compute_mass(&self, density: f32) -> MassData {
        compute_hull_mass(self, density)
    }

    pub fn vertex_count(&self) -> i32 {
        unsafe { self.raw.as_ref().vertexCount }
    }

    pub fn face_count(&self) -> i32 {
        unsafe { self.raw.as_ref().faceCount }
    }

    fn from_raw(raw: *mut sys::b3HullData) -> Result<Self> {
        let raw = NonNull::new(raw).ok_or(Error::Null)?;
        Ok(Self { raw })
    }
}

impl Clone for Hull {
    fn clone(&self) -> Self {
        Self::from_raw(unsafe { sys::b3CloneHull(self.raw.as_ptr()) })
            .expect("box3d returned a null hull")
    }
}

impl Drop for Hull {
    fn drop(&mut self) {
        unsafe { sys::b3DestroyHull(self.raw.as_ptr()) };
    }
}

#[derive(Clone, Copy)]
pub enum HullRef<'a> {
    Box(&'a BoxHull),
    Custom(&'a Hull),
}

impl<'a> From<&'a BoxHull> for HullRef<'a> {
    fn from(value: &'a BoxHull) -> Self {
        Self::Box(value)
    }
}

impl<'a> From<&'a Hull> for HullRef<'a> {
    fn from(value: &'a Hull) -> Self {
        Self::Custom(value)
    }
}

impl HullRef<'_> {
    pub(crate) fn raw(self) -> *const sys::b3HullData {
        match self {
            Self::Box(hull) => &hull.raw.base,
            Self::Custom(hull) => hull.raw.as_ptr(),
        }
    }
}

impl Body<'_> {
    pub fn create_hull<'a>(&self, hull: impl Into<HullRef<'a>>, def: ShapeDef) -> Shape<'_> {
        let raw_def = raw_shape_def(def);
        let raw = handle::shape(unsafe {
            sys::b3CreateHullShape(self.raw(), &raw_def, hull.into().raw())
        })
        .expect("box3d returned an invalid shape");

        Shape::from_raw(raw)
    }

    pub fn create_transformed_hull<'a>(
        &self,
        hull: impl Into<HullRef<'a>>,
        transform: Transform,
        scale: Vec3,
        def: ShapeDef,
    ) -> Shape<'_> {
        assert_valid_transform(transform);
        assert_valid_vec3(scale);
        let raw_def = raw_shape_def(def);
        let raw = handle::shape(unsafe {
            sys::b3CreateTransformedHullShape(
                self.raw(),
                &raw_def,
                hull.into().raw(),
                transform.into(),
                scale.into(),
            )
        })
        .expect("box3d returned an invalid shape");

        Shape::from_raw(raw)
    }
}

fn assert_valid_transform(transform: Transform) {
    assert_valid_vec3(transform.p);
    assert_valid_vec3(transform.q.v);
    assert!(transform.q.s.is_finite());
}

fn assert_valid_vec3(value: Vec3) {
    assert!(value.x.is_finite() && value.y.is_finite() && value.z.is_finite());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, Quat, ShapeType, World};

    #[test]
    fn custom_cube_hull_steps() {
        let world = World::default();
        let ground = world.create_body(BodyDef::static_at(Vec3::new(0.0, -10.0, 0.0)));
        let ground_hull = BoxHull::new(Vec3::new(50.0, 10.0, 50.0));
        let _ground_shape = ground.create_hull(&ground_hull, ShapeDef::default());

        let points = [
            Vec3::new(-0.5, -0.5, -0.5),
            Vec3::new(0.5, -0.5, -0.5),
            Vec3::new(0.5, 0.5, -0.5),
            Vec3::new(-0.5, 0.5, -0.5),
            Vec3::new(-0.5, -0.5, 0.5),
            Vec3::new(0.5, -0.5, 0.5),
            Vec3::new(0.5, 0.5, 0.5),
            Vec3::new(-0.5, 0.5, 0.5),
        ];
        let hull = Hull::from_points(&points, 8).unwrap();
        let body = world.create_body(BodyDef::dynamic_at(Vec3::new(0.0, 4.0, 0.0)));
        let shape = body.create_hull(
            &hull,
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                ..ShapeDef::default()
            },
        );

        world.step(1.0 / 60.0, 4);

        assert!(shape.is_valid());
        assert!(body.position().y.is_finite());
    }

    #[test]
    fn transformed_hull_creation_bakes_geometry() {
        let world = World::new(Vec3::ZERO);
        let hull = BoxHull::new(Vec3::new(0.5, 0.25, 0.5));
        let body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let shape = body.create_transformed_hull(
            &hull,
            Transform::new(Vec3::new(0.1, 0.0, 0.0), Quat::IDENTITY),
            Vec3::new(-1.0, 1.0, 1.0),
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                ..ShapeDef::default()
            },
        );

        assert!(shape.is_valid());
        assert_eq!(shape.shape_type(), ShapeType::Hull);
    }

    #[test]
    fn hull_generators_clone_and_box_helpers() {
        let cube = BoxHull::cube(0.5);
        assert_eq!(cube.vertex_count(), 8);
        assert_eq!(cube.face_count(), 6);
        assert!(cube.compute_mass(1.0).mass > 0.0);

        let scaled = BoxHull::scaled(
            Vec3::new(0.5, 0.5, 0.5),
            Transform::IDENTITY,
            Vec3::new(-2.0, 1.0, 1.0),
        );
        assert_eq!(scaled.face_count(), 6);

        let (half_extents, transform) = BoxHull::scale_box(
            Vec3::new(0.5, 0.25, 0.75),
            Transform::IDENTITY,
            Vec3::new(-2.0, 1.0, 1.0),
            0.01,
        );
        assert!(half_extents.x >= 0.01);
        assert!(transform.q.s.is_finite());

        let cylinder = Hull::cylinder(1.0, 0.5, 0.0, 8).unwrap();
        let cone = Hull::cone(1.0, 0.5, 0.25, 8).unwrap();
        let rock = Hull::rock(0.5).unwrap();
        let cloned = cylinder.clone();
        let transformed = cloned
            .clone_transformed(Transform::IDENTITY, Vec3::new(1.0, 1.0, 1.0))
            .unwrap();

        assert!(cylinder.vertex_count() >= 8);
        assert!(cone.vertex_count() >= 8);
        assert!(rock.vertex_count() > 0);
        assert_eq!(transformed.face_count(), cloned.face_count());
        assert!(transformed
            .compute_aabb(Transform::IDENTITY)
            .upper_bound
            .y
            .is_finite());
    }
}
