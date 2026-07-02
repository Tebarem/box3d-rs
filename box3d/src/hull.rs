use std::ptr::NonNull;

use box3d_sys as sys;

use crate::{
    body::Body,
    handle,
    math::{Transform, Vec3},
    shape::{raw_shape_def, Shape, ShapeDef},
    Error, Result,
};

pub struct BoxHull {
    raw: sys::b3BoxHull,
}

impl BoxHull {
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
}

pub struct Hull {
    raw: NonNull<sys::b3HullData>,
}

impl Hull {
    pub fn from_points(points: &[Vec3], max_vertices: i32) -> Result<Self> {
        let point_count = i32::try_from(points.len()).map_err(|_| Error::Null)?;
        let raw_points = points.iter().copied().map(Into::into).collect::<Vec<_>>();
        let raw = unsafe { sys::b3CreateHull(raw_points.as_ptr(), point_count, max_vertices) };
        let raw = NonNull::new(raw).ok_or(Error::Null)?;

        Ok(Self { raw })
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
}
