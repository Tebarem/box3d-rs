use std::marker::PhantomData;

use box3d_sys as sys;

use crate::{body::Body, handle, math::Vec3};

#[derive(Clone, Copy, Debug)]
pub struct ShapeDef {
    pub density: f32,
    pub friction: f32,
}

impl Default for ShapeDef {
    fn default() -> Self {
        Self {
            density: 0.0,
            friction: 0.6,
        }
    }
}

pub struct Shape<'body> {
    raw: sys::b3ShapeId,
    _body: PhantomData<&'body ()>,
}

impl<'body> Shape<'body> {
    pub(crate) fn from_raw(raw: sys::b3ShapeId) -> Self {
        Self {
            raw,
            _body: PhantomData,
        }
    }

    pub fn is_valid(&self) -> bool {
        handle::is_shape_valid(self.raw)
    }
}

impl Drop for Shape<'_> {
    fn drop(&mut self) {
        handle::destroy_shape(self.raw, true);
    }
}

impl Body<'_> {
    pub fn create_sphere(&self, center: Vec3, radius: f32, def: ShapeDef) -> Shape<'_> {
        assert!(radius > 0.0);
        let raw_def = raw_shape_def(def);
        let sphere = sys::b3Sphere {
            center: center.into(),
            radius,
        };
        let raw = handle::shape(unsafe { sys::b3CreateSphereShape(self.raw(), &raw_def, &sphere) })
            .expect("box3d returned an invalid shape");

        Shape::from_raw(raw)
    }

    pub fn create_capsule(
        &self,
        point1: Vec3,
        point2: Vec3,
        radius: f32,
        def: ShapeDef,
    ) -> Shape<'_> {
        assert!(radius > 0.0);
        let raw_def = raw_shape_def(def);
        let capsule = sys::b3Capsule {
            center1: point1.into(),
            center2: point2.into(),
            radius,
        };
        let raw =
            handle::shape(unsafe { sys::b3CreateCapsuleShape(self.raw(), &raw_def, &capsule) })
                .expect("box3d returned an invalid shape");

        Shape::from_raw(raw)
    }
}

fn raw_shape_def(def: ShapeDef) -> sys::b3ShapeDef {
    let mut raw_def = unsafe { sys::b3DefaultShapeDef() };
    raw_def.density = def.density;
    raw_def.baseMaterial.friction = def.friction;
    raw_def
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, World};

    #[test]
    fn sphere_and_capsule_shapes_step() {
        let world = World::default();
        let ground = world.create_body(BodyDef::static_at(Vec3::new(0.0, -10.0, 0.0)));
        let _ground_shape = ground.create_box(Vec3::new(50.0, 10.0, 50.0), ShapeDef::default());

        let sphere = world.create_body(BodyDef::dynamic_at(Vec3::new(-1.0, 4.0, 0.0)));
        let sphere_shape = sphere.create_sphere(
            Vec3::ZERO,
            0.5,
            ShapeDef {
                density: 1.0,
                friction: 0.3,
            },
        );

        let capsule = world.create_body(BodyDef::dynamic_at(Vec3::new(1.0, 4.0, 0.0)));
        let capsule_shape = capsule.create_capsule(
            Vec3::new(0.0, -0.5, 0.0),
            Vec3::new(0.0, 0.5, 0.0),
            0.25,
            ShapeDef {
                density: 1.0,
                friction: 0.3,
            },
        );

        world.step(1.0 / 60.0, 4);

        assert!(sphere_shape.is_valid());
        assert!(capsule_shape.is_valid());
        assert!(sphere.position().y.is_finite());
        assert!(capsule.position().y.is_finite());
    }
}
