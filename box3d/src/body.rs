use std::marker::PhantomData;

use box3d_sys as sys;

use crate::{
    handle,
    math::Vec3,
    shape::{Shape, ShapeDef},
    world::World,
    Result,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BodyType {
    Static,
    Kinematic,
    Dynamic,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyDef {
    pub body_type: BodyType,
    pub position: Vec3,
}

impl BodyDef {
    pub const fn static_at(position: Vec3) -> Self {
        Self {
            body_type: BodyType::Static,
            position,
        }
    }

    pub const fn dynamic_at(position: Vec3) -> Self {
        Self {
            body_type: BodyType::Dynamic,
            position,
        }
    }
}

impl Default for BodyDef {
    fn default() -> Self {
        Self::static_at(Vec3::ZERO)
    }
}

pub struct Body<'world> {
    raw: sys::b3BodyId,
    _world: PhantomData<&'world World>,
}

impl<'world> Body<'world> {
    pub(crate) fn from_raw(raw: sys::b3BodyId) -> Self {
        Self {
            raw,
            _world: PhantomData,
        }
    }

    pub fn position(&self) -> Vec3 {
        unsafe { sys::b3Body_GetPosition(self.raw) }.into()
    }

    pub fn create_box(&self, half_extents: Vec3, def: ShapeDef) -> Shape<'_> {
        self.try_create_box(half_extents, def)
            .expect("box3d returned an invalid shape")
    }

    pub fn try_create_box(&self, half_extents: Vec3, def: ShapeDef) -> Result<Shape<'_>> {
        let mut raw_def = unsafe { sys::b3DefaultShapeDef() };
        raw_def.density = def.density;
        raw_def.baseMaterial.friction = def.friction;

        let hull = unsafe { sys::b3MakeBoxHull(half_extents.x, half_extents.y, half_extents.z) };
        let raw = handle::shape(unsafe { sys::b3CreateHullShape(self.raw, &raw_def, &hull.base) })?;

        Ok(Shape::from_raw(raw))
    }
}

impl Drop for Body<'_> {
    fn drop(&mut self) {
        handle::destroy_body(self.raw);
    }
}
