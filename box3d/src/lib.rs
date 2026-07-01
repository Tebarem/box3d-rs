use std::{cell::Cell, marker::PhantomData};

use box3d_sys as sys;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

impl From<Vec3> for sys::b3Vec3 {
    fn from(value: Vec3) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<sys::b3Pos> for Vec3 {
    fn from(value: sys::b3Pos) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

pub struct World {
    raw: sys::b3WorldId,
    _not_sync: PhantomData<Cell<()>>,
}

impl World {
    pub fn new(gravity: Vec3) -> Self {
        let mut def = unsafe { sys::b3DefaultWorldDef() };
        def.gravity = gravity.into();

        let raw = unsafe { sys::b3CreateWorld(&def) };
        assert!(unsafe { sys::b3World_IsValid(raw) });

        Self {
            raw,
            _not_sync: PhantomData,
        }
    }

    pub fn create_body(&self, def: BodyDef) -> Body<'_> {
        let mut raw_def = unsafe { sys::b3DefaultBodyDef() };
        raw_def.type_ = match def.body_type {
            BodyType::Static => sys::b3BodyType_b3_staticBody,
            BodyType::Kinematic => sys::b3BodyType_b3_kinematicBody,
            BodyType::Dynamic => sys::b3BodyType_b3_dynamicBody,
        };
        raw_def.position = def.position.into();

        let raw = unsafe { sys::b3CreateBody(self.raw, &raw_def) };
        assert!(unsafe { sys::b3Body_IsValid(raw) });

        Body {
            raw,
            _world: PhantomData,
        }
    }

    pub fn step(&self, time_step: f32, sub_step_count: i32) {
        unsafe { sys::b3World_Step(self.raw, time_step, sub_step_count) };
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new(Vec3::new(0.0, -10.0, 0.0))
    }
}

impl Drop for World {
    fn drop(&mut self) {
        unsafe { sys::b3DestroyWorld(self.raw) };
    }
}

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

impl Body<'_> {
    pub fn position(&self) -> Vec3 {
        unsafe { sys::b3Body_GetPosition(self.raw) }.into()
    }

    pub fn create_box(&self, half_extents: Vec3, def: ShapeDef) -> Shape<'_> {
        let mut raw_def = unsafe { sys::b3DefaultShapeDef() };
        raw_def.density = def.density;
        raw_def.baseMaterial.friction = def.friction;

        let hull = unsafe { sys::b3MakeBoxHull(half_extents.x, half_extents.y, half_extents.z) };
        let raw = unsafe { sys::b3CreateHullShape(self.raw, &raw_def, &hull.base) };
        assert!(unsafe { sys::b3Shape_IsValid(raw) });

        Shape {
            raw,
            _body: PhantomData,
        }
    }
}

impl Drop for Body<'_> {
    fn drop(&mut self) {
        if unsafe { sys::b3Body_IsValid(self.raw) } {
            unsafe { sys::b3DestroyBody(self.raw) };
        }
    }
}

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

impl Shape<'_> {
    pub fn is_valid(&self) -> bool {
        unsafe { sys::b3Shape_IsValid(self.raw) }
    }
}

impl Drop for Shape<'_> {
    fn drop(&mut self) {
        if self.is_valid() {
            unsafe { sys::b3DestroyShape(self.raw, true) };
        }
    }
}

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
