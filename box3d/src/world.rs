use std::{cell::Cell, marker::PhantomData};

use box3d_sys as sys;

use crate::{
    body::{Body, BodyDef, BodyType},
    handle,
    math::Vec3,
    Result,
};

pub struct World {
    raw: sys::b3WorldId,
    _not_sync: PhantomData<Cell<()>>,
}

impl World {
    pub fn new(gravity: Vec3) -> Self {
        Self::try_new(gravity).expect("box3d returned an invalid world")
    }

    pub fn try_new(gravity: Vec3) -> Result<Self> {
        let mut def = unsafe { sys::b3DefaultWorldDef() };
        def.gravity = gravity.into();

        let raw = handle::world(unsafe { sys::b3CreateWorld(&def) })?;

        Ok(Self {
            raw,
            _not_sync: PhantomData,
        })
    }

    pub fn create_body(&self, def: BodyDef) -> Body<'_> {
        self.try_create_body(def)
            .expect("box3d returned an invalid body")
    }

    pub fn try_create_body(&self, def: BodyDef) -> Result<Body<'_>> {
        let mut raw_def = unsafe { sys::b3DefaultBodyDef() };
        raw_def.type_ = match def.body_type {
            BodyType::Static => sys::b3BodyType_b3_staticBody,
            BodyType::Kinematic => sys::b3BodyType_b3_kinematicBody,
            BodyType::Dynamic => sys::b3BodyType_b3_dynamicBody,
        };
        raw_def.position = def.position.into();

        let raw = handle::body(unsafe { sys::b3CreateBody(self.raw, &raw_def) })?;

        Ok(Body::from_raw(raw))
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
