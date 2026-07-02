use std::marker::PhantomData;

use box3d_sys as sys;

use crate::handle;

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
