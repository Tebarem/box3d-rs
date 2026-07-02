use std::sync::Mutex;

use box3d_sys as sys;

use crate::{Error, Result};

static WORLD_ALLOC: Mutex<()> = Mutex::new(());

pub(crate) fn create_world(def: &sys::b3WorldDef) -> Result<sys::b3WorldId> {
    let _guard = WORLD_ALLOC.lock().expect("box3d world mutex poisoned");
    world(unsafe { sys::b3CreateWorld(def) })
}

pub(crate) fn world(raw: sys::b3WorldId) -> Result<sys::b3WorldId> {
    if is_world_valid(raw) {
        Ok(raw)
    } else {
        Err(Error::InvalidWorld)
    }
}

pub(crate) fn body(raw: sys::b3BodyId) -> Result<sys::b3BodyId> {
    if is_body_valid(raw) {
        Ok(raw)
    } else {
        Err(Error::InvalidBody)
    }
}

pub(crate) fn shape(raw: sys::b3ShapeId) -> Result<sys::b3ShapeId> {
    if is_shape_valid(raw) {
        Ok(raw)
    } else {
        Err(Error::InvalidShape)
    }
}

pub(crate) fn is_world_valid(raw: sys::b3WorldId) -> bool {
    unsafe { sys::b3World_IsValid(raw) }
}

pub(crate) fn is_body_valid(raw: sys::b3BodyId) -> bool {
    unsafe { sys::b3Body_IsValid(raw) }
}

pub(crate) fn is_shape_valid(raw: sys::b3ShapeId) -> bool {
    unsafe { sys::b3Shape_IsValid(raw) }
}

#[allow(dead_code)]
pub(crate) fn is_joint_valid(raw: sys::b3JointId) -> bool {
    unsafe { sys::b3Joint_IsValid(raw) }
}

#[allow(dead_code)]
pub(crate) fn is_contact_valid(raw: sys::b3ContactId) -> bool {
    unsafe { sys::b3Contact_IsValid(raw) }
}

pub(crate) fn destroy_body(raw: sys::b3BodyId) {
    if is_body_valid(raw) {
        unsafe { sys::b3DestroyBody(raw) };
    }
}

pub(crate) fn destroy_world(raw: sys::b3WorldId) {
    let _guard = WORLD_ALLOC.lock().expect("box3d world mutex poisoned");
    if is_world_valid(raw) {
        unsafe { sys::b3DestroyWorld(raw) };
    }
}

pub(crate) fn destroy_shape(raw: sys::b3ShapeId, update_body_mass: bool) {
    if is_shape_valid(raw) {
        unsafe { sys::b3DestroyShape(raw, update_body_mass) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn null_world_is_invalid() {
        let raw = sys::b3WorldId {
            index1: 0,
            generation: 0,
        };
        assert!(matches!(world(raw), Err(Error::InvalidWorld)));
    }
}
