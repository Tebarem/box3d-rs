use std::{
    any::Any,
    ffi::c_void,
    panic::{catch_unwind, resume_unwind, AssertUnwindSafe},
    sync::{Arc, Mutex},
};

use box3d_sys as sys;

use crate::{events::ShapeId, math::Vec3, world::World};

type CallbackPanic = Box<dyn Any + Send + 'static>;
type CustomFilter = dyn Fn(ShapeId, ShapeId) -> bool + Send + Sync + 'static;
type PreSolve = dyn Fn(PreSolveContact) -> bool + Send + Sync + 'static;

pub type FrictionCallback = extern "C" fn(f32, u64, f32, u64) -> f32;
pub type RestitutionCallback = extern "C" fn(f32, u64, f32, u64) -> f32;

#[derive(Clone, Copy, Debug)]
pub struct PreSolveContact {
    pub shape_a: ShapeId,
    pub shape_b: ShapeId,
    pub point: Vec3,
    pub normal: Vec3,
}

#[derive(Default)]
pub(crate) struct CallbackState {
    custom_filter: Mutex<Option<Box<CustomFilter>>>,
    pre_solve: Mutex<Option<Box<PreSolve>>>,
    panic: Mutex<Option<CallbackPanic>>,
}

impl World {
    pub fn set_friction_callback(&self, callback: FrictionCallback) {
        unsafe { sys::b3World_SetFrictionCallback(self.raw(), Some(callback)) };
    }

    pub fn clear_friction_callback(&self) {
        unsafe { sys::b3World_SetFrictionCallback(self.raw(), None) };
    }

    pub fn set_restitution_callback(&self, callback: RestitutionCallback) {
        unsafe { sys::b3World_SetRestitutionCallback(self.raw(), Some(callback)) };
    }

    pub fn clear_restitution_callback(&self) {
        unsafe { sys::b3World_SetRestitutionCallback(self.raw(), None) };
    }

    pub fn set_custom_filter<F>(&self, callback: F)
    where
        F: Fn(ShapeId, ShapeId) -> bool + Send + Sync + 'static,
    {
        *self
            .callbacks
            .custom_filter
            .lock()
            .expect("custom filter mutex poisoned") = Some(Box::new(callback));

        unsafe {
            sys::b3World_SetCustomFilterCallback(
                self.raw(),
                Some(custom_filter_trampoline),
                self.callback_context(),
            )
        };
    }

    pub fn clear_custom_filter(&self) {
        unsafe { sys::b3World_SetCustomFilterCallback(self.raw(), None, std::ptr::null_mut()) };
        *self
            .callbacks
            .custom_filter
            .lock()
            .expect("custom filter mutex poisoned") = None;
    }

    /// Pre-solve callbacks run during `World::step`; do not mutate this world
    /// or reconfigure callbacks from inside the callback.
    pub fn set_pre_solve<F>(&self, callback: F)
    where
        F: Fn(PreSolveContact) -> bool + Send + Sync + 'static,
    {
        *self
            .callbacks
            .pre_solve
            .lock()
            .expect("pre-solve mutex poisoned") = Some(Box::new(callback));

        unsafe {
            sys::b3World_SetPreSolveCallback(
                self.raw(),
                Some(pre_solve_trampoline),
                self.callback_context(),
            )
        };
    }

    pub fn clear_pre_solve(&self) {
        *self
            .callbacks
            .pre_solve
            .lock()
            .expect("pre-solve mutex poisoned") = None;

        unsafe {
            sys::b3World_SetPreSolveCallback(
                self.raw(),
                Some(pre_solve_trampoline),
                self.callback_context(),
            )
        };
    }

    pub(crate) fn callback_state() -> Arc<CallbackState> {
        Arc::new(CallbackState::default())
    }

    pub(crate) fn callback_context(&self) -> *mut c_void {
        Arc::as_ptr(&self.callbacks).cast_mut().cast()
    }

    pub(crate) fn resume_callback_panic(&self) {
        if let Some(panic) = self
            .callbacks
            .panic
            .lock()
            .expect("callback panic mutex poisoned")
            .take()
        {
            resume_unwind(panic);
        }
    }
}

unsafe extern "C" fn custom_filter_trampoline(
    shape_id_a: sys::b3ShapeId,
    shape_id_b: sys::b3ShapeId,
    context: *mut c_void,
) -> bool {
    let state = unsafe { &*context.cast::<CallbackState>() };
    if state.has_panic() {
        return false;
    }

    let callback = state
        .custom_filter
        .lock()
        .expect("custom filter mutex poisoned");
    let result = catch_unwind(AssertUnwindSafe(|| {
        if let Some(callback) = callback.as_ref() {
            callback(ShapeId::from_raw(shape_id_a), ShapeId::from_raw(shape_id_b))
        } else {
            true
        }
    }));

    match result {
        Ok(should_collide) => should_collide,
        Err(panic) => {
            state.store_panic(panic);
            false
        }
    }
}

unsafe extern "C" fn pre_solve_trampoline(
    shape_id_a: sys::b3ShapeId,
    shape_id_b: sys::b3ShapeId,
    point: sys::b3Pos,
    normal: sys::b3Vec3,
    context: *mut c_void,
) -> bool {
    let state = unsafe { &*context.cast::<CallbackState>() };
    if state.has_panic() {
        return false;
    }

    let callback = state.pre_solve.lock().expect("pre-solve mutex poisoned");
    let contact = PreSolveContact {
        shape_a: ShapeId::from_raw(shape_id_a),
        shape_b: ShapeId::from_raw(shape_id_b),
        point: point.into(),
        normal: normal.into(),
    };
    let result = catch_unwind(AssertUnwindSafe(|| {
        if let Some(callback) = callback.as_ref() {
            callback(contact)
        } else {
            true
        }
    }));

    match result {
        Ok(should_collide) => should_collide,
        Err(panic) => {
            state.store_panic(panic);
            false
        }
    }
}

impl CallbackState {
    fn has_panic(&self) -> bool {
        self.panic
            .lock()
            .expect("callback panic mutex poisoned")
            .is_some()
    }

    fn store_panic(&self, panic: CallbackPanic) {
        let mut slot = self.panic.lock().expect("callback panic mutex poisoned");
        if slot.is_none() {
            *slot = Some(panic);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{
        atomic::{AtomicU64, AtomicUsize, Ordering},
        Arc,
    };

    use super::*;
    use crate::{BodyDef, ShapeDef, SurfaceMaterial};

    static FRICTION_CALLS: AtomicUsize = AtomicUsize::new(0);
    static FRICTION_ID_A: AtomicU64 = AtomicU64::new(0);
    static FRICTION_ID_B: AtomicU64 = AtomicU64::new(0);
    static RESTITUTION_CALLS: AtomicUsize = AtomicUsize::new(0);
    static RESTITUTION_ID_A: AtomicU64 = AtomicU64::new(0);
    static RESTITUTION_ID_B: AtomicU64 = AtomicU64::new(0);

    extern "C" fn test_friction(
        friction_a: f32,
        material_a: u64,
        friction_b: f32,
        material_b: u64,
    ) -> f32 {
        FRICTION_CALLS.fetch_add(1, Ordering::SeqCst);
        FRICTION_ID_A.store(material_a, Ordering::SeqCst);
        FRICTION_ID_B.store(material_b, Ordering::SeqCst);
        friction_a.max(friction_b)
    }

    extern "C" fn test_restitution(
        restitution_a: f32,
        material_a: u64,
        restitution_b: f32,
        material_b: u64,
    ) -> f32 {
        RESTITUTION_CALLS.fetch_add(1, Ordering::SeqCst);
        RESTITUTION_ID_A.store(material_a, Ordering::SeqCst);
        RESTITUTION_ID_B.store(material_b, Ordering::SeqCst);
        restitution_a.max(restitution_b)
    }

    #[test]
    fn custom_filter_can_disable_collision() {
        let world = World::new(Vec3::ZERO);
        let ground = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let _ground_shape = ground.create_box(Vec3::new(1.0, 1.0, 1.0), ShapeDef::default());
        let body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let _shape = body.create_box(
            Vec3::new(1.0, 1.0, 1.0),
            ShapeDef {
                density: 1.0,
                enable_contact_events: true,
                enable_custom_filtering: true,
                ..ShapeDef::default()
            },
        );

        let calls = Arc::new(AtomicUsize::new(0));
        world.set_custom_filter({
            let calls = Arc::clone(&calls);
            move |shape_a, shape_b| {
                assert!(shape_a.is_valid());
                assert!(shape_b.is_valid());
                calls.fetch_add(1, Ordering::Relaxed);
                false
            }
        });

        world.step(1.0 / 60.0, 4);

        assert!(calls.load(Ordering::Relaxed) > 0);
        assert_eq!(world.contact_events().begins().count(), 0);
    }

    #[test]
    fn pre_solve_can_disable_collision() {
        let world = World::default();
        let ground = world.create_body(BodyDef::static_at(Vec3::new(0.0, -0.5, 0.0)));
        let _ground_shape = ground.create_box(Vec3::new(10.0, 0.5, 10.0), ShapeDef::default());
        let body = world.create_body(BodyDef::dynamic_at(Vec3::new(0.0, 4.0, 0.0)));
        let _shape = body.create_sphere(
            Vec3::ZERO,
            0.5,
            ShapeDef {
                density: 1.0,
                enable_pre_solve_events: true,
                ..ShapeDef::default()
            },
        );

        let calls = Arc::new(AtomicUsize::new(0));
        world.set_pre_solve({
            let calls = Arc::clone(&calls);
            move |contact| {
                assert!(contact.shape_a.is_valid());
                assert!(contact.shape_b.is_valid());
                calls.fetch_add(1, Ordering::Relaxed);
                false
            }
        });

        for _ in 0..90 {
            world.step(1.0 / 60.0, 4);
        }

        assert!(calls.load(Ordering::Relaxed) > 0);
        assert!(body.position().y < -0.5, "{:?}", body.position());
    }

    #[test]
    fn material_callbacks_run_during_contact_creation() {
        FRICTION_CALLS.store(0, Ordering::SeqCst);
        FRICTION_ID_A.store(0, Ordering::SeqCst);
        FRICTION_ID_B.store(0, Ordering::SeqCst);
        RESTITUTION_CALLS.store(0, Ordering::SeqCst);
        RESTITUTION_ID_A.store(0, Ordering::SeqCst);
        RESTITUTION_ID_B.store(0, Ordering::SeqCst);

        let world = World::new(Vec3::ZERO);
        world.set_friction_callback(test_friction);
        world.set_restitution_callback(test_restitution);

        let ground = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let ground_shape = ground.create_box(Vec3::new(1.0, 1.0, 1.0), ShapeDef::default());
        ground_shape.set_surface_material(SurfaceMaterial {
            friction: 0.2,
            restitution: 0.1,
            user_material_id: 11,
            ..SurfaceMaterial::default()
        });

        let body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let body_shape = body.create_box(
            Vec3::new(1.0, 1.0, 1.0),
            ShapeDef {
                density: 1.0,
                ..ShapeDef::default()
            },
        );
        body_shape.set_surface_material(SurfaceMaterial {
            friction: 0.8,
            restitution: 0.6,
            user_material_id: 22,
            ..SurfaceMaterial::default()
        });

        world.step(1.0 / 60.0, 4);
        world.clear_friction_callback();
        world.clear_restitution_callback();

        assert!(FRICTION_CALLS.load(Ordering::SeqCst) > 0);
        assert!(RESTITUTION_CALLS.load(Ordering::SeqCst) > 0);
        let friction_ids = [
            FRICTION_ID_A.load(Ordering::SeqCst),
            FRICTION_ID_B.load(Ordering::SeqCst),
        ];
        let restitution_ids = [
            RESTITUTION_ID_A.load(Ordering::SeqCst),
            RESTITUTION_ID_B.load(Ordering::SeqCst),
        ];
        assert!(friction_ids.contains(&11) && friction_ids.contains(&22));
        assert!(restitution_ids.contains(&11) && restitution_ids.contains(&22));
    }
}
