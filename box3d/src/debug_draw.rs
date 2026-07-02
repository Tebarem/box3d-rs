use std::{
    any::Any,
    ffi::{c_char, c_void, CStr},
    panic::{catch_unwind, resume_unwind, AssertUnwindSafe},
};

use box3d_sys as sys;

use crate::{
    math::{Aabb, Transform, Vec3},
    world::World,
};

pub const DEFAULT_DEBUG_MASK: u64 = u64::MAX;

pub trait DebugDraw {
    fn draw_shape(&mut self, _transform: Transform, _color: u32) -> bool {
        true
    }

    fn draw_segment(&mut self, _p1: Vec3, _p2: Vec3, _color: u32) {}

    fn draw_transform(&mut self, _transform: Transform) {}

    fn draw_point(&mut self, _point: Vec3, _size: f32, _color: u32) {}

    fn draw_sphere(&mut self, _center: Vec3, _radius: f32, _color: u32, _alpha: f32) {}

    fn draw_capsule(
        &mut self,
        _point1: Vec3,
        _point2: Vec3,
        _radius: f32,
        _color: u32,
        _alpha: f32,
    ) {
    }

    fn draw_bounds(&mut self, _bounds: Aabb, _color: u32) {}

    fn draw_box(&mut self, _extents: Vec3, _transform: Transform, _color: u32) {}

    fn draw_string(&mut self, _point: Vec3, _text: &str, _color: u32) {}
}

type CallbackPanic = Box<dyn Any + Send + 'static>;

struct DebugDrawContext<'a, D> {
    draw: &'a mut D,
    panic: Option<CallbackPanic>,
}

impl World {
    pub fn draw<D: DebugDraw>(&self, draw: &mut D, mask_bits: u64) {
        let mut context = DebugDrawContext { draw, panic: None };
        let mut raw = unsafe { sys::b3DefaultDebugDraw() };
        raw.DrawShapeFcn = Some(draw_shape::<D>);
        raw.DrawSegmentFcn = Some(draw_segment::<D>);
        raw.DrawTransformFcn = Some(draw_transform::<D>);
        raw.DrawPointFcn = Some(draw_point::<D>);
        raw.DrawSphereFcn = Some(draw_sphere::<D>);
        raw.DrawCapsuleFcn = Some(draw_capsule::<D>);
        raw.DrawBoundsFcn = Some(draw_bounds::<D>);
        raw.DrawBoxFcn = Some(draw_box::<D>);
        raw.DrawStringFcn = Some(draw_string::<D>);
        raw.drawShapes = true;
        raw.drawJoints = true;
        raw.drawBounds = true;
        raw.drawMass = true;
        raw.context = (&mut context as *mut DebugDrawContext<'_, D>).cast();

        unsafe { sys::b3World_Draw(self.raw(), &mut raw, mask_bits) };

        if let Some(panic) = context.panic.take() {
            resume_unwind(panic);
        }
    }
}

unsafe extern "C" fn draw_shape<D: DebugDraw>(
    _user_shape: *mut c_void,
    transform: sys::b3WorldTransform,
    color: sys::b3HexColor,
    context: *mut c_void,
) -> bool {
    with_draw::<D, _>(context, false, |draw| {
        draw.draw_shape(transform.into(), color as u32)
    })
}

unsafe extern "C" fn draw_segment<D: DebugDraw>(
    p1: sys::b3Pos,
    p2: sys::b3Pos,
    color: sys::b3HexColor,
    context: *mut c_void,
) {
    with_draw::<D, _>(context, (), |draw| {
        draw.draw_segment(p1.into(), p2.into(), color as u32);
    });
}

unsafe extern "C" fn draw_transform<D: DebugDraw>(
    transform: sys::b3WorldTransform,
    context: *mut c_void,
) {
    with_draw::<D, _>(context, (), |draw| {
        draw.draw_transform(transform.into());
    });
}

unsafe extern "C" fn draw_point<D: DebugDraw>(
    point: sys::b3Pos,
    size: f32,
    color: sys::b3HexColor,
    context: *mut c_void,
) {
    with_draw::<D, _>(context, (), |draw| {
        draw.draw_point(point.into(), size, color as u32);
    });
}

unsafe extern "C" fn draw_sphere<D: DebugDraw>(
    center: sys::b3Pos,
    radius: f32,
    color: sys::b3HexColor,
    alpha: f32,
    context: *mut c_void,
) {
    with_draw::<D, _>(context, (), |draw| {
        draw.draw_sphere(center.into(), radius, color as u32, alpha);
    });
}

unsafe extern "C" fn draw_capsule<D: DebugDraw>(
    point1: sys::b3Pos,
    point2: sys::b3Pos,
    radius: f32,
    color: sys::b3HexColor,
    alpha: f32,
    context: *mut c_void,
) {
    with_draw::<D, _>(context, (), |draw| {
        draw.draw_capsule(point1.into(), point2.into(), radius, color as u32, alpha);
    });
}

unsafe extern "C" fn draw_bounds<D: DebugDraw>(
    bounds: sys::b3AABB,
    color: sys::b3HexColor,
    context: *mut c_void,
) {
    with_draw::<D, _>(context, (), |draw| {
        draw.draw_bounds(bounds.into(), color as u32);
    });
}

unsafe extern "C" fn draw_box<D: DebugDraw>(
    extents: sys::b3Vec3,
    transform: sys::b3WorldTransform,
    color: sys::b3HexColor,
    context: *mut c_void,
) {
    with_draw::<D, _>(context, (), |draw| {
        draw.draw_box(extents.into(), transform.into(), color as u32);
    });
}

unsafe extern "C" fn draw_string<D: DebugDraw>(
    point: sys::b3Pos,
    text: *const c_char,
    color: sys::b3HexColor,
    context: *mut c_void,
) {
    if text.is_null() {
        return;
    }

    with_draw::<D, _>(context, (), |draw| {
        let text = unsafe { CStr::from_ptr(text) }.to_string_lossy();
        draw.draw_string(point.into(), &text, color as u32);
    });
}

fn with_draw<D, R>(context: *mut c_void, panicked: R, f: impl FnOnce(&mut D) -> R) -> R
where
    D: DebugDraw,
{
    let context = unsafe { &mut *context.cast::<DebugDrawContext<'_, D>>() };
    if context.panic.is_some() {
        return panicked;
    }

    match catch_unwind(AssertUnwindSafe(|| f(context.draw))) {
        Ok(value) => value,
        Err(panic) => {
            context.panic = Some(panic);
            panicked
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, DistanceJointDef, ShapeDef};

    #[derive(Default)]
    struct Collector {
        bounds: usize,
        segments: usize,
        transforms: usize,
    }

    impl DebugDraw for Collector {
        fn draw_bounds(&mut self, _bounds: Aabb, _color: u32) {
            self.bounds += 1;
        }

        fn draw_segment(&mut self, _p1: Vec3, _p2: Vec3, _color: u32) {
            self.segments += 1;
        }

        fn draw_transform(&mut self, _transform: Transform) {
            self.transforms += 1;
        }
    }

    #[test]
    fn draw_reports_bounds_and_joint_segments() {
        let world = World::new(Vec3::ZERO);
        let body_a = world.create_body(BodyDef::dynamic_at(Vec3::new(-1.0, 0.0, 0.0)));
        let body_b = world.create_body(BodyDef::dynamic_at(Vec3::new(1.0, 0.0, 0.0)));
        let _shape_a = body_a.create_box(
            Vec3::new(0.5, 0.5, 0.5),
            ShapeDef {
                density: 1.0,
                ..ShapeDef::default()
            },
        );
        let _shape_b = body_b.create_box(
            Vec3::new(0.5, 0.5, 0.5),
            ShapeDef {
                density: 1.0,
                ..ShapeDef::default()
            },
        );
        let _joint = world.create_distance_joint(DistanceJointDef::new(&body_a, &body_b));

        let mut draw = Collector::default();
        world.draw(&mut draw, DEFAULT_DEBUG_MASK);

        assert!(draw.bounds >= 2, "{:?}", draw.bounds);
        assert!(draw.segments > 0, "{:?}", draw.segments);
        assert!(draw.transforms > 0, "{:?}", draw.transforms);
    }
}
