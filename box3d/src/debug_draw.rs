use std::{
    any::Any,
    ffi::{c_char, c_void, CStr},
    panic::{catch_unwind, resume_unwind, AssertUnwindSafe},
    ptr::NonNull,
};

use box3d_sys as sys;

use crate::{
    math::{Aabb, Transform, Vec3},
    world::World,
};

pub const DEFAULT_DEBUG_MASK: u64 = u64::MAX;

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DebugMaterial {
    Default,
    Matte,
    Soft,
    Dead,
    Glossy,
    Metallic,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct DebugShapeHandle(NonNull<c_void>);

impl DebugShapeHandle {
    pub fn from_ptr(ptr: *mut c_void) -> Option<Self> {
        NonNull::new(ptr).map(Self)
    }

    pub fn as_ptr(self) -> *mut c_void {
        self.0.as_ptr()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DebugDrawOptions {
    pub drawing_bounds: Aabb,
    pub force_scale: f32,
    pub joint_scale: f32,
    pub draw_shapes: bool,
    pub draw_joints: bool,
    pub draw_joint_extras: bool,
    pub draw_bounds: bool,
    pub draw_mass: bool,
    pub draw_body_names: bool,
    pub draw_contacts: bool,
    pub draw_anchor_a: i32,
    pub draw_graph_colors: bool,
    pub draw_contact_features: bool,
    pub draw_contact_normals: bool,
    pub draw_contact_forces: bool,
    pub draw_friction_forces: bool,
    pub draw_islands: bool,
}

impl Default for DebugDrawOptions {
    fn default() -> Self {
        let raw = unsafe { sys::b3DefaultDebugDraw() };
        Self {
            drawing_bounds: raw.drawingBounds.into(),
            force_scale: raw.forceScale,
            joint_scale: raw.jointScale,
            draw_shapes: true,
            draw_joints: true,
            draw_joint_extras: raw.drawJointExtras,
            draw_bounds: true,
            draw_mass: true,
            draw_body_names: raw.drawBodyNames,
            draw_contacts: raw.drawContacts,
            draw_anchor_a: raw.drawAnchorA,
            draw_graph_colors: raw.drawGraphColors,
            draw_contact_features: raw.drawContactFeatures,
            draw_contact_normals: raw.drawContactNormals,
            draw_contact_forces: raw.drawContactForces,
            draw_friction_forces: raw.drawFrictionForces,
            draw_islands: raw.drawIslands,
        }
    }
}

impl DebugDrawOptions {
    fn apply_to(self, raw: &mut sys::b3DebugDraw) {
        raw.drawingBounds = self.drawing_bounds.into();
        raw.forceScale = self.force_scale;
        raw.jointScale = self.joint_scale;
        raw.drawShapes = self.draw_shapes;
        raw.drawJoints = self.draw_joints;
        raw.drawJointExtras = self.draw_joint_extras;
        raw.drawBounds = self.draw_bounds;
        raw.drawMass = self.draw_mass;
        raw.drawBodyNames = self.draw_body_names;
        raw.drawContacts = self.draw_contacts;
        raw.drawAnchorA = self.draw_anchor_a;
        raw.drawGraphColors = self.draw_graph_colors;
        raw.drawContactFeatures = self.draw_contact_features;
        raw.drawContactNormals = self.draw_contact_normals;
        raw.drawContactForces = self.draw_contact_forces;
        raw.drawFrictionForces = self.draw_friction_forces;
        raw.drawIslands = self.draw_islands;
    }
}

pub trait DebugDraw {
    fn draw_shape(&mut self, _transform: Transform, _color: u32) -> bool {
        true
    }

    fn draw_shape_with_handle(
        &mut self,
        _shape: DebugShapeHandle,
        transform: Transform,
        color: u32,
    ) -> bool {
        self.draw_shape(transform, color)
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
        with_raw_debug_draw(draw, |raw| unsafe {
            sys::b3World_Draw(self.raw(), raw, mask_bits)
        });
    }

    pub fn draw_with_options<D: DebugDraw>(
        &self,
        draw: &mut D,
        mask_bits: u64,
        options: DebugDrawOptions,
    ) {
        with_raw_debug_draw_options(draw, options, |raw| unsafe {
            sys::b3World_Draw(self.raw(), raw, mask_bits)
        });
    }
}

pub fn graph_color(index: i32) -> u32 {
    unsafe { sys::b3GetGraphColor(index) as u32 }
}

pub fn make_debug_color(rgb: u32, material: DebugMaterial) -> u32 {
    (rgb & 0x00ff_ffff) | ((material as u32) << 24)
}

pub(crate) fn with_raw_debug_draw<D, R>(
    draw: &mut D,
    f: impl FnOnce(&mut sys::b3DebugDraw) -> R,
) -> R
where
    D: DebugDraw,
{
    with_raw_debug_draw_options(draw, DebugDrawOptions::default(), f)
}

pub(crate) fn with_raw_debug_draw_options<D, R>(
    draw: &mut D,
    options: DebugDrawOptions,
    f: impl FnOnce(&mut sys::b3DebugDraw) -> R,
) -> R
where
    D: DebugDraw,
{
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
    options.apply_to(&mut raw);
    raw.context = (&mut context as *mut DebugDrawContext<'_, D>).cast();

    let result = f(&mut raw);

    if let Some(panic) = context.panic.take() {
        resume_unwind(panic);
    }

    result
}

unsafe extern "C" fn draw_shape<D: DebugDraw>(
    user_shape: *mut c_void,
    transform: sys::b3WorldTransform,
    color: sys::b3HexColor,
    context: *mut c_void,
) -> bool {
    with_draw::<D, _>(context, false, |draw| {
        match DebugShapeHandle::from_ptr(user_shape) {
            Some(handle) => draw.draw_shape_with_handle(handle, transform.into(), color as u32),
            None => draw.draw_shape(transform.into(), color as u32),
        }
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
        shapes: usize,
        transforms: usize,
    }

    impl DebugDraw for Collector {
        fn draw_bounds(&mut self, _bounds: Aabb, _color: u32) {
            self.bounds += 1;
        }

        fn draw_segment(&mut self, _p1: Vec3, _p2: Vec3, _color: u32) {
            self.segments += 1;
        }

        fn draw_shape_with_handle(
            &mut self,
            _shape: DebugShapeHandle,
            _transform: Transform,
            _color: u32,
        ) -> bool {
            self.shapes += 1;
            true
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

    #[test]
    fn draw_with_options_can_disable_bounds() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let _shape = body.create_box(
            Vec3::new(0.5, 0.5, 0.5),
            ShapeDef {
                density: 1.0,
                ..ShapeDef::default()
            },
        );

        let mut draw = Collector::default();
        world.draw_with_options(
            &mut draw,
            DEFAULT_DEBUG_MASK,
            DebugDrawOptions {
                draw_bounds: false,
                ..DebugDrawOptions::default()
            },
        );

        assert_eq!(draw.bounds, 0);
    }

    #[test]
    fn debug_color_helpers_match_native_layout() {
        assert_eq!(
            make_debug_color(0xff_123456, DebugMaterial::Metallic),
            0x0512_3456
        );
        assert_ne!(graph_color(0), 0);
    }

    #[test]
    fn draw_shape_trampoline_forwards_user_shape_handle() {
        let mut draw = Collector::default();
        let mut context = DebugDrawContext {
            draw: &mut draw,
            panic: None,
        };

        let ok = unsafe {
            draw_shape::<Collector>(
                std::ptr::dangling_mut(),
                Transform::IDENTITY.into(),
                0x00ff00,
                (&mut context as *mut DebugDrawContext<'_, Collector>).cast(),
            )
        };

        assert!(ok);
        assert_eq!(draw.shapes, 1);
    }
}
