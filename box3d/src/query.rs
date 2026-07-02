use std::{
    any::Any,
    ffi::c_void,
    marker::PhantomData,
    panic::{catch_unwind, resume_unwind, AssertUnwindSafe},
    ptr, slice,
};

use box3d_sys as sys;

use crate::{
    body::Body,
    handle,
    math::{Aabb, Plane, Transform, Vec3},
    world::World,
    Error, Result,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QueryFilter {
    pub category_bits: u64,
    pub mask_bits: u64,
    pub id: u64,
}

impl Default for QueryFilter {
    fn default() -> Self {
        unsafe { sys::b3DefaultQueryFilter() }.into()
    }
}

impl From<sys::b3QueryFilter> for QueryFilter {
    fn from(value: sys::b3QueryFilter) -> Self {
        Self {
            category_bits: value.categoryBits,
            mask_bits: value.maskBits,
            id: value.id,
        }
    }
}

impl From<QueryFilter> for sys::b3QueryFilter {
    fn from(value: QueryFilter) -> Self {
        Self {
            categoryBits: value.category_bits,
            maskBits: value.mask_bits,
            id: value.id,
            name: ptr::null(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    pub point: Vec3,
    pub normal: Vec3,
    pub user_material_id: u64,
    pub fraction: f32,
    pub triangle_index: i32,
    pub child_index: i32,
    pub node_visits: i32,
    pub leaf_visits: i32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QueryStats {
    pub node_visits: i32,
    pub leaf_visits: i32,
}

impl From<sys::b3TreeStats> for QueryStats {
    fn from(value: sys::b3TreeStats) -> Self {
        Self {
            node_visits: value.nodeVisits,
            leaf_visits: value.leafVisits,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ShapeProxy<'a> {
    points: &'a [Vec3],
    radius: f32,
}

impl<'a> ShapeProxy<'a> {
    pub fn new(points: &'a [Vec3], radius: f32) -> Result<Self> {
        if points.is_empty()
            || points.len() > sys::B3_MAX_SHAPE_CAST_POINTS as usize
            || radius < 0.0
            || !radius.is_finite()
            || points
                .iter()
                .any(|point| !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite())
        {
            return Err(Error::InvalidInput);
        }

        Ok(Self { points, radius })
    }

    pub(crate) fn raw_points(self) -> Vec<sys::b3Vec3> {
        self.points.iter().copied().map(Into::into).collect()
    }

    pub(crate) fn raw(self, points: &[sys::b3Vec3]) -> sys::b3ShapeProxy {
        sys::b3ShapeProxy {
            points: points.as_ptr(),
            count: points.len() as i32,
            radius: self.radius,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ShapeRef<'a> {
    raw: sys::b3ShapeId,
    _marker: PhantomData<&'a mut ()>,
}

impl ShapeRef<'_> {
    fn from_raw(raw: sys::b3ShapeId) -> Self {
        Self {
            raw,
            _marker: PhantomData,
        }
    }

    pub fn is_valid(self) -> bool {
        handle::is_shape_valid(self.raw)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MoverPlane {
    pub plane: Plane,
    pub point: Vec3,
}

impl From<sys::b3PlaneResult> for MoverPlane {
    fn from(value: sys::b3PlaneResult) -> Self {
        Self {
            plane: value.plane.into(),
            point: value.point.into(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CastHit<'a> {
    pub shape: ShapeRef<'a>,
    pub point: Vec3,
    pub normal: Vec3,
    pub user_material_id: u64,
    pub fraction: f32,
    pub triangle_index: i32,
    pub child_index: i32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyClosestPoint {
    pub point: Vec3,
    pub distance: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyCastHit<'a> {
    pub shape: ShapeRef<'a>,
    pub point: Vec3,
    pub normal: Vec3,
    pub user_material_id: u64,
    pub fraction: f32,
    pub triangle_index: i32,
    pub iterations: i32,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyPlane<'a> {
    pub shape: ShapeRef<'a>,
    pub plane: MoverPlane,
}

type CallbackPanic = Box<dyn Any + Send + 'static>;

struct ShapeCallbackContext<'a, F> {
    f: &'a mut F,
    panic: Option<CallbackPanic>,
}

unsafe extern "C" fn shape_callback<F>(shape_id: sys::b3ShapeId, context: *mut c_void) -> bool
where
    F: for<'shape> FnMut(ShapeRef<'shape>) -> bool,
{
    let context = unsafe { &mut *context.cast::<ShapeCallbackContext<'_, F>>() };
    if context.panic.is_some() {
        return false;
    }

    match catch_unwind(AssertUnwindSafe(|| {
        (context.f)(ShapeRef::from_raw(shape_id))
    })) {
        Ok(keep_going) => keep_going,
        Err(panic) => {
            context.panic = Some(panic);
            false
        }
    }
}

struct CastCallbackContext<'a, F> {
    f: &'a mut F,
    panic: Option<CallbackPanic>,
}

unsafe extern "C" fn cast_callback<F>(
    shape_id: sys::b3ShapeId,
    point: sys::b3Pos,
    normal: sys::b3Vec3,
    fraction: f32,
    user_material_id: u64,
    triangle_index: i32,
    child_index: i32,
    context: *mut c_void,
) -> f32
where
    F: for<'shape> FnMut(CastHit<'shape>) -> f32,
{
    let context = unsafe { &mut *context.cast::<CastCallbackContext<'_, F>>() };
    if context.panic.is_some() {
        return 0.0;
    }

    match catch_unwind(AssertUnwindSafe(|| {
        (context.f)(CastHit {
            shape: ShapeRef::from_raw(shape_id),
            point: point.into(),
            normal: normal.into(),
            user_material_id,
            fraction,
            triangle_index,
            child_index,
        })
    })) {
        Ok(next_fraction) => next_fraction,
        Err(panic) => {
            context.panic = Some(panic);
            0.0
        }
    }
}

struct MoverPlaneContext<'a, F> {
    f: &'a mut F,
    panic: Option<CallbackPanic>,
}

unsafe extern "C" fn mover_plane_callback<F>(
    shape_id: sys::b3ShapeId,
    planes: *const sys::b3PlaneResult,
    plane_count: i32,
    context: *mut c_void,
) -> bool
where
    F: for<'shape> FnMut(ShapeRef<'shape>, MoverPlane) -> bool,
{
    if plane_count <= 0 {
        return true;
    }

    if planes.is_null() {
        return false;
    }

    let context = unsafe { &mut *context.cast::<MoverPlaneContext<'_, F>>() };
    if context.panic.is_some() {
        return false;
    }

    for plane in unsafe { slice::from_raw_parts(planes, plane_count as usize) } {
        match catch_unwind(AssertUnwindSafe(|| {
            (context.f)(ShapeRef::from_raw(shape_id), (*plane).into())
        })) {
            Ok(true) => {}
            Ok(false) => return false,
            Err(panic) => {
                context.panic = Some(panic);
                return false;
            }
        }
    }

    true
}

fn resume_callback_panic(panic: Option<CallbackPanic>) {
    if let Some(panic) = panic {
        resume_unwind(panic);
    }
}

fn raw_mover(points: [Vec3; 2], radius: f32) -> sys::b3Capsule {
    assert!(radius > 0.0);
    sys::b3Capsule {
        center1: points[0].into(),
        center2: points[1].into(),
        radius,
    }
}

impl RayHit {
    fn from_raw(value: sys::b3RayResult) -> Option<Self> {
        value.hit.then(|| Self {
            point: value.point.into(),
            normal: value.normal.into(),
            user_material_id: value.userMaterialId,
            fraction: value.fraction,
            triangle_index: value.triangleIndex,
            child_index: value.childIndex,
            node_visits: value.nodeVisits,
            leaf_visits: value.leafVisits,
        })
    }
}

impl BodyCastHit<'_> {
    fn from_raw(value: sys::b3BodyCastResult) -> Option<Self> {
        value.hit.then(|| Self {
            shape: ShapeRef::from_raw(value.shapeId),
            point: value.point.into(),
            normal: value.normal.into(),
            user_material_id: value.userMaterialId,
            fraction: value.fraction,
            triangle_index: value.triangleIndex,
            iterations: value.iterations,
        })
    }
}

impl BodyPlane<'_> {
    fn from_raw(value: sys::b3BodyPlaneResult) -> Self {
        Self {
            shape: ShapeRef::from_raw(value.shapeId),
            plane: value.result.into(),
        }
    }
}

impl World {
    pub fn overlap_aabb<F>(&self, aabb: Aabb, filter: QueryFilter, mut f: F) -> QueryStats
    where
        F: for<'shape> FnMut(ShapeRef<'shape>) -> bool,
    {
        let mut context = ShapeCallbackContext {
            f: &mut f,
            panic: None,
        };
        let stats = unsafe {
            sys::b3World_OverlapAABB(
                self.raw(),
                aabb.into(),
                filter.into(),
                Some(shape_callback::<F>),
                (&mut context as *mut ShapeCallbackContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
        stats.into()
    }

    pub fn overlap_shape<F>(
        &self,
        origin: Vec3,
        proxy: ShapeProxy<'_>,
        filter: QueryFilter,
        mut f: F,
    ) -> QueryStats
    where
        F: for<'shape> FnMut(ShapeRef<'shape>) -> bool,
    {
        let raw_points = proxy.raw_points();
        let raw_proxy = proxy.raw(&raw_points);
        let mut context = ShapeCallbackContext {
            f: &mut f,
            panic: None,
        };
        let stats = unsafe {
            sys::b3World_OverlapShape(
                self.raw(),
                origin.into(),
                &raw_proxy,
                filter.into(),
                Some(shape_callback::<F>),
                (&mut context as *mut ShapeCallbackContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
        stats.into()
    }

    pub fn cast_ray<F>(
        &self,
        origin: Vec3,
        translation: Vec3,
        filter: QueryFilter,
        mut f: F,
    ) -> QueryStats
    where
        F: for<'shape> FnMut(CastHit<'shape>) -> f32,
    {
        let mut context = CastCallbackContext {
            f: &mut f,
            panic: None,
        };
        let stats = unsafe {
            sys::b3World_CastRay(
                self.raw(),
                origin.into(),
                translation.into(),
                filter.into(),
                Some(cast_callback::<F>),
                (&mut context as *mut CastCallbackContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
        stats.into()
    }

    pub fn cast_ray_closest(
        &self,
        origin: Vec3,
        translation: Vec3,
        filter: QueryFilter,
    ) -> Option<RayHit> {
        let hit = unsafe {
            sys::b3World_CastRayClosest(
                self.raw(),
                origin.into(),
                translation.into(),
                filter.into(),
            )
        };
        RayHit::from_raw(hit)
    }

    pub fn cast_shape<F>(
        &self,
        origin: Vec3,
        proxy: ShapeProxy<'_>,
        translation: Vec3,
        filter: QueryFilter,
        mut f: F,
    ) -> QueryStats
    where
        F: for<'shape> FnMut(CastHit<'shape>) -> f32,
    {
        let raw_points = proxy.raw_points();
        let raw_proxy = proxy.raw(&raw_points);
        let mut context = CastCallbackContext {
            f: &mut f,
            panic: None,
        };
        let stats = unsafe {
            sys::b3World_CastShape(
                self.raw(),
                origin.into(),
                &raw_proxy,
                translation.into(),
                filter.into(),
                Some(cast_callback::<F>),
                (&mut context as *mut CastCallbackContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
        stats.into()
    }

    pub fn cast_mover(
        &self,
        origin: Vec3,
        points: [Vec3; 2],
        radius: f32,
        translation: Vec3,
        filter: QueryFilter,
    ) -> f32 {
        let mover = raw_mover(points, radius);
        unsafe {
            sys::b3World_CastMover(
                self.raw(),
                origin.into(),
                &mover,
                translation.into(),
                filter.into(),
                None,
                ptr::null_mut(),
            )
        }
    }

    pub fn cast_mover_filtered<F>(
        &self,
        origin: Vec3,
        points: [Vec3; 2],
        radius: f32,
        translation: Vec3,
        filter: QueryFilter,
        mut f: F,
    ) -> f32
    where
        F: for<'shape> FnMut(ShapeRef<'shape>) -> bool,
    {
        let mover = raw_mover(points, radius);
        let mut context = ShapeCallbackContext {
            f: &mut f,
            panic: None,
        };
        let fraction = unsafe {
            sys::b3World_CastMover(
                self.raw(),
                origin.into(),
                &mover,
                translation.into(),
                filter.into(),
                Some(shape_callback::<F>),
                (&mut context as *mut ShapeCallbackContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
        fraction
    }

    pub fn collide_mover<F>(
        &self,
        origin: Vec3,
        points: [Vec3; 2],
        radius: f32,
        filter: QueryFilter,
        mut f: F,
    ) where
        F: for<'shape> FnMut(ShapeRef<'shape>, MoverPlane) -> bool,
    {
        let mover = raw_mover(points, radius);
        let mut context = MoverPlaneContext {
            f: &mut f,
            panic: None,
        };
        unsafe {
            sys::b3World_CollideMover(
                self.raw(),
                origin.into(),
                &mover,
                filter.into(),
                Some(mover_plane_callback::<F>),
                (&mut context as *mut MoverPlaneContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
    }
}

impl Body<'_> {
    pub fn compute_aabb(&self) -> Aabb {
        unsafe { sys::b3Body_ComputeAABB(self.raw()) }.into()
    }

    pub fn closest_point(&self, target: Vec3) -> BodyClosestPoint {
        let mut point = sys::b3Vec3::default();
        let distance =
            unsafe { sys::b3Body_GetClosestPoint(self.raw(), &mut point, target.into()) };
        BodyClosestPoint {
            point: point.into(),
            distance,
        }
    }

    pub fn cast_ray_at_transform(
        &self,
        origin: Vec3,
        translation: Vec3,
        filter: QueryFilter,
        max_fraction: f32,
        body_transform: Transform,
    ) -> Option<BodyCastHit<'_>> {
        assert!(max_fraction.is_finite() && (0.0..=1.0).contains(&max_fraction));
        BodyCastHit::from_raw(unsafe {
            sys::b3Body_CastRay(
                self.raw(),
                origin.into(),
                translation.into(),
                filter.into(),
                max_fraction,
                body_transform.into(),
            )
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub fn cast_shape_at_transform(
        &self,
        origin: Vec3,
        proxy: ShapeProxy<'_>,
        translation: Vec3,
        filter: QueryFilter,
        max_fraction: f32,
        can_encroach: bool,
        body_transform: Transform,
    ) -> Option<BodyCastHit<'_>> {
        assert!(max_fraction.is_finite() && (0.0..=1.0).contains(&max_fraction));
        let raw_points = proxy.raw_points();
        let raw_proxy = proxy.raw(&raw_points);
        BodyCastHit::from_raw(unsafe {
            sys::b3Body_CastShape(
                self.raw(),
                origin.into(),
                &raw_proxy,
                translation.into(),
                filter.into(),
                max_fraction,
                can_encroach,
                body_transform.into(),
            )
        })
    }

    pub fn overlap_shape_at_transform(
        &self,
        origin: Vec3,
        proxy: ShapeProxy<'_>,
        filter: QueryFilter,
        body_transform: Transform,
    ) -> bool {
        let raw_points = proxy.raw_points();
        let raw_proxy = proxy.raw(&raw_points);
        unsafe {
            sys::b3Body_OverlapShape(
                self.raw(),
                origin.into(),
                &raw_proxy,
                filter.into(),
                body_transform.into(),
            )
        }
    }

    pub fn collide_mover_at_transform(
        &self,
        origin: Vec3,
        points: [Vec3; 2],
        radius: f32,
        filter: QueryFilter,
        body_transform: Transform,
        plane_capacity: usize,
    ) -> Vec<BodyPlane<'_>> {
        if plane_capacity == 0 {
            return Vec::new();
        }

        let capacity = plane_capacity.min(i32::MAX as usize) as i32;
        let mover = raw_mover(points, radius);
        let mut planes = vec![sys::b3BodyPlaneResult::default(); capacity as usize];
        let count = unsafe {
            sys::b3Body_CollideMover(
                self.raw(),
                planes.as_mut_ptr(),
                capacity,
                origin.into(),
                &mover,
                filter.into(),
                body_transform.into(),
            )
        };
        planes.truncate(count.max(0) as usize);
        planes.into_iter().map(BodyPlane::from_raw).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, ShapeDef};

    #[test]
    fn closest_ray_hits_static_box() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let _shape = body.create_box(Vec3::new(1.0, 1.0, 1.0), ShapeDef::default());

        let hit = world
            .cast_ray_closest(
                Vec3::new(-3.0, 0.0, 0.0),
                Vec3::new(6.0, 0.0, 0.0),
                QueryFilter::default(),
            )
            .expect("ray should hit box");

        assert!((hit.point.x + 1.0).abs() < 0.01, "{hit:?}");
        assert!(hit.normal.x < 0.0, "{hit:?}");
        assert!(hit.fraction > 0.0 && hit.fraction < 1.0, "{hit:?}");
    }

    #[test]
    fn closest_ray_misses_empty_world() {
        let world = World::new(Vec3::ZERO);

        let hit = world.cast_ray_closest(
            Vec3::new(-3.0, 0.0, 0.0),
            Vec3::new(6.0, 0.0, 0.0),
            QueryFilter::default(),
        );

        assert!(hit.is_none());
    }

    #[test]
    fn overlap_aabb_reports_shapes_in_bounds() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let _shape = body.create_box(Vec3::new(0.5, 0.5, 0.5), ShapeDef::default());
        let other = world.create_body(BodyDef::static_at(Vec3::new(5.0, 0.0, 0.0)));
        let _other_shape = other.create_box(Vec3::new(0.5, 0.5, 0.5), ShapeDef::default());

        let mut count = 0;
        let stats = world.overlap_aabb(
            Aabb {
                lower_bound: Vec3::new(-1.0, -1.0, -1.0),
                upper_bound: Vec3::new(1.0, 1.0, 1.0),
            },
            QueryFilter::default(),
            |shape| {
                assert!(shape.is_valid());
                count += 1;
                true
            },
        );

        assert_eq!(count, 1);
        assert!(stats.leaf_visits >= 1, "{stats:?}");
    }

    #[test]
    fn overlap_shape_reports_shapes_in_proxy() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let _shape = body.create_box(Vec3::new(0.5, 0.5, 0.5), ShapeDef::default());
        let other = world.create_body(BodyDef::static_at(Vec3::new(5.0, 0.0, 0.0)));
        let _other_shape = other.create_box(Vec3::new(0.5, 0.5, 0.5), ShapeDef::default());
        let points = [Vec3::ZERO];
        let proxy = ShapeProxy::new(&points, 1.0).unwrap();

        let mut count = 0;
        let stats = world.overlap_shape(Vec3::ZERO, proxy, QueryFilter::default(), |shape| {
            assert!(shape.is_valid());
            count += 1;
            true
        });

        assert_eq!(count, 1);
        assert!(stats.leaf_visits >= 1, "{stats:?}");
    }

    #[test]
    fn streaming_ray_cast_reports_and_clips_hits() {
        let world = World::new(Vec3::ZERO);
        let near = world.create_body(BodyDef::static_at(Vec3::new(0.0, 0.0, 0.0)));
        let _near_shape = near.create_box(Vec3::new(0.5, 0.5, 0.5), ShapeDef::default());
        let far = world.create_body(BodyDef::static_at(Vec3::new(3.0, 0.0, 0.0)));
        let _far_shape = far.create_box(Vec3::new(0.5, 0.5, 0.5), ShapeDef::default());

        let mut fractions = Vec::new();
        let stats = world.cast_ray(
            Vec3::new(-3.0, 0.0, 0.0),
            Vec3::new(8.0, 0.0, 0.0),
            QueryFilter::default(),
            |hit| {
                assert!(hit.shape.is_valid());
                assert!(hit.fraction > 0.0 && hit.fraction < 1.0, "{hit:?}");
                fractions.push(hit.fraction);
                hit.fraction
            },
        );

        assert!(!fractions.is_empty());
        assert!(stats.leaf_visits >= 1, "{stats:?}");
    }

    #[test]
    fn cast_mover_hits_static_wall() {
        let world = World::new(Vec3::ZERO);
        let wall = world.create_body(BodyDef::static_at(Vec3::new(2.0, 0.0, 0.0)));
        let _wall_shape = wall.create_box(Vec3::new(0.5, 2.0, 2.0), ShapeDef::default());

        let fraction = world.cast_mover(
            Vec3::ZERO,
            [Vec3::new(0.0, -0.5, 0.0), Vec3::new(0.0, 0.5, 0.0)],
            0.25,
            Vec3::new(4.0, 0.0, 0.0),
            QueryFilter::default(),
        );

        assert!(fraction > 0.0 && fraction < 1.0, "{fraction}");
    }

    #[test]
    fn cast_shape_hits_static_wall() {
        let world = World::new(Vec3::ZERO);
        let wall = world.create_body(BodyDef::static_at(Vec3::new(2.0, 0.0, 0.0)));
        let _wall_shape = wall.create_box(Vec3::new(0.5, 2.0, 2.0), ShapeDef::default());
        let points = [Vec3::ZERO];
        let proxy = ShapeProxy::new(&points, 0.25).unwrap();

        let mut hit_fraction = 1.0;
        let stats = world.cast_shape(
            Vec3::ZERO,
            proxy,
            Vec3::new(4.0, 0.0, 0.0),
            QueryFilter::default(),
            |hit| {
                assert!(hit.shape.is_valid());
                hit_fraction = hit.fraction;
                hit.fraction
            },
        );

        assert!(hit_fraction > 0.0 && hit_fraction < 1.0, "{hit_fraction}");
        assert!(stats.leaf_visits >= 1, "{stats:?}");
    }

    #[test]
    fn collide_mover_reports_planes() {
        let world = World::new(Vec3::ZERO);
        let wall = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let _wall_shape = wall.create_box(Vec3::new(0.5, 0.5, 0.5), ShapeDef::default());

        let mut count = 0;
        let mut saw_top_face = false;
        world.collide_mover(
            Vec3::ZERO,
            [Vec3::new(-0.3, 0.6, 0.0), Vec3::new(0.3, 0.6, 0.0)],
            0.2,
            QueryFilter::default(),
            |shape, plane| {
                assert!(shape.is_valid());
                count += 1;
                saw_top_face |= plane.plane.normal.y > 0.9;
                true
            },
        );

        assert!(count >= 1);
        assert!(saw_top_face);
    }

    #[test]
    fn body_queries_hit_attached_shape() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let _shape = body.create_box(Vec3::new(0.5, 0.5, 0.5), ShapeDef::default());
        let transform = body.transform();

        let aabb = body.compute_aabb();
        assert!(aabb.lower_bound.x < 0.0 && aabb.upper_bound.x > 0.0);

        let closest = body.closest_point(Vec3::new(2.0, 0.0, 0.0));
        assert!((closest.point.x - 0.5).abs() < 0.01, "{closest:?}");
        assert!(closest.distance > 0.0, "{closest:?}");

        let ray_hit = body
            .cast_ray_at_transform(
                Vec3::new(-3.0, 0.0, 0.0),
                Vec3::new(6.0, 0.0, 0.0),
                QueryFilter::default(),
                1.0,
                transform,
            )
            .expect("ray should hit body");
        assert!(ray_hit.shape.is_valid());
        assert!(ray_hit.fraction > 0.0 && ray_hit.fraction < 1.0);

        let points = [Vec3::ZERO];
        let proxy = ShapeProxy::new(&points, 0.25).unwrap();
        let shape_hit = body
            .cast_shape_at_transform(
                Vec3::new(-3.0, 0.0, 0.0),
                proxy,
                Vec3::new(6.0, 0.0, 0.0),
                QueryFilter::default(),
                1.0,
                false,
                transform,
            )
            .expect("shape cast should hit body");
        assert!(shape_hit.shape.is_valid());
        assert!(shape_hit.fraction > 0.0 && shape_hit.fraction < 1.0);

        let overlap_proxy = ShapeProxy::new(&points, 1.0).unwrap();
        assert!(body.overlap_shape_at_transform(
            Vec3::ZERO,
            overlap_proxy,
            QueryFilter::default(),
            transform
        ));

        let planes = body.collide_mover_at_transform(
            Vec3::ZERO,
            [Vec3::new(-0.3, 0.6, 0.0), Vec3::new(0.3, 0.6, 0.0)],
            0.2,
            QueryFilter::default(),
            transform,
            8,
        );
        assert!(planes.iter().any(|plane| plane.shape.is_valid()));
    }
}
