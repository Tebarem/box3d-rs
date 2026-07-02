use box3d_sys as sys;

use crate::{
    compound::Compound,
    hull::HullRef,
    math::{Aabb, MassData, Quat, Transform, Vec3},
    mesh::{HeightField, Mesh},
    query::ShapeProxy,
};

const LOCAL_MANIFOLD_CAPACITY: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sphere {
    pub center: Vec3,
    pub radius: f32,
}

impl Sphere {
    pub fn new(center: Vec3, radius: f32) -> Self {
        assert!(radius > 0.0);
        Self { center, radius }
    }

    pub fn compute_aabb(self, transform: Transform) -> Aabb {
        compute_sphere_aabb(self, transform)
    }

    pub fn compute_mass(self, density: f32) -> MassData {
        compute_sphere_mass(self, density)
    }

    pub(crate) fn raw(self) -> sys::b3Sphere {
        assert!(self.radius > 0.0);
        sys::b3Sphere {
            center: self.center.into(),
            radius: self.radius,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Capsule {
    pub point1: Vec3,
    pub point2: Vec3,
    pub radius: f32,
}

impl Capsule {
    pub fn new(point1: Vec3, point2: Vec3, radius: f32) -> Self {
        assert!(radius > 0.0);
        Self {
            point1,
            point2,
            radius,
        }
    }

    pub fn compute_aabb(self, transform: Transform) -> Aabb {
        compute_capsule_aabb(self, transform)
    }

    pub fn compute_mass(self, density: f32) -> MassData {
        compute_capsule_mass(self, density)
    }

    pub(crate) fn raw(self) -> sys::b3Capsule {
        assert!(self.radius > 0.0);
        sys::b3Capsule {
            center1: self.point1.into(),
            center2: self.point2.into(),
            radius: self.radius,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxShape {
    pub half_extents: Vec3,
}

impl BoxShape {
    pub fn new(half_extents: Vec3) -> Self {
        assert!(half_extents.x > 0.0 && half_extents.y > 0.0 && half_extents.z > 0.0);
        Self { half_extents }
    }

    pub fn compute_aabb(self, transform: Transform) -> Aabb {
        compute_box_aabb(self, transform)
    }

    pub fn compute_mass(self, density: f32) -> MassData {
        compute_box_mass(self, density)
    }

    fn raw(self) -> sys::b3BoxHull {
        let h = self.half_extents;
        assert!(h.x > 0.0 && h.y > 0.0 && h.z > 0.0);
        unsafe { sys::b3MakeBoxHull(h.x, h.y, h.z) }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SimpleShape {
    Sphere(Sphere),
    Capsule(Capsule),
    Box(BoxShape),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DistanceOutput {
    pub point_a: Vec3,
    pub point_b: Vec3,
    pub normal: Vec3,
    pub distance: f32,
    pub iterations: i32,
    pub simplex_count: i32,
}

impl From<sys::b3DistanceOutput> for DistanceOutput {
    fn from(value: sys::b3DistanceOutput) -> Self {
        Self {
            point_a: value.pointA.into(),
            point_b: value.pointB.into(),
            normal: value.normal.into(),
            distance: value.distance,
            iterations: value.iterations,
            simplex_count: value.simplexCount,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShapeCastOutput {
    pub normal: Vec3,
    pub point: Vec3,
    pub fraction: f32,
    pub iterations: i32,
    pub triangle_index: i32,
    pub child_index: i32,
    pub material_index: i32,
}

impl ShapeCastOutput {
    pub(crate) fn from_raw(value: sys::b3CastOutput) -> Option<Self> {
        value.hit.then(|| Self {
            normal: value.normal.into(),
            point: value.point.into(),
            fraction: value.fraction,
            iterations: value.iterations,
            triangle_index: value.triangleIndex,
            child_index: value.childIndex,
            material_index: value.materialIndex,
        })
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FeaturePair {
    pub owner1: u8,
    pub index1: u8,
    pub owner2: u8,
    pub index2: u8,
}

impl From<sys::b3FeaturePair> for FeaturePair {
    fn from(value: sys::b3FeaturePair) -> Self {
        Self {
            owner1: value.owner1,
            index1: value.index1,
            owner2: value.owner2,
            index2: value.index2,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct LocalManifoldPoint {
    pub point: Vec3,
    pub separation: f32,
    pub pair: FeaturePair,
    pub triangle_index: i32,
}

impl From<sys::b3LocalManifoldPoint> for LocalManifoldPoint {
    fn from(value: sys::b3LocalManifoldPoint) -> Self {
        Self {
            point: value.point.into(),
            separation: value.separation,
            pair: value.pair.into(),
            triangle_index: value.triangleIndex,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct LocalManifold {
    pub normal: Vec3,
    pub triangle_normal: Vec3,
    pub points: Vec<LocalManifoldPoint>,
    pub triangle_index: i32,
    pub indices: [i32; 3],
    pub squared_distance: f32,
    pub feature: TriangleFeature,
    pub triangle_flags: MeshEdgeFlags,
}

impl LocalManifold {
    pub fn points(&self) -> &[LocalManifoldPoint] {
        &self.points
    }

    fn from_raw(raw: sys::b3LocalManifold, points: &[sys::b3LocalManifoldPoint]) -> Self {
        assert!(raw.pointCount <= points.len() as i32);
        let count = raw.pointCount.max(0) as usize;
        Self {
            normal: raw.normal.into(),
            triangle_normal: raw.triangleNormal.into(),
            points: points[..count].iter().copied().map(Into::into).collect(),
            triangle_index: raw.triangleIndex,
            indices: [raw.i1, raw.i2, raw.i3],
            squared_distance: raw.squaredDistance,
            feature: raw.feature.into(),
            triangle_flags: MeshEdgeFlags(raw.triangleFlags),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleFeature {
    None,
    TriangleFace,
    HullFace,
    Edge1,
    Edge2,
    Edge3,
    Vertex1,
    Vertex2,
    Vertex3,
}

impl From<sys::b3TriangleFeature> for TriangleFeature {
    fn from(value: sys::b3TriangleFeature) -> Self {
        match value {
            sys::b3TriangleFeature_b3_featureNone => Self::None,
            sys::b3TriangleFeature_b3_featureTriangleFace => Self::TriangleFace,
            sys::b3TriangleFeature_b3_featureHullFace => Self::HullFace,
            sys::b3TriangleFeature_b3_featureEdge1 => Self::Edge1,
            sys::b3TriangleFeature_b3_featureEdge2 => Self::Edge2,
            sys::b3TriangleFeature_b3_featureEdge3 => Self::Edge3,
            sys::b3TriangleFeature_b3_featureVertex1 => Self::Vertex1,
            sys::b3TriangleFeature_b3_featureVertex2 => Self::Vertex2,
            sys::b3TriangleFeature_b3_featureVertex3 => Self::Vertex3,
            _ => panic!("unknown box3d triangle feature {value}"),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct MeshEdgeFlags(pub i32);

impl MeshEdgeFlags {
    pub const CONCAVE_EDGE1: Self = Self(sys::b3MeshEdgeFlags_b3_concaveEdge1);
    pub const CONCAVE_EDGE2: Self = Self(sys::b3MeshEdgeFlags_b3_concaveEdge2);
    pub const CONCAVE_EDGE3: Self = Self(sys::b3MeshEdgeFlags_b3_concaveEdge3);
    pub const INVERSE_CONCAVE_EDGE1: Self = Self(sys::b3MeshEdgeFlags_b3_inverseConcaveEdge1);
    pub const INVERSE_CONCAVE_EDGE2: Self = Self(sys::b3MeshEdgeFlags_b3_inverseConcaveEdge2);
    pub const INVERSE_CONCAVE_EDGE3: Self = Self(sys::b3MeshEdgeFlags_b3_inverseConcaveEdge3);
    pub const ALL_CONCAVE_EDGES: Self = Self(sys::b3MeshEdgeFlags_b3_allConcaveEdges);
    pub const FLAT_EDGE1: Self = Self(sys::b3MeshEdgeFlags_b3_flatEdge1);
    pub const FLAT_EDGE2: Self = Self(sys::b3MeshEdgeFlags_b3_flatEdge2);
    pub const FLAT_EDGE3: Self = Self(sys::b3MeshEdgeFlags_b3_flatEdge3);
    pub const ALL_FLAT_EDGES: Self = Self(sys::b3MeshEdgeFlags_b3_allFlatEdges);

    pub const fn bits(self) -> i32 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sweep {
    pub local_center: Vec3,
    pub center1: Vec3,
    pub center2: Vec3,
    pub rotation1: Quat,
    pub rotation2: Quat,
}

impl Sweep {
    pub fn new(
        local_center: Vec3,
        center1: Vec3,
        center2: Vec3,
        rotation1: Quat,
        rotation2: Quat,
    ) -> Self {
        assert_valid_vec3(local_center);
        assert_valid_vec3(center1);
        assert_valid_vec3(center2);
        Self {
            local_center,
            center1,
            center2,
            rotation1,
            rotation2,
        }
    }

    fn raw(self) -> sys::b3Sweep {
        Self::new(
            self.local_center,
            self.center1,
            self.center2,
            self.rotation1,
            self.rotation2,
        );
        sys::b3Sweep {
            localCenter: self.local_center.into(),
            c1: self.center1.into(),
            c2: self.center2.into(),
            q1: self.rotation1.into(),
            q2: self.rotation2.into(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TimeOfImpactInput<'a> {
    pub proxy_a: ShapeProxy<'a>,
    pub proxy_b: ShapeProxy<'a>,
    pub sweep_a: Sweep,
    pub sweep_b: Sweep,
    pub max_fraction: f32,
}

impl<'a> TimeOfImpactInput<'a> {
    pub fn new(
        proxy_a: ShapeProxy<'a>,
        proxy_b: ShapeProxy<'a>,
        sweep_a: Sweep,
        sweep_b: Sweep,
        max_fraction: f32,
    ) -> Self {
        assert!((0.0..=1.0).contains(&max_fraction));
        Self {
            proxy_a,
            proxy_b,
            sweep_a,
            sweep_b,
            max_fraction,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimeOfImpactState {
    Unknown,
    Failed,
    Overlapped,
    Hit,
    Separated,
}

impl From<sys::b3TOIState> for TimeOfImpactState {
    fn from(value: sys::b3TOIState) -> Self {
        match value {
            sys::b3TOIState_b3_toiStateUnknown => Self::Unknown,
            sys::b3TOIState_b3_toiStateFailed => Self::Failed,
            sys::b3TOIState_b3_toiStateOverlapped => Self::Overlapped,
            sys::b3TOIState_b3_toiStateHit => Self::Hit,
            sys::b3TOIState_b3_toiStateSeparated => Self::Separated,
            _ => panic!("unknown box3d TOI state {value}"),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimeOfImpactOutput {
    pub state: TimeOfImpactState,
    pub point: Vec3,
    pub normal: Vec3,
    pub fraction: f32,
    pub distance: f32,
    pub distance_iterations: i32,
    pub push_back_iterations: i32,
    pub root_iterations: i32,
    pub used_fallback: bool,
}

impl From<sys::b3TOIOutput> for TimeOfImpactOutput {
    fn from(value: sys::b3TOIOutput) -> Self {
        Self {
            state: value.state.into(),
            point: value.point.into(),
            normal: value.normal.into(),
            fraction: value.fraction,
            distance: value.distance,
            distance_iterations: value.distanceIterations,
            push_back_iterations: value.pushBackIterations,
            root_iterations: value.rootIterations,
            used_fallback: value.usedFallback,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayCastInput {
    pub origin: Vec3,
    pub translation: Vec3,
    pub max_fraction: f32,
}

impl RayCastInput {
    pub fn new(origin: Vec3, translation: Vec3, max_fraction: f32) -> Self {
        assert_valid_vec3(origin);
        assert_valid_vec3(translation);
        assert!((0.0..=1.0).contains(&max_fraction));
        Self {
            origin,
            translation,
            max_fraction,
        }
    }

    fn raw(self) -> sys::b3RayCastInput {
        Self::new(self.origin, self.translation, self.max_fraction);
        sys::b3RayCastInput {
            origin: self.origin.into(),
            translation: self.translation.into(),
            maxFraction: self.max_fraction,
        }
    }

    fn raw_unchecked(self) -> sys::b3RayCastInput {
        sys::b3RayCastInput {
            origin: self.origin.into(),
            translation: self.translation.into(),
            maxFraction: self.max_fraction,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ShapeCastInput<'a> {
    pub proxy: ShapeProxy<'a>,
    pub translation: Vec3,
    pub max_fraction: f32,
    pub can_encroach: bool,
}

impl<'a> ShapeCastInput<'a> {
    pub fn new(
        proxy: ShapeProxy<'a>,
        translation: Vec3,
        max_fraction: f32,
        can_encroach: bool,
    ) -> Self {
        assert_valid_vec3(translation);
        assert!((0.0..=1.0).contains(&max_fraction));
        Self {
            proxy,
            translation,
            max_fraction,
            can_encroach,
        }
    }

    fn raw(self, points: &[sys::b3Vec3]) -> sys::b3ShapeCastInput {
        Self::new(
            self.proxy,
            self.translation,
            self.max_fraction,
            self.can_encroach,
        );
        sys::b3ShapeCastInput {
            proxy: self.proxy.raw(points),
            translation: self.translation.into(),
            maxFraction: self.max_fraction,
            canEncroach: self.can_encroach,
        }
    }
}

impl From<sys::b3Sphere> for Sphere {
    fn from(value: sys::b3Sphere) -> Self {
        Self {
            center: value.center.into(),
            radius: value.radius,
        }
    }
}

impl From<sys::b3Capsule> for Capsule {
    fn from(value: sys::b3Capsule) -> Self {
        Self {
            point1: value.center1.into(),
            point2: value.center2.into(),
            radius: value.radius,
        }
    }
}

impl From<Sphere> for SimpleShape {
    fn from(value: Sphere) -> Self {
        Self::Sphere(value)
    }
}

impl From<Capsule> for SimpleShape {
    fn from(value: Capsule) -> Self {
        Self::Capsule(value)
    }
}

impl From<BoxShape> for SimpleShape {
    fn from(value: BoxShape) -> Self {
        Self::Box(value)
    }
}

pub fn compute_aabb(shape: impl Into<SimpleShape>, transform: Transform) -> Aabb {
    match shape.into() {
        SimpleShape::Sphere(shape) => compute_sphere_aabb(shape, transform),
        SimpleShape::Capsule(shape) => compute_capsule_aabb(shape, transform),
        SimpleShape::Box(shape) => compute_box_aabb(shape, transform),
    }
}

pub fn compute_mass(shape: impl Into<SimpleShape>, density: f32) -> MassData {
    match shape.into() {
        SimpleShape::Sphere(shape) => compute_sphere_mass(shape, density),
        SimpleShape::Capsule(shape) => compute_capsule_mass(shape, density),
        SimpleShape::Box(shape) => compute_box_mass(shape, density),
    }
}

pub fn is_valid_ray(input: RayCastInput) -> bool {
    let raw = input.raw_unchecked();
    unsafe { sys::b3IsValidRay(&raw) }
}

pub fn shape_distance(
    proxy_a: ShapeProxy<'_>,
    proxy_b: ShapeProxy<'_>,
    transform_b_to_a: Transform,
    use_radii: bool,
) -> DistanceOutput {
    let points_a = proxy_a.raw_points();
    let points_b = proxy_b.raw_points();
    let input = sys::b3DistanceInput {
        proxyA: proxy_a.raw(&points_a),
        proxyB: proxy_b.raw(&points_b),
        transform: transform_b_to_a.into(),
        useRadii: use_radii,
    };
    let mut cache = sys::b3SimplexCache {
        metric: 0.0,
        count: 0,
        indexA: [0; 4],
        indexB: [0; 4],
    };

    unsafe { sys::b3ShapeDistance(&input, &mut cache, std::ptr::null_mut(), 0) }.into()
}

pub fn shape_cast(
    proxy_a: ShapeProxy<'_>,
    proxy_b: ShapeProxy<'_>,
    transform_b_to_a: Transform,
    translation_b: Vec3,
    max_fraction: f32,
    can_encroach: bool,
) -> Option<ShapeCastOutput> {
    assert!((0.0..=1.0).contains(&max_fraction));
    let points_a = proxy_a.raw_points();
    let points_b = proxy_b.raw_points();
    let input = sys::b3ShapeCastPairInput {
        proxyA: proxy_a.raw(&points_a),
        proxyB: proxy_b.raw(&points_b),
        transform: transform_b_to_a.into(),
        translationB: translation_b.into(),
        maxFraction: max_fraction,
        canEncroach: can_encroach,
    };

    ShapeCastOutput::from_raw(unsafe { sys::b3ShapeCast(&input) })
}

pub fn compute_sphere_aabb(shape: Sphere, transform: Transform) -> Aabb {
    let raw = shape.raw();
    unsafe { sys::b3ComputeSphereAABB(&raw, transform.into()) }.into()
}

pub fn compute_sphere_mass(shape: Sphere, density: f32) -> MassData {
    let raw = shape.raw();
    unsafe { sys::b3ComputeSphereMass(&raw, density) }.into()
}

pub fn compute_capsule_aabb(shape: Capsule, transform: Transform) -> Aabb {
    let raw = shape.raw();
    unsafe { sys::b3ComputeCapsuleAABB(&raw, transform.into()) }.into()
}

pub fn compute_capsule_mass(shape: Capsule, density: f32) -> MassData {
    let raw = shape.raw();
    unsafe { sys::b3ComputeCapsuleMass(&raw, density) }.into()
}

pub fn compute_box_aabb(shape: BoxShape, transform: Transform) -> Aabb {
    let raw = shape.raw();
    unsafe { sys::b3ComputeHullAABB(&raw.base, transform.into()) }.into()
}

pub fn compute_box_mass(shape: BoxShape, density: f32) -> MassData {
    let raw = shape.raw();
    unsafe { sys::b3ComputeHullMass(&raw.base, density) }.into()
}

pub fn compute_hull_aabb<'a>(hull: impl Into<HullRef<'a>>, transform: Transform) -> Aabb {
    unsafe { sys::b3ComputeHullAABB(hull.into().raw(), transform.into()) }.into()
}

pub fn compute_hull_mass<'a>(hull: impl Into<HullRef<'a>>, density: f32) -> MassData {
    unsafe { sys::b3ComputeHullMass(hull.into().raw(), density) }.into()
}

pub fn compute_mesh_aabb(mesh: &Mesh, transform: Transform, scale: Vec3) -> Aabb {
    assert_mesh_scale(scale);
    unsafe { sys::b3ComputeMeshAABB(mesh.raw(), transform.into(), scale.into()) }.into()
}

pub fn compute_height_field_aabb(height_field: &HeightField, transform: Transform) -> Aabb {
    unsafe { sys::b3ComputeHeightFieldAABB(height_field.raw(), transform.into()) }.into()
}

pub fn compute_compound_aabb(compound: &Compound, transform: Transform) -> Aabb {
    unsafe { sys::b3ComputeCompoundAABB(compound.raw(), transform.into()) }.into()
}

pub fn overlap_sphere(shape: Sphere, shape_transform: Transform, proxy: ShapeProxy<'_>) -> bool {
    let raw_shape = shape.raw();
    with_shape_proxy(proxy, |proxy| unsafe {
        sys::b3OverlapSphere(&raw_shape, shape_transform.into(), proxy)
    })
}

pub fn overlap_capsule(shape: Capsule, shape_transform: Transform, proxy: ShapeProxy<'_>) -> bool {
    let raw_shape = shape.raw();
    with_shape_proxy(proxy, |proxy| unsafe {
        sys::b3OverlapCapsule(&raw_shape, shape_transform.into(), proxy)
    })
}

pub fn overlap_hull<'a>(
    hull: impl Into<HullRef<'a>>,
    shape_transform: Transform,
    proxy: ShapeProxy<'_>,
) -> bool {
    let raw_hull = hull.into().raw();
    with_shape_proxy(proxy, |proxy| unsafe {
        sys::b3OverlapHull(raw_hull, shape_transform.into(), proxy)
    })
}

pub fn overlap_mesh(
    mesh: &Mesh,
    scale: Vec3,
    shape_transform: Transform,
    proxy: ShapeProxy<'_>,
) -> bool {
    let raw_mesh = raw_mesh(mesh, scale);
    with_shape_proxy(proxy, |proxy| unsafe {
        sys::b3OverlapMesh(&raw_mesh, shape_transform.into(), proxy)
    })
}

pub fn overlap_height_field(
    height_field: &HeightField,
    shape_transform: Transform,
    proxy: ShapeProxy<'_>,
) -> bool {
    with_shape_proxy(proxy, |proxy| unsafe {
        sys::b3OverlapHeightField(height_field.raw(), shape_transform.into(), proxy)
    })
}

pub fn overlap_compound(
    compound: &Compound,
    shape_transform: Transform,
    proxy: ShapeProxy<'_>,
) -> bool {
    with_shape_proxy(proxy, |proxy| unsafe {
        sys::b3OverlapCompound(compound.raw(), shape_transform.into(), proxy)
    })
}

pub fn ray_cast_sphere(shape: Sphere, input: RayCastInput) -> Option<ShapeCastOutput> {
    let raw_shape = shape.raw();
    let input = input.raw();
    ShapeCastOutput::from_raw(unsafe { sys::b3RayCastSphere(&raw_shape, &input) })
}

pub fn ray_cast_hollow_sphere(shape: Sphere, input: RayCastInput) -> Option<ShapeCastOutput> {
    let raw_shape = shape.raw();
    let input = input.raw();
    ShapeCastOutput::from_raw(unsafe { sys::b3RayCastHollowSphere(&raw_shape, &input) })
}

pub fn ray_cast_capsule(shape: Capsule, input: RayCastInput) -> Option<ShapeCastOutput> {
    let raw_shape = shape.raw();
    let input = input.raw();
    ShapeCastOutput::from_raw(unsafe { sys::b3RayCastCapsule(&raw_shape, &input) })
}

pub fn ray_cast_hull<'a>(
    hull: impl Into<HullRef<'a>>,
    input: RayCastInput,
) -> Option<ShapeCastOutput> {
    let input = input.raw();
    ShapeCastOutput::from_raw(unsafe { sys::b3RayCastHull(hull.into().raw(), &input) })
}

pub fn ray_cast_mesh(mesh: &Mesh, scale: Vec3, input: RayCastInput) -> Option<ShapeCastOutput> {
    let raw_mesh = raw_mesh(mesh, scale);
    let input = input.raw();
    ShapeCastOutput::from_raw(unsafe { sys::b3RayCastMesh(&raw_mesh, &input) })
}

pub fn ray_cast_height_field(
    height_field: &HeightField,
    input: RayCastInput,
) -> Option<ShapeCastOutput> {
    let input = input.raw();
    ShapeCastOutput::from_raw(unsafe { sys::b3RayCastHeightField(height_field.raw(), &input) })
}

pub fn ray_cast_compound(compound: &Compound, input: RayCastInput) -> Option<ShapeCastOutput> {
    let input = input.raw();
    ShapeCastOutput::from_raw(unsafe { sys::b3RayCastCompound(compound.raw(), &input) })
}

pub fn shape_cast_sphere(shape: Sphere, input: ShapeCastInput<'_>) -> Option<ShapeCastOutput> {
    let raw_shape = shape.raw();
    with_shape_cast_input(input, |input| unsafe {
        sys::b3ShapeCastSphere(&raw_shape, input)
    })
}

pub fn shape_cast_capsule(shape: Capsule, input: ShapeCastInput<'_>) -> Option<ShapeCastOutput> {
    let raw_shape = shape.raw();
    with_shape_cast_input(input, |input| unsafe {
        sys::b3ShapeCastCapsule(&raw_shape, input)
    })
}

pub fn shape_cast_hull<'a>(
    hull: impl Into<HullRef<'a>>,
    input: ShapeCastInput<'_>,
) -> Option<ShapeCastOutput> {
    let raw_hull = hull.into().raw();
    with_shape_cast_input(input, |input| unsafe {
        sys::b3ShapeCastHull(raw_hull, input)
    })
}

pub fn shape_cast_mesh(
    mesh: &Mesh,
    scale: Vec3,
    input: ShapeCastInput<'_>,
) -> Option<ShapeCastOutput> {
    let raw_mesh = raw_mesh(mesh, scale);
    with_shape_cast_input(input, |input| unsafe {
        sys::b3ShapeCastMesh(&raw_mesh, input)
    })
}

pub fn shape_cast_height_field(
    height_field: &HeightField,
    input: ShapeCastInput<'_>,
) -> Option<ShapeCastOutput> {
    with_shape_cast_input(input, |input| unsafe {
        sys::b3ShapeCastHeightField(height_field.raw(), input)
    })
}

pub fn shape_cast_compound(
    compound: &Compound,
    input: ShapeCastInput<'_>,
) -> Option<ShapeCastOutput> {
    with_shape_cast_input(input, |input| unsafe {
        sys::b3ShapeCastCompound(compound.raw(), input)
    })
}

pub fn get_sweep_transform(sweep: Sweep, time: f32) -> Transform {
    assert!(time.is_finite());
    let sweep = sweep.raw();
    unsafe { sys::b3GetSweepTransform(&sweep, time) }.into()
}

pub fn time_of_impact(input: TimeOfImpactInput<'_>) -> TimeOfImpactOutput {
    let points_a = input.proxy_a.raw_points();
    let points_b = input.proxy_b.raw_points();
    let input = sys::b3TOIInput {
        proxyA: input.proxy_a.raw(&points_a),
        proxyB: input.proxy_b.raw(&points_b),
        sweepA: input.sweep_a.raw(),
        sweepB: input.sweep_b.raw(),
        maxFraction: input.max_fraction,
    };
    unsafe { sys::b3TimeOfImpact(&input) }.into()
}

pub fn collide_spheres(
    sphere_a: Sphere,
    sphere_b: Sphere,
    transform_b_to_a: Transform,
) -> LocalManifold {
    let sphere_a = sphere_a.raw();
    let sphere_b = sphere_b.raw();
    with_local_manifold(|manifold, capacity| unsafe {
        sys::b3CollideSpheres(
            manifold,
            capacity,
            &sphere_a,
            &sphere_b,
            transform_b_to_a.into(),
        )
    })
}

pub fn collide_capsule_and_sphere(
    capsule_a: Capsule,
    sphere_b: Sphere,
    transform_b_to_a: Transform,
) -> LocalManifold {
    let capsule_a = capsule_a.raw();
    let sphere_b = sphere_b.raw();
    with_local_manifold(|manifold, capacity| unsafe {
        sys::b3CollideCapsuleAndSphere(
            manifold,
            capacity,
            &capsule_a,
            &sphere_b,
            transform_b_to_a.into(),
        )
    })
}

pub fn collide_hull_and_sphere<'a>(
    hull_a: impl Into<HullRef<'a>>,
    sphere_b: Sphere,
    transform_b_to_a: Transform,
) -> LocalManifold {
    let hull_a = hull_a.into().raw();
    let sphere_b = sphere_b.raw();
    let mut cache = sys::b3SimplexCache::default();
    with_local_manifold(|manifold, capacity| unsafe {
        sys::b3CollideHullAndSphere(
            manifold,
            capacity,
            hull_a,
            &sphere_b,
            transform_b_to_a.into(),
            &mut cache,
        )
    })
}

pub fn collide_capsules(
    capsule_a: Capsule,
    capsule_b: Capsule,
    transform_b_to_a: Transform,
) -> LocalManifold {
    let capsule_a = capsule_a.raw();
    let capsule_b = capsule_b.raw();
    with_local_manifold(|manifold, capacity| unsafe {
        sys::b3CollideCapsules(
            manifold,
            capacity,
            &capsule_a,
            &capsule_b,
            transform_b_to_a.into(),
        )
    })
}

pub fn collide_hull_and_capsule<'a>(
    hull_a: impl Into<HullRef<'a>>,
    capsule_b: Capsule,
    transform_b_to_a: Transform,
) -> LocalManifold {
    let hull_a = hull_a.into().raw();
    let capsule_b = capsule_b.raw();
    let mut cache = sys::b3SimplexCache::default();
    with_local_manifold(|manifold, capacity| unsafe {
        sys::b3CollideHullAndCapsule(
            manifold,
            capacity,
            hull_a,
            &capsule_b,
            transform_b_to_a.into(),
            &mut cache,
        )
    })
}

pub fn collide_hulls<'a, 'b>(
    hull_a: impl Into<HullRef<'a>>,
    hull_b: impl Into<HullRef<'b>>,
    transform_b_to_a: Transform,
) -> LocalManifold {
    let hull_a = hull_a.into().raw();
    let hull_b = hull_b.into().raw();
    let mut cache = sys::b3SATCache::default();
    with_local_manifold(|manifold, capacity| unsafe {
        sys::b3CollideHulls(
            manifold,
            capacity,
            hull_a,
            hull_b,
            transform_b_to_a.into(),
            &mut cache,
        )
    })
}

pub fn collide_capsule_and_triangle(capsule_a: Capsule, triangle_b: [Vec3; 3]) -> LocalManifold {
    let capsule_a = capsule_a.raw();
    let triangle_b = raw_triangle(triangle_b);
    let mut cache = sys::b3SimplexCache::default();
    with_local_manifold(|manifold, capacity| unsafe {
        sys::b3CollideCapsuleAndTriangle(
            manifold,
            capacity,
            &capsule_a,
            triangle_b.as_ptr(),
            &mut cache,
        )
    })
}

pub fn collide_hull_and_triangle<'a>(
    hull_a: impl Into<HullRef<'a>>,
    triangle_b: [Vec3; 3],
    triangle_flags: MeshEdgeFlags,
) -> LocalManifold {
    let hull_a = hull_a.into().raw();
    let triangle_b = raw_triangle(triangle_b);
    let mut cache = sys::b3SATCache::default();
    with_local_manifold(|manifold, capacity| unsafe {
        sys::b3CollideHullAndTriangle(
            manifold,
            capacity,
            hull_a,
            triangle_b[0],
            triangle_b[1],
            triangle_b[2],
            triangle_flags.bits(),
            &mut cache,
        )
    })
}

pub fn collide_sphere_and_triangle(sphere_a: Sphere, triangle_b: [Vec3; 3]) -> LocalManifold {
    let sphere_a = sphere_a.raw();
    let triangle_b = raw_triangle(triangle_b);
    with_local_manifold(|manifold, capacity| unsafe {
        sys::b3CollideSphereAndTriangle(manifold, capacity, &sphere_a, triangle_b.as_ptr())
    })
}

fn with_shape_proxy<T>(proxy: ShapeProxy<'_>, f: impl FnOnce(&sys::b3ShapeProxy) -> T) -> T {
    let points = proxy.raw_points();
    let proxy = proxy.raw(&points);
    f(&proxy)
}

fn with_shape_cast_input(
    input: ShapeCastInput<'_>,
    f: impl FnOnce(&sys::b3ShapeCastInput) -> sys::b3CastOutput,
) -> Option<ShapeCastOutput> {
    let points = input.proxy.raw_points();
    let input = input.raw(&points);
    ShapeCastOutput::from_raw(f(&input))
}

fn raw_mesh(mesh: &Mesh, scale: Vec3) -> sys::b3Mesh {
    assert_mesh_scale(scale);
    sys::b3Mesh {
        data: mesh.raw(),
        scale: scale.into(),
    }
}

fn assert_mesh_scale(scale: Vec3) {
    assert_valid_vec3(scale);
    assert!(scale.x != 0.0 && scale.y != 0.0 && scale.z != 0.0);
}

fn assert_valid_vec3(value: Vec3) {
    assert!(value.x.is_finite() && value.y.is_finite() && value.z.is_finite());
}

fn with_local_manifold(f: impl FnOnce(*mut sys::b3LocalManifold, i32)) -> LocalManifold {
    let mut points = [sys::b3LocalManifoldPoint::default(); LOCAL_MANIFOLD_CAPACITY];
    let mut manifold = sys::b3LocalManifold {
        points: points.as_mut_ptr(),
        ..sys::b3LocalManifold::default()
    };

    f(&mut manifold, LOCAL_MANIFOLD_CAPACITY as i32);
    LocalManifold::from_raw(manifold, &points)
}

fn raw_triangle(triangle: [Vec3; 3]) -> [sys::b3Vec3; 3] {
    [triangle[0].into(), triangle[1].into(), triangle[2].into()]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        hull::BoxHull,
        math::{Quat, Transform},
    };

    fn assert_close(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < 1.0e-5, "{actual} != {expected}");
    }

    #[test]
    fn sphere_aabb_and_mass_are_owned_values() {
        let sphere = Sphere::new(Vec3::new(1.0, 2.0, 3.0), 2.0);
        let aabb = compute_aabb(sphere, Transform::IDENTITY);

        assert_eq!(aabb.lower_bound, Vec3::new(-1.0, 0.0, 1.0));
        assert_eq!(aabb.upper_bound, Vec3::new(3.0, 4.0, 5.0));

        let mass = sphere.compute_mass(1.0);
        let copy = mass;
        let expected_mass = 4.0 / 3.0 * std::f32::consts::PI * 8.0;

        assert_close(copy.mass, expected_mass);
        assert_eq!(copy.center, sphere.center);
        assert_close(copy.inertia.cx.x, 0.4 * expected_mass * 4.0);
    }

    #[test]
    fn capsule_aabb_and_mass_are_owned_values() {
        let capsule = Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.25);
        let transform = Transform::new(Vec3::new(2.0, 0.0, -1.0), Quat::IDENTITY);
        let aabb = capsule.compute_aabb(transform);

        assert_eq!(aabb.lower_bound, Vec3::new(1.75, -1.25, -1.25));
        assert_eq!(aabb.upper_bound, Vec3::new(2.25, 1.25, -0.75));

        let mass = compute_mass(capsule, 2.0);
        assert!(mass.mass > 0.0);
        assert_eq!(mass.center, Vec3::ZERO);
    }

    #[test]
    fn box_aabb_and_mass_use_hull_helpers() {
        let box_shape = BoxShape::new(Vec3::new(1.0, 2.0, 3.0));
        let aabb = box_shape.compute_aabb(Transform::IDENTITY);

        assert_eq!(aabb.lower_bound, Vec3::new(-1.0, -2.0, -3.0));
        assert_eq!(aabb.upper_bound, Vec3::new(1.0, 2.0, 3.0));

        let mass = box_shape.compute_mass(0.5);
        assert_close(mass.mass, 24.0);
        assert_eq!(mass.center, Vec3::ZERO);
    }

    #[test]
    fn proxy_distance_and_shape_cast_work() {
        let point_a = [Vec3::ZERO];
        let point_b = [Vec3::ZERO];
        let proxy_a = ShapeProxy::new(&point_a, 0.5).unwrap();
        let proxy_b = ShapeProxy::new(&point_b, 0.5).unwrap();
        let transform = Transform::new(Vec3::new(2.0, 0.0, 0.0), Quat::IDENTITY);

        let distance = shape_distance(proxy_a, proxy_b, transform, true);
        assert_close(distance.distance, 1.0);

        let cast = shape_cast(
            proxy_a,
            proxy_b,
            transform,
            Vec3::new(-3.0, 0.0, 0.0),
            1.0,
            false,
        )
        .expect("moving sphere should hit fixed sphere");
        assert!(cast.fraction > 0.0 && cast.fraction < 1.0, "{cast:?}");
    }

    #[test]
    fn standalone_sphere_capsule_and_hull_queries() {
        let point = [Vec3::ZERO];
        let proxy = ShapeProxy::new(&point, 0.25).unwrap();

        let sphere = Sphere::new(Vec3::ZERO, 1.0);
        assert!(is_valid_ray(RayCastInput::new(
            Vec3::new(-2.0, 0.0, 0.0),
            Vec3::new(4.0, 0.0, 0.0),
            1.0,
        )));
        assert!(overlap_sphere(sphere, Transform::IDENTITY, proxy));
        assert!(ray_cast_sphere(
            sphere,
            RayCastInput::new(Vec3::new(-2.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0), 1.0),
        )
        .is_some());
        assert!(ray_cast_hollow_sphere(
            sphere,
            RayCastInput::new(Vec3::new(-2.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0), 1.0),
        )
        .is_some());
        assert!(shape_cast_sphere(
            Sphere::new(Vec3::new(2.0, 0.0, 0.0), 0.5),
            ShapeCastInput::new(proxy, Vec3::new(3.0, 0.0, 0.0), 1.0, false),
        )
        .is_some());

        let capsule = Capsule::new(Vec3::new(-0.5, 0.0, 0.0), Vec3::new(0.5, 0.0, 0.0), 0.5);
        assert!(overlap_capsule(capsule, Transform::IDENTITY, proxy));
        assert!(ray_cast_capsule(
            capsule,
            RayCastInput::new(Vec3::new(-2.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0), 1.0),
        )
        .is_some());
        assert!(shape_cast_capsule(
            Capsule::new(Vec3::new(2.0, -0.5, 0.0), Vec3::new(2.0, 0.5, 0.0), 0.5),
            ShapeCastInput::new(proxy, Vec3::new(3.0, 0.0, 0.0), 1.0, false),
        )
        .is_some());

        let hull = BoxHull::cube(1.0);
        assert!(overlap_hull(&hull, Transform::IDENTITY, proxy));
        assert!(ray_cast_hull(
            &hull,
            RayCastInput::new(Vec3::new(-3.0, 0.0, 0.0), Vec3::new(6.0, 0.0, 0.0), 1.0),
        )
        .is_some());
        assert!(shape_cast_hull(
            &hull,
            ShapeCastInput::new(proxy, Vec3::new(3.0, 0.0, 0.0), 1.0, false),
        )
        .is_some());
    }

    #[test]
    fn local_collision_wrappers_return_copied_manifolds() {
        fn assert_manifold(manifold: &LocalManifold) {
            assert!(!manifold.points().is_empty(), "{manifold:?}");
            assert!(manifold.normal.x.is_finite());
            assert!(manifold.normal.y.is_finite());
            assert!(manifold.normal.z.is_finite());
            for point in manifold.points() {
                assert!(point.point.x.is_finite());
                assert!(point.point.y.is_finite());
                assert!(point.point.z.is_finite());
                assert!(point.separation.is_finite());
            }
        }

        let sphere_a = Sphere::new(Vec3::ZERO, 0.75);
        let sphere_b = Sphere::new(Vec3::ZERO, 0.75);
        assert_manifold(&collide_spheres(
            sphere_a,
            sphere_b,
            Transform::new(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY),
        ));

        let capsule = Capsule::new(Vec3::new(-0.5, 0.0, 0.0), Vec3::new(0.5, 0.0, 0.0), 0.5);
        assert_manifold(&collide_capsule_and_sphere(
            capsule,
            Sphere::new(Vec3::new(0.0, 0.7, 0.0), 0.5),
            Transform::IDENTITY,
        ));
        assert_manifold(&collide_capsules(
            capsule,
            capsule,
            Transform::new(Vec3::new(0.0, 0.7, 0.0), Quat::IDENTITY),
        ));

        let hull = BoxHull::cube(1.0);
        assert_manifold(&collide_hull_and_sphere(
            &hull,
            Sphere::new(Vec3::new(0.8, 0.0, 0.0), 0.5),
            Transform::IDENTITY,
        ));
        assert_manifold(&collide_hull_and_capsule(
            &hull,
            capsule,
            Transform::new(Vec3::new(0.8, 0.0, 0.0), Quat::IDENTITY),
        ));
        assert_manifold(&collide_hulls(
            &hull,
            &hull,
            Transform::new(Vec3::new(0.5, 0.0, 0.0), Quat::IDENTITY),
        ));

        let triangle = [
            Vec3::new(-2.0, -1.0, 0.0),
            Vec3::new(2.0, -1.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ];
        let triangle_capsule =
            Capsule::new(Vec3::new(0.0, 0.0, -0.5), Vec3::new(0.0, 0.0, 0.5), 0.25);
        assert_manifold(&collide_capsule_and_triangle(triangle_capsule, triangle));
        assert_manifold(&collide_hull_and_triangle(
            &hull,
            triangle,
            MeshEdgeFlags::default(),
        ));
        assert_manifold(&collide_sphere_and_triangle(
            Sphere::new(Vec3::ZERO, 0.5),
            triangle,
        ));
    }

    #[test]
    fn sweep_transform_and_time_of_impact_match_native_fixture() {
        let square = [
            Vec3::new(-1.0, -1.0, 0.0),
            Vec3::new(1.0, -1.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(-1.0, 1.0, 0.0),
        ];
        let segment = [Vec3::new(2.0, -1.0, 0.0), Vec3::new(2.0, 1.0, 0.0)];
        let proxy_a = ShapeProxy::new(&square, 0.0).unwrap();
        let proxy_b = ShapeProxy::new(&segment, 0.0).unwrap();
        let sweep_a = Sweep::new(
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::ZERO,
            Quat::IDENTITY,
            Quat::IDENTITY,
        );
        let sweep_b = Sweep::new(
            Vec3::ZERO,
            Vec3::ZERO,
            Vec3::new(-2.0, 0.0, 0.0),
            Quat::IDENTITY,
            Quat::IDENTITY,
        );

        let halfway = get_sweep_transform(sweep_b, 0.5);
        assert!((halfway.p.x + 1.0).abs() < 1.0e-5, "{halfway:?}");

        let output = time_of_impact(TimeOfImpactInput::new(
            proxy_a, proxy_b, sweep_a, sweep_b, 1.0,
        ));

        assert_eq!(output.state, TimeOfImpactState::Hit);
        assert!((output.fraction - 0.5).abs() < 0.005, "{output:?}");
    }
}
