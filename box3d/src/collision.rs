use box3d_sys as sys;

use crate::{
    compound::Compound,
    hull::HullRef,
    math::{Aabb, MassData, Transform, Vec3},
    mesh::{HeightField, Mesh},
    query::ShapeProxy,
};

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
}
