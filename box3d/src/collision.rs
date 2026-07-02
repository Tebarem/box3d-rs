use box3d_sys as sys;

use crate::{
    math::{Aabb, MassData, Transform, Vec3},
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

    fn raw(self) -> sys::b3Sphere {
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

    fn raw(self) -> sys::b3Capsule {
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
    fn from_raw(value: sys::b3CastOutput) -> Option<Self> {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{Quat, Transform};

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
}
