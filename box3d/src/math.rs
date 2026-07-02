use box3d_sys as sys;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

impl From<Vec3> for sys::b3Vec3 {
    fn from(value: Vec3) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<sys::b3Vec3> for Vec3 {
    fn from(value: sys::b3Vec3) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub v: Vec3,
    pub s: f32,
}

impl Quat {
    pub const IDENTITY: Self = Self::new(Vec3::ZERO, 1.0);

    pub const fn new(v: Vec3, s: f32) -> Self {
        Self { v, s }
    }
}

impl From<Quat> for sys::b3Quat {
    fn from(value: Quat) -> Self {
        Self {
            v: value.v.into(),
            s: value.s,
        }
    }
}

impl From<sys::b3Quat> for Quat {
    fn from(value: sys::b3Quat) -> Self {
        Self {
            v: value.v.into(),
            s: value.s,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub p: Vec3,
    pub q: Quat,
}

impl Transform {
    pub const IDENTITY: Self = Self::new(Vec3::ZERO, Quat::IDENTITY);

    pub const fn new(p: Vec3, q: Quat) -> Self {
        Self { p, q }
    }
}

impl From<Transform> for sys::b3Transform {
    fn from(value: Transform) -> Self {
        Self {
            p: value.p.into(),
            q: value.q.into(),
        }
    }
}

impl From<sys::b3Transform> for Transform {
    fn from(value: sys::b3Transform) -> Self {
        Self {
            p: value.p.into(),
            q: value.q.into(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Matrix3 {
    pub cx: Vec3,
    pub cy: Vec3,
    pub cz: Vec3,
}

impl Matrix3 {
    pub const IDENTITY: Self = Self {
        cx: Vec3::new(1.0, 0.0, 0.0),
        cy: Vec3::new(0.0, 1.0, 0.0),
        cz: Vec3::new(0.0, 0.0, 1.0),
    };
}

impl From<Matrix3> for sys::b3Matrix3 {
    fn from(value: Matrix3) -> Self {
        Self {
            cx: value.cx.into(),
            cy: value.cy.into(),
            cz: value.cz.into(),
        }
    }
}

impl From<sys::b3Matrix3> for Matrix3 {
    fn from(value: sys::b3Matrix3) -> Self {
        Self {
            cx: value.cx.into(),
            cy: value.cy.into(),
            cz: value.cz.into(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CosSin {
    pub cosine: f32,
    pub sine: f32,
}

impl From<sys::b3CosSin> for CosSin {
    fn from(value: sys::b3CosSin) -> Self {
        Self {
            cosine: value.cosine,
            sine: value.sine,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SegmentDistance {
    pub point1: Vec3,
    pub fraction1: f32,
    pub point2: Vec3,
    pub fraction2: f32,
}

impl From<sys::b3SegmentDistanceResult> for SegmentDistance {
    fn from(value: sys::b3SegmentDistanceResult) -> Self {
        Self {
            point1: value.point1.into(),
            fraction1: value.fraction1,
            point2: value.point2.into(),
            fraction2: value.fraction2,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Aabb {
    pub lower_bound: Vec3,
    pub upper_bound: Vec3,
}

impl From<Aabb> for sys::b3AABB {
    fn from(value: Aabb) -> Self {
        Self {
            lowerBound: value.lower_bound.into(),
            upperBound: value.upper_bound.into(),
        }
    }
}

impl From<sys::b3AABB> for Aabb {
    fn from(value: sys::b3AABB) -> Self {
        Self {
            lower_bound: value.lowerBound.into(),
            upper_bound: value.upperBound.into(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MassData {
    pub mass: f32,
    pub center: Vec3,
    pub inertia: Matrix3,
}

impl From<MassData> for sys::b3MassData {
    fn from(value: MassData) -> Self {
        Self {
            mass: value.mass,
            center: value.center.into(),
            inertia: value.inertia.into(),
        }
    }
}

impl From<sys::b3MassData> for MassData {
    fn from(value: sys::b3MassData) -> Self {
        Self {
            mass: value.mass,
            center: value.center.into(),
            inertia: value.inertia.into(),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Filter {
    pub category_bits: u64,
    pub mask_bits: u64,
    pub group_index: i32,
}

impl Default for Filter {
    fn default() -> Self {
        unsafe { sys::b3DefaultFilter() }.into()
    }
}

impl From<Filter> for sys::b3Filter {
    fn from(value: Filter) -> Self {
        Self {
            categoryBits: value.category_bits,
            maskBits: value.mask_bits,
            groupIndex: value.group_index,
        }
    }
}

impl From<sys::b3Filter> for Filter {
    fn from(value: sys::b3Filter) -> Self {
        Self {
            category_bits: value.categoryBits,
            mask_bits: value.maskBits,
            group_index: value.groupIndex,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceMaterial {
    pub friction: f32,
    pub restitution: f32,
    pub rolling_resistance: f32,
    pub tangent_velocity: Vec3,
    pub user_material_id: u64,
    pub custom_color: u32,
}

impl Default for SurfaceMaterial {
    fn default() -> Self {
        unsafe { sys::b3DefaultSurfaceMaterial() }.into()
    }
}

impl From<SurfaceMaterial> for sys::b3SurfaceMaterial {
    fn from(value: SurfaceMaterial) -> Self {
        Self {
            friction: value.friction,
            restitution: value.restitution,
            rollingResistance: value.rolling_resistance,
            tangentVelocity: value.tangent_velocity.into(),
            userMaterialId: value.user_material_id,
            customColor: value.custom_color,
        }
    }
}

impl From<sys::b3SurfaceMaterial> for SurfaceMaterial {
    fn from(value: sys::b3SurfaceMaterial) -> Self {
        Self {
            friction: value.friction,
            restitution: value.restitution,
            rolling_resistance: value.rollingResistance,
            tangent_velocity: value.tangentVelocity.into(),
            user_material_id: value.userMaterialId,
            custom_color: value.customColor,
        }
    }
}

pub fn deterministic_atan2(y: f32, x: f32) -> f32 {
    unsafe { sys::b3Atan2(y, x) }
}

pub fn compute_cos_sin(radians: f32) -> CosSin {
    unsafe { sys::b3ComputeCosSin(radians) }.into()
}

pub fn make_quat_from_matrix(matrix: Matrix3) -> Quat {
    let matrix = matrix.into();
    unsafe { sys::b3MakeQuatFromMatrix(&matrix) }.into()
}

pub fn compute_quat_between_unit_vectors(v1: Vec3, v2: Vec3) -> Quat {
    unsafe { sys::b3ComputeQuatBetweenUnitVectors(v1.into(), v2.into()) }.into()
}

pub fn steiner(mass: f32, origin: Vec3) -> Matrix3 {
    unsafe { sys::b3Steiner(mass, origin.into()) }.into()
}

pub fn point_to_segment_distance(a: Vec3, b: Vec3, q: Vec3) -> Vec3 {
    unsafe { sys::b3PointToSegmentDistance(a.into(), b.into(), q.into()) }.into()
}

pub fn line_distance(p1: Vec3, d1: Vec3, p2: Vec3, d2: Vec3) -> SegmentDistance {
    unsafe { sys::b3LineDistance(p1.into(), d1.into(), p2.into(), d2.into()) }.into()
}

pub fn segment_distance(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> SegmentDistance {
    unsafe { sys::b3SegmentDistance(p1.into(), q1.into(), p2.into(), q2.into()) }.into()
}

pub fn is_valid_float(value: f32) -> bool {
    unsafe { sys::b3IsValidFloat(value) }
}

pub fn is_valid_vec3(value: Vec3) -> bool {
    unsafe { sys::b3IsValidVec3(value.into()) }
}

pub fn is_valid_quat(value: Quat) -> bool {
    unsafe { sys::b3IsValidQuat(value.into()) }
}

pub fn is_valid_transform(value: Transform) -> bool {
    unsafe { sys::b3IsValidTransform(value.into()) }
}

pub fn is_valid_matrix3(value: Matrix3) -> bool {
    unsafe { sys::b3IsValidMatrix3(value.into()) }
}

pub fn is_valid_aabb(value: Aabb) -> bool {
    unsafe { sys::b3IsValidAABB(value.into()) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn value_types_round_trip_through_sys() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(Vec3::from(sys::b3Vec3::from(v)), v);

        let t = Transform::new(v, Quat::new(Vec3::new(0.1, 0.2, 0.3), 0.4));
        assert_eq!(Transform::from(sys::b3Transform::from(t)), t);

        let aabb = Aabb {
            lower_bound: Vec3::new(-1.0, -2.0, -3.0),
            upper_bound: Vec3::new(4.0, 5.0, 6.0),
        };
        assert_eq!(Aabb::from(sys::b3AABB::from(aabb)), aabb);

        let mass = MassData {
            mass: 7.0,
            center: v,
            inertia: Matrix3 {
                cx: Vec3::new(1.0, 0.0, 0.0),
                cy: Vec3::new(0.0, 1.0, 0.0),
                cz: Vec3::new(0.0, 0.0, 1.0),
            },
        };
        assert_eq!(MassData::from(sys::b3MassData::from(mass)), mass);

        let filter = Filter {
            category_bits: 1,
            mask_bits: 2,
            group_index: -3,
        };
        assert_eq!(Filter::from(sys::b3Filter::from(filter)), filter);

        let material = SurfaceMaterial {
            friction: 0.5,
            restitution: 0.25,
            rolling_resistance: 0.125,
            tangent_velocity: v,
            user_material_id: 8,
            custom_color: 0x00ff00,
        };
        assert_eq!(
            SurfaceMaterial::from(sys::b3SurfaceMaterial::from(material)),
            material
        );
    }

    #[test]
    fn math_helpers_return_copied_values() {
        let angle = deterministic_atan2(1.0, 0.0);
        assert!((angle - std::f32::consts::FRAC_PI_2).abs() < 1.0e-3);

        let cs = compute_cos_sin(0.0);
        assert!((cs.cosine - 1.0).abs() < 1.0e-6);
        assert!(cs.sine.abs() < 1.0e-6);

        let quat = make_quat_from_matrix(Matrix3::IDENTITY);
        assert!(is_valid_quat(quat));
        let between =
            compute_quat_between_unit_vectors(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        assert!(is_valid_quat(between));

        let inertia = steiner(2.0, Vec3::new(1.0, 0.0, 0.0));
        assert!(is_valid_matrix3(inertia));

        let closest = point_to_segment_distance(
            Vec3::ZERO,
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
        );
        assert_eq!(closest, Vec3::new(1.0, 0.0, 0.0));

        let line = line_distance(
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        );
        assert_eq!(line.point1, Vec3::ZERO);
        assert_eq!(line.point2, Vec3::new(0.0, 1.0, 0.0));

        let segment = segment_distance(
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
        );
        assert_eq!(segment.point1, Vec3::ZERO);
        assert_eq!(segment.point2, Vec3::new(0.0, 1.0, 0.0));

        assert!(is_valid_float(1.0));
        assert!(is_valid_vec3(Vec3::ZERO));
        assert!(is_valid_transform(Transform::IDENTITY));
        assert!(is_valid_aabb(Aabb {
            lower_bound: Vec3::ZERO,
            upper_bound: Vec3::new(1.0, 1.0, 1.0),
        }));
    }
}
