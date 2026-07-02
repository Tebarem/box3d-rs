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
}
