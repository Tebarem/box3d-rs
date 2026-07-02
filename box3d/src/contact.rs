use std::slice;

use box3d_sys as sys;

use crate::{
    events::{ContactId, ShapeId},
    math::Vec3,
};

#[derive(Clone, Debug, PartialEq)]
pub struct ContactData {
    pub contact: ContactId,
    pub shape_a: ShapeId,
    pub shape_b: ShapeId,
    pub manifolds: Vec<ContactManifold>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContactManifold {
    pub points: Vec<ContactPoint>,
    pub normal: Vec3,
    pub twist_impulse: f32,
    pub friction_impulse: Vec3,
    pub rolling_impulse: Vec3,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactPoint {
    pub anchor_a: Vec3,
    pub anchor_b: Vec3,
    pub separation: f32,
    pub base_separation: f32,
    pub normal_impulse: f32,
    pub total_normal_impulse: f32,
    pub normal_velocity: f32,
    pub feature_id: u32,
    pub triangle_index: i32,
    pub persisted: bool,
}

impl ContactId {
    pub fn data(self) -> Option<ContactData> {
        self.is_valid()
            .then(|| unsafe { sys::b3Contact_GetData(self.raw()) }.into())
    }
}

impl From<sys::b3ContactData> for ContactData {
    fn from(value: sys::b3ContactData) -> Self {
        let manifolds = if value.manifoldCount <= 0 || value.manifolds.is_null() {
            Vec::new()
        } else {
            unsafe { slice::from_raw_parts(value.manifolds, value.manifoldCount as usize) }
                .iter()
                .copied()
                .map(ContactManifold::from)
                .collect()
        };

        Self {
            contact: ContactId::from_raw(value.contactId),
            shape_a: ShapeId::from_raw(value.shapeIdA),
            shape_b: ShapeId::from_raw(value.shapeIdB),
            manifolds,
        }
    }
}

impl From<sys::b3Manifold> for ContactManifold {
    fn from(value: sys::b3Manifold) -> Self {
        let point_count = value.pointCount.clamp(0, value.points.len() as i32) as usize;
        Self {
            points: value.points[..point_count]
                .iter()
                .copied()
                .map(ContactPoint::from)
                .collect(),
            normal: value.normal.into(),
            twist_impulse: value.twistImpulse,
            friction_impulse: value.frictionImpulse.into(),
            rolling_impulse: value.rollingImpulse.into(),
        }
    }
}

impl From<sys::b3ManifoldPoint> for ContactPoint {
    fn from(value: sys::b3ManifoldPoint) -> Self {
        Self {
            anchor_a: value.anchorA.into(),
            anchor_b: value.anchorB.into(),
            separation: value.separation,
            base_separation: value.baseSeparation,
            normal_impulse: value.normalImpulse,
            total_normal_impulse: value.totalNormalImpulse,
            normal_velocity: value.normalVelocity,
            feature_id: value.featureId,
            triangle_index: value.triangleIndex,
            persisted: value.persisted,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{BodyDef, ShapeDef, Vec3, World};

    #[test]
    fn contact_id_data_copies_manifolds() {
        let world = World::default();
        let ground = world.create_body(BodyDef::static_at(Vec3::new(0.0, -0.5, 0.0)));
        let _ground_shape = ground.create_box(Vec3::new(10.0, 0.5, 10.0), ShapeDef::default());
        let body = world.create_body(BodyDef::dynamic_at(Vec3::new(0.0, 4.0, 0.0)));
        let _shape = body.create_sphere(
            Vec3::ZERO,
            0.5,
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                ..ShapeDef::default()
            },
        );

        let mut contacts = Vec::new();
        for _ in 0..120 {
            world.step(1.0 / 60.0, 4);
            contacts = body.contact_data();
            if !contacts.is_empty() {
                break;
            }
        }

        let contact = contacts.first().expect("body should touch ground").contact;
        let data = contact.data().expect("contact should still be valid");

        assert_eq!(data.contact, contact);
        assert!(data.shape_a.is_valid());
        assert!(data.shape_b.is_valid());
        assert!(!data.manifolds.is_empty());
        assert!(data
            .manifolds
            .iter()
            .any(|manifold| !manifold.points.is_empty()));
    }
}
