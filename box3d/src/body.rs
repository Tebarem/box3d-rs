use std::{
    ffi::{c_void, CStr, CString},
    marker::PhantomData,
    slice,
};

use box3d_sys as sys;

use crate::{
    events::{BodyId, ContactId, JointId, ShapeId, WorldId},
    handle,
    math::{MassData, Matrix3, Quat, Transform, Vec3},
    shape::{raw_shape_def, Shape, ShapeDef},
    world::World,
    Result,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BodyType {
    Static,
    Kinematic,
    Dynamic,
}

impl From<sys::b3BodyType> for BodyType {
    fn from(value: sys::b3BodyType) -> Self {
        match value {
            sys::b3BodyType_b3_staticBody => Self::Static,
            sys::b3BodyType_b3_kinematicBody => Self::Kinematic,
            sys::b3BodyType_b3_dynamicBody => Self::Dynamic,
            _ => panic!("unknown box3d body type {value}"),
        }
    }
}

impl From<BodyType> for sys::b3BodyType {
    fn from(value: BodyType) -> Self {
        match value {
            BodyType::Static => sys::b3BodyType_b3_staticBody,
            BodyType::Kinematic => sys::b3BodyType_b3_kinematicBody,
            BodyType::Dynamic => sys::b3BodyType_b3_dynamicBody,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BodyDef {
    pub body_type: BodyType,
    pub position: Vec3,
}

impl BodyDef {
    pub const fn static_at(position: Vec3) -> Self {
        Self {
            body_type: BodyType::Static,
            position,
        }
    }

    pub const fn dynamic_at(position: Vec3) -> Self {
        Self {
            body_type: BodyType::Dynamic,
            position,
        }
    }
}

impl Default for BodyDef {
    fn default() -> Self {
        Self::static_at(Vec3::ZERO)
    }
}

pub struct Body<'world> {
    raw: sys::b3BodyId,
    _world: PhantomData<&'world World>,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct MotionLocks {
    pub linear_x: bool,
    pub linear_y: bool,
    pub linear_z: bool,
    pub angular_x: bool,
    pub angular_y: bool,
    pub angular_z: bool,
}

impl From<sys::b3MotionLocks> for MotionLocks {
    fn from(value: sys::b3MotionLocks) -> Self {
        Self {
            linear_x: value.linearX,
            linear_y: value.linearY,
            linear_z: value.linearZ,
            angular_x: value.angularX,
            angular_y: value.angularY,
            angular_z: value.angularZ,
        }
    }
}

impl From<MotionLocks> for sys::b3MotionLocks {
    fn from(value: MotionLocks) -> Self {
        Self {
            linearX: value.linear_x,
            linearY: value.linear_y,
            linearZ: value.linear_z,
            angularX: value.angular_x,
            angularY: value.angular_y,
            angularZ: value.angular_z,
        }
    }
}

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

impl<'world> Body<'world> {
    pub(crate) fn from_raw(raw: sys::b3BodyId) -> Self {
        Self {
            raw,
            _world: PhantomData,
        }
    }

    pub(crate) fn raw(&self) -> sys::b3BodyId {
        self.raw
    }

    pub fn id(&self) -> BodyId {
        BodyId::from_raw(self.raw)
    }

    pub fn world_id(&self) -> WorldId {
        WorldId::from_raw(unsafe { sys::b3Body_GetWorld(self.raw) })
    }

    pub fn body_type(&self) -> BodyType {
        unsafe { sys::b3Body_GetType(self.raw) }.into()
    }

    pub fn set_body_type(&self, body_type: BodyType) {
        unsafe { sys::b3Body_SetType(self.raw, body_type.into()) };
    }

    pub fn set_name(&self, name: &str) -> Result<()> {
        if name.len() >= sys::B3_BODY_NAME_LENGTH as usize {
            return Err(crate::Error::InvalidInput);
        }

        let name = CString::new(name).map_err(|_| crate::Error::InvalidInput)?;
        unsafe { sys::b3Body_SetName(self.raw, name.as_ptr()) };
        Ok(())
    }

    pub fn name(&self) -> String {
        let name = unsafe { sys::b3Body_GetName(self.raw) };
        if name.is_null() {
            String::new()
        } else {
            unsafe { CStr::from_ptr(name) }
                .to_string_lossy()
                .into_owned()
        }
    }

    pub fn set_user_data(&self, user_data: usize) {
        unsafe { sys::b3Body_SetUserData(self.raw, user_data as *mut c_void) };
    }

    pub fn user_data(&self) -> usize {
        unsafe { sys::b3Body_GetUserData(self.raw) as usize }
    }

    pub fn position(&self) -> Vec3 {
        unsafe { sys::b3Body_GetPosition(self.raw) }.into()
    }

    pub fn rotation(&self) -> Quat {
        unsafe { sys::b3Body_GetRotation(self.raw) }.into()
    }

    pub fn transform(&self) -> Transform {
        unsafe { sys::b3Body_GetTransform(self.raw) }.into()
    }

    pub fn set_transform(&self, position: Vec3, rotation: Quat) {
        unsafe { sys::b3Body_SetTransform(self.raw, position.into(), rotation.into()) };
    }

    pub fn local_point(&self, world_point: Vec3) -> Vec3 {
        unsafe { sys::b3Body_GetLocalPoint(self.raw, world_point.into()) }.into()
    }

    pub fn world_point(&self, local_point: Vec3) -> Vec3 {
        unsafe { sys::b3Body_GetWorldPoint(self.raw, local_point.into()) }.into()
    }

    pub fn local_vector(&self, world_vector: Vec3) -> Vec3 {
        unsafe { sys::b3Body_GetLocalVector(self.raw, world_vector.into()) }.into()
    }

    pub fn world_vector(&self, local_vector: Vec3) -> Vec3 {
        unsafe { sys::b3Body_GetWorldVector(self.raw, local_vector.into()) }.into()
    }

    pub fn linear_velocity(&self) -> Vec3 {
        unsafe { sys::b3Body_GetLinearVelocity(self.raw) }.into()
    }

    pub fn set_linear_velocity(&self, velocity: Vec3) {
        unsafe { sys::b3Body_SetLinearVelocity(self.raw, velocity.into()) };
    }

    pub fn angular_velocity(&self) -> Vec3 {
        unsafe { sys::b3Body_GetAngularVelocity(self.raw) }.into()
    }

    pub fn set_angular_velocity(&self, velocity: Vec3) {
        unsafe { sys::b3Body_SetAngularVelocity(self.raw, velocity.into()) };
    }

    pub fn set_target_transform(&self, target: Transform, time_step: f32, wake: bool) {
        assert!(time_step.is_finite() && time_step > 0.0);
        unsafe { sys::b3Body_SetTargetTransform(self.raw, target.into(), time_step, wake) };
    }

    pub fn local_point_velocity(&self, local_point: Vec3) -> Vec3 {
        unsafe { sys::b3Body_GetLocalPointVelocity(self.raw, local_point.into()) }.into()
    }

    pub fn world_point_velocity(&self, world_point: Vec3) -> Vec3 {
        unsafe { sys::b3Body_GetWorldPointVelocity(self.raw, world_point.into()) }.into()
    }

    pub fn apply_force(&self, force: Vec3, point: Vec3, wake: bool) {
        unsafe { sys::b3Body_ApplyForce(self.raw, force.into(), point.into(), wake) };
    }

    pub fn apply_force_to_center(&self, force: Vec3, wake: bool) {
        unsafe { sys::b3Body_ApplyForceToCenter(self.raw, force.into(), wake) };
    }

    pub fn apply_torque(&self, torque: Vec3, wake: bool) {
        unsafe { sys::b3Body_ApplyTorque(self.raw, torque.into(), wake) };
    }

    pub fn apply_linear_impulse(&self, impulse: Vec3, point: Vec3, wake: bool) {
        unsafe { sys::b3Body_ApplyLinearImpulse(self.raw, impulse.into(), point.into(), wake) };
    }

    pub fn apply_linear_impulse_to_center(&self, impulse: Vec3, wake: bool) {
        unsafe { sys::b3Body_ApplyLinearImpulseToCenter(self.raw, impulse.into(), wake) };
    }

    pub fn apply_angular_impulse(&self, impulse: Vec3, wake: bool) {
        unsafe { sys::b3Body_ApplyAngularImpulse(self.raw, impulse.into(), wake) };
    }

    pub fn mass(&self) -> f32 {
        unsafe { sys::b3Body_GetMass(self.raw) }
    }

    pub fn local_rotational_inertia(&self) -> Matrix3 {
        unsafe { sys::b3Body_GetLocalRotationalInertia(self.raw) }.into()
    }

    pub fn inverse_mass(&self) -> f32 {
        unsafe { sys::b3Body_GetInverseMass(self.raw) }
    }

    pub fn world_inverse_rotational_inertia(&self) -> Matrix3 {
        unsafe { sys::b3Body_GetWorldInverseRotationalInertia(self.raw) }.into()
    }

    pub fn local_center_of_mass(&self) -> Vec3 {
        unsafe { sys::b3Body_GetLocalCenterOfMass(self.raw) }.into()
    }

    pub fn world_center_of_mass(&self) -> Vec3 {
        unsafe { sys::b3Body_GetWorldCenterOfMass(self.raw) }.into()
    }

    pub fn set_mass_data(&self, mass_data: MassData) {
        unsafe { sys::b3Body_SetMassData(self.raw, mass_data.into()) };
    }

    pub fn mass_data(&self) -> MassData {
        unsafe { sys::b3Body_GetMassData(self.raw) }.into()
    }

    pub fn apply_mass_from_shapes(&self) {
        unsafe { sys::b3Body_ApplyMassFromShapes(self.raw) };
    }

    pub fn linear_damping(&self) -> f32 {
        unsafe { sys::b3Body_GetLinearDamping(self.raw) }
    }

    pub fn set_linear_damping(&self, damping: f32) {
        assert!(damping.is_finite() && damping >= 0.0);
        unsafe { sys::b3Body_SetLinearDamping(self.raw, damping) };
    }

    pub fn angular_damping(&self) -> f32 {
        unsafe { sys::b3Body_GetAngularDamping(self.raw) }
    }

    pub fn set_angular_damping(&self, damping: f32) {
        assert!(damping.is_finite() && damping >= 0.0);
        unsafe { sys::b3Body_SetAngularDamping(self.raw, damping) };
    }

    pub fn wake(&self) {
        self.set_awake(true);
    }

    pub fn set_awake(&self, awake: bool) {
        unsafe { sys::b3Body_SetAwake(self.raw, awake) };
    }

    pub fn is_awake(&self) -> bool {
        unsafe { sys::b3Body_IsAwake(self.raw) }
    }

    pub fn set_sleep_enabled(&self, enabled: bool) {
        unsafe { sys::b3Body_EnableSleep(self.raw, enabled) };
    }

    pub fn is_sleep_enabled(&self) -> bool {
        unsafe { sys::b3Body_IsSleepEnabled(self.raw) }
    }

    pub fn set_sleep_threshold(&self, threshold: f32) {
        unsafe { sys::b3Body_SetSleepThreshold(self.raw, threshold) };
    }

    pub fn sleep_threshold(&self) -> f32 {
        unsafe { sys::b3Body_GetSleepThreshold(self.raw) }
    }

    pub fn set_enabled(&self, enabled: bool) {
        if enabled {
            unsafe { sys::b3Body_Enable(self.raw) };
        } else {
            unsafe { sys::b3Body_Disable(self.raw) };
        }
    }

    pub fn is_enabled(&self) -> bool {
        unsafe { sys::b3Body_IsEnabled(self.raw) }
    }

    pub fn motion_locks(&self) -> MotionLocks {
        unsafe { sys::b3Body_GetMotionLocks(self.raw) }.into()
    }

    pub fn set_motion_locks(&self, locks: MotionLocks) {
        unsafe { sys::b3Body_SetMotionLocks(self.raw, locks.into()) };
    }

    pub fn set_bullet(&self, bullet: bool) {
        unsafe { sys::b3Body_SetBullet(self.raw, bullet) };
    }

    pub fn is_bullet(&self) -> bool {
        unsafe { sys::b3Body_IsBullet(self.raw) }
    }

    pub fn set_gravity_scale(&self, scale: f32) {
        unsafe { sys::b3Body_SetGravityScale(self.raw, scale) };
    }

    pub fn gravity_scale(&self) -> f32 {
        unsafe { sys::b3Body_GetGravityScale(self.raw) }
    }

    pub fn set_contact_recycling_enabled(&self, enabled: bool) {
        unsafe { sys::b3Body_EnableContactRecycling(self.raw, enabled) };
    }

    pub fn is_contact_recycling_enabled(&self) -> bool {
        unsafe { sys::b3Body_IsContactRecyclingEnabled(self.raw) }
    }

    pub fn enable_hit_events(&self, enabled: bool) {
        unsafe { sys::b3Body_EnableHitEvents(self.raw, enabled) };
    }

    pub fn shape_count(&self) -> i32 {
        unsafe { sys::b3Body_GetShapeCount(self.raw) }
    }

    pub fn shape_ids(&self) -> Vec<ShapeId> {
        let capacity = self.shape_count();
        if capacity <= 0 {
            return Vec::new();
        }

        let mut raw = vec![sys::b3ShapeId::default(); capacity as usize];
        let count = unsafe { sys::b3Body_GetShapes(self.raw, raw.as_mut_ptr(), capacity) };
        raw.truncate(count.max(0) as usize);
        raw.into_iter().map(ShapeId::from_raw).collect()
    }

    pub fn joint_count(&self) -> i32 {
        unsafe { sys::b3Body_GetJointCount(self.raw) }
    }

    pub fn joint_ids(&self) -> Vec<JointId> {
        let capacity = self.joint_count();
        if capacity <= 0 {
            return Vec::new();
        }

        let mut raw = vec![sys::b3JointId::default(); capacity as usize];
        let count = unsafe { sys::b3Body_GetJoints(self.raw, raw.as_mut_ptr(), capacity) };
        raw.truncate(count.max(0) as usize);
        raw.into_iter().map(JointId::from_raw).collect()
    }

    pub fn contact_capacity(&self) -> i32 {
        unsafe { sys::b3Body_GetContactCapacity(self.raw) }
    }

    pub fn contact_data(&self) -> Vec<ContactData> {
        let capacity = self.contact_capacity();
        if capacity <= 0 {
            return Vec::new();
        }

        let mut raw = vec![sys::b3ContactData::default(); capacity as usize];
        let count = unsafe { sys::b3Body_GetContactData(self.raw, raw.as_mut_ptr(), capacity) };
        raw.truncate(count.max(0) as usize);
        raw.into_iter().map(ContactData::from).collect()
    }

    pub fn create_box(&self, half_extents: Vec3, def: ShapeDef) -> Shape<'_> {
        self.try_create_box(half_extents, def)
            .expect("box3d returned an invalid shape")
    }

    pub fn try_create_box(&self, half_extents: Vec3, def: ShapeDef) -> Result<Shape<'_>> {
        let raw_def = raw_shape_def(def);

        let hull = unsafe { sys::b3MakeBoxHull(half_extents.x, half_extents.y, half_extents.z) };
        let raw = handle::shape(unsafe { sys::b3CreateHullShape(self.raw, &raw_def, &hull.base) })?;

        Ok(Shape::from_raw(raw))
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

impl Drop for Body<'_> {
    fn drop(&mut self) {
        handle::destroy_body(self.raw);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_velocity_moves_body() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let _shape = body.create_box(
            Vec3::new(0.5, 0.5, 0.5),
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                ..ShapeDef::default()
            },
        );

        body.set_linear_velocity(Vec3::new(2.0, 0.0, 0.0));
        body.set_angular_velocity(Vec3::new(0.0, 1.0, 0.0));
        world.step(1.0 / 60.0, 4);

        assert!(body.position().x > 0.0, "{:?}", body.position());
        assert_eq!(body.linear_velocity().x, 2.0);
        assert_eq!(body.angular_velocity().y, 1.0);
    }

    #[test]
    fn transform_accessors_round_trip() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let position = Vec3::new(1.0, 2.0, 3.0);

        body.set_transform(position, Quat::IDENTITY);

        assert_eq!(body.position(), position);
        assert_eq!(body.rotation(), Quat::IDENTITY);
        assert_eq!(body.transform(), Transform::new(position, Quat::IDENTITY));
    }

    #[test]
    fn impulse_changes_velocity() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let _shape = body.create_box(
            Vec3::new(0.5, 0.5, 0.5),
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                ..ShapeDef::default()
            },
        );

        body.apply_linear_impulse_to_center(Vec3::new(1.0, 0.0, 0.0), true);
        world.step(1.0 / 60.0, 4);

        assert!(
            body.linear_velocity().x > 0.0,
            "{:?}",
            body.linear_velocity()
        );
        assert!(body.position().x > 0.0, "{:?}", body.position());
    }

    #[test]
    fn runtime_controls_round_trip() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let _shape = body.create_box(
            Vec3::new(0.5, 0.5, 0.5),
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                ..ShapeDef::default()
            },
        );

        assert!(body.mass() > 0.0);
        assert!(body.inverse_mass() > 0.0);
        assert!(body.id().is_valid());
        assert_eq!(body.world_id(), world.id());
        assert_eq!(body.body_type(), BodyType::Dynamic);
        body.set_body_type(BodyType::Kinematic);
        assert_eq!(body.body_type(), BodyType::Kinematic);
        body.set_body_type(BodyType::Dynamic);
        assert_eq!(body.body_type(), BodyType::Dynamic);
        body.set_name("body").unwrap();
        assert_eq!(body.name(), "body");
        assert!(body.set_name("bad\0name").is_err());
        body.set_user_data(0x1234);
        assert_eq!(body.user_data(), 0x1234);
        assert_eq!(body.local_center_of_mass(), Vec3::ZERO);
        assert_eq!(body.world_center_of_mass(), Vec3::ZERO);
        assert_eq!(
            body.local_point(Vec3::new(1.0, 2.0, 3.0)),
            Vec3::new(1.0, 2.0, 3.0)
        );
        assert_eq!(
            body.world_point(Vec3::new(1.0, 2.0, 3.0)),
            Vec3::new(1.0, 2.0, 3.0)
        );
        assert_eq!(
            body.local_vector(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(1.0, 0.0, 0.0)
        );
        assert_eq!(
            body.world_vector(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(1.0, 0.0, 0.0)
        );
        let mass_data = body.mass_data();
        body.set_mass_data(mass_data);
        body.apply_mass_from_shapes();
        let _ = body.local_rotational_inertia();
        let _ = body.world_inverse_rotational_inertia();
        body.set_linear_damping(0.2);
        assert_eq!(body.linear_damping(), 0.2);
        body.set_angular_damping(0.3);
        assert_eq!(body.angular_damping(), 0.3);
        body.set_target_transform(
            Transform::new(Vec3::new(1.0, 0.0, 0.0), Quat::IDENTITY),
            1.0,
            true,
        );
        assert!(body.linear_velocity().x > 0.0);
        assert!(body.local_point_velocity(Vec3::ZERO).x > 0.0);
        assert!(body.world_point_velocity(Vec3::ZERO).x > 0.0);

        body.set_sleep_enabled(false);
        assert!(!body.is_sleep_enabled());
        body.set_sleep_enabled(true);
        assert!(body.is_sleep_enabled());
        body.set_awake(false);
        assert!(!body.is_awake());
        body.wake();
        assert!(body.is_awake());
        body.set_sleep_threshold(0.25);
        assert_eq!(body.sleep_threshold(), 0.25);

        body.set_enabled(false);
        assert!(!body.is_enabled());
        body.set_enabled(true);
        assert!(body.is_enabled());
        body.set_bullet(true);
        assert!(body.is_bullet());
        body.set_gravity_scale(0.5);
        assert_eq!(body.gravity_scale(), 0.5);
        body.set_motion_locks(MotionLocks {
            linear_x: true,
            angular_z: true,
            ..MotionLocks::default()
        });
        assert_eq!(
            body.motion_locks(),
            MotionLocks {
                linear_x: true,
                angular_z: true,
                ..MotionLocks::default()
            }
        );
        body.set_contact_recycling_enabled(false);
        assert!(!body.is_contact_recycling_enabled());
        body.set_contact_recycling_enabled(true);
        assert!(body.is_contact_recycling_enabled());
        body.enable_hit_events(true);
        body.enable_hit_events(false);
        assert_eq!(body.shape_count(), 1);
        assert_eq!(body.shape_ids().len(), 1);
        assert_eq!(body.joint_count(), 0);
        assert!(body.joint_ids().is_empty());
        assert_eq!(body.contact_capacity(), 0);
        assert!(body.contact_data().is_empty());
    }

    #[test]
    fn contact_data_reports_touching_shapes() {
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

        let contact = contacts.first().expect("body should touch ground");
        assert!(contact.contact.is_valid());
        assert!(contact.shape_a.is_valid());
        assert!(contact.shape_b.is_valid());
        assert!(!contact.manifolds.is_empty());
    }

    #[test]
    fn joint_ids_report_attached_joint() {
        let world = World::new(Vec3::ZERO);
        let body_a = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let body_b = world.create_body(BodyDef::dynamic_at(Vec3::new(1.0, 0.0, 0.0)));
        let joint = world.create_distance_joint(crate::DistanceJointDef::new(&body_a, &body_b));

        assert!(joint.is_valid());
        assert_eq!(body_a.joint_count(), 1);
        assert_eq!(body_a.joint_ids().len(), 1);
        assert!(body_a.joint_ids()[0].is_valid());
    }
}
