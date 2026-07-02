use std::marker::PhantomData;

use box3d_sys as sys;

use crate::{
    handle,
    math::{MassData, Matrix3, Quat, Transform, Vec3},
    shape::{Shape, ShapeDef},
    world::World,
    Result,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BodyType {
    Static,
    Kinematic,
    Dynamic,
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

    pub fn create_box(&self, half_extents: Vec3, def: ShapeDef) -> Shape<'_> {
        self.try_create_box(half_extents, def)
            .expect("box3d returned an invalid shape")
    }

    pub fn try_create_box(&self, half_extents: Vec3, def: ShapeDef) -> Result<Shape<'_>> {
        let mut raw_def = unsafe { sys::b3DefaultShapeDef() };
        raw_def.density = def.density;
        raw_def.baseMaterial.friction = def.friction;

        let hull = unsafe { sys::b3MakeBoxHull(half_extents.x, half_extents.y, half_extents.z) };
        let raw = handle::shape(unsafe { sys::b3CreateHullShape(self.raw, &raw_def, &hull.base) })?;

        Ok(Shape::from_raw(raw))
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
            },
        );

        assert!(body.mass() > 0.0);
        assert!(body.inverse_mass() > 0.0);
        assert_eq!(body.local_center_of_mass(), Vec3::ZERO);
        assert_eq!(body.world_center_of_mass(), Vec3::ZERO);
        let mass_data = body.mass_data();
        body.set_mass_data(mass_data);
        body.apply_mass_from_shapes();
        let _ = body.local_rotational_inertia();
        let _ = body.world_inverse_rotational_inertia();

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
    }
}
