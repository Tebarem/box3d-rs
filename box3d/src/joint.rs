use std::{marker::PhantomData, ops::Deref};

use box3d_sys as sys;

use crate::{
    body::Body,
    handle,
    math::{Quat, Transform, Vec3},
    world::World,
    Error, Result,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum JointType {
    Parallel,
    Distance,
    Filter,
    Motor,
    Prismatic,
    Revolute,
    Spherical,
    Weld,
    Wheel,
}

impl From<sys::b3JointType> for JointType {
    fn from(value: sys::b3JointType) -> Self {
        match value {
            sys::b3JointType_b3_parallelJoint => Self::Parallel,
            sys::b3JointType_b3_distanceJoint => Self::Distance,
            sys::b3JointType_b3_filterJoint => Self::Filter,
            sys::b3JointType_b3_motorJoint => Self::Motor,
            sys::b3JointType_b3_prismaticJoint => Self::Prismatic,
            sys::b3JointType_b3_revoluteJoint => Self::Revolute,
            sys::b3JointType_b3_sphericalJoint => Self::Spherical,
            sys::b3JointType_b3_weldJoint => Self::Weld,
            sys::b3JointType_b3_wheelJoint => Self::Wheel,
            _ => unreachable!("unknown Box3D joint type"),
        }
    }
}

#[derive(Clone, Copy)]
pub struct JointDef<'body, 'world> {
    pub body_a: &'body Body<'world>,
    pub body_b: &'body Body<'world>,
    pub local_frame_a: Transform,
    pub local_frame_b: Transform,
    pub force_threshold: f32,
    pub torque_threshold: f32,
    pub constraint_hertz: f32,
    pub constraint_damping_ratio: f32,
    pub draw_scale: f32,
    pub collide_connected: bool,
}

impl<'body, 'world> JointDef<'body, 'world> {
    fn from_raw(
        body_a: &'body Body<'world>,
        body_b: &'body Body<'world>,
        raw: sys::b3JointDef,
    ) -> Self {
        Self {
            body_a,
            body_b,
            local_frame_a: raw.localFrameA.into(),
            local_frame_b: raw.localFrameB.into(),
            force_threshold: raw.forceThreshold,
            torque_threshold: raw.torqueThreshold,
            constraint_hertz: raw.constraintHertz,
            constraint_damping_ratio: raw.constraintDampingRatio,
            draw_scale: raw.drawScale,
            collide_connected: raw.collideConnected,
        }
    }

    fn apply_to(&self, world: sys::b3WorldId, raw: &mut sys::b3JointDef) -> Result<()> {
        let body_a = self.body_a.raw();
        let body_b = self.body_b.raw();

        if !handle::is_body_valid(body_a) || !handle::is_body_valid(body_b) {
            return Err(Error::InvalidBody);
        }

        if !body_in_world(world, body_a) || !body_in_world(world, body_b) {
            return Err(Error::InvalidInput);
        }

        raw.bodyIdA = body_a;
        raw.bodyIdB = body_b;
        raw.localFrameA = self.local_frame_a.into();
        raw.localFrameB = self.local_frame_b.into();
        raw.forceThreshold = self.force_threshold;
        raw.torqueThreshold = self.torque_threshold;
        raw.constraintHertz = self.constraint_hertz;
        raw.constraintDampingRatio = self.constraint_damping_ratio;
        raw.drawScale = self.draw_scale;
        raw.collideConnected = self.collide_connected;

        Ok(())
    }
}

fn body_in_world(world: sys::b3WorldId, body: sys::b3BodyId) -> bool {
    u32::from(body.world0) + 1 == u32::from(world.index1)
}

#[derive(Clone, Copy)]
pub struct DistanceJointDef<'body, 'world> {
    pub base: JointDef<'body, 'world>,
    pub length: f32,
    pub enable_spring: bool,
    pub lower_spring_force: f32,
    pub upper_spring_force: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub enable_limit: bool,
    pub min_length: f32,
    pub max_length: f32,
    pub enable_motor: bool,
    pub max_motor_force: f32,
    pub motor_speed: f32,
}

impl<'body, 'world> DistanceJointDef<'body, 'world> {
    pub fn new(body_a: &'body Body<'world>, body_b: &'body Body<'world>) -> Self {
        let raw = unsafe { sys::b3DefaultDistanceJointDef() };
        Self {
            base: JointDef::from_raw(body_a, body_b, raw.base),
            length: raw.length,
            enable_spring: raw.enableSpring,
            lower_spring_force: raw.lowerSpringForce,
            upper_spring_force: raw.upperSpringForce,
            hertz: raw.hertz,
            damping_ratio: raw.dampingRatio,
            enable_limit: raw.enableLimit,
            min_length: raw.minLength,
            max_length: raw.maxLength,
            enable_motor: raw.enableMotor,
            max_motor_force: raw.maxMotorForce,
            motor_speed: raw.motorSpeed,
        }
    }

    fn raw(&self, world: sys::b3WorldId) -> Result<sys::b3DistanceJointDef> {
        let mut raw = unsafe { sys::b3DefaultDistanceJointDef() };
        self.base.apply_to(world, &mut raw.base)?;
        raw.length = self.length;
        raw.enableSpring = self.enable_spring;
        raw.lowerSpringForce = self.lower_spring_force;
        raw.upperSpringForce = self.upper_spring_force;
        raw.hertz = self.hertz;
        raw.dampingRatio = self.damping_ratio;
        raw.enableLimit = self.enable_limit;
        raw.minLength = self.min_length;
        raw.maxLength = self.max_length;
        raw.enableMotor = self.enable_motor;
        raw.maxMotorForce = self.max_motor_force;
        raw.motorSpeed = self.motor_speed;
        Ok(raw)
    }
}

#[derive(Clone, Copy)]
pub struct MotorJointDef<'body, 'world> {
    pub base: JointDef<'body, 'world>,
    pub linear_velocity: Vec3,
    pub max_velocity_force: f32,
    pub angular_velocity: Vec3,
    pub max_velocity_torque: f32,
    pub linear_hertz: f32,
    pub linear_damping_ratio: f32,
    pub max_spring_force: f32,
    pub angular_hertz: f32,
    pub angular_damping_ratio: f32,
    pub max_spring_torque: f32,
}

impl<'body, 'world> MotorJointDef<'body, 'world> {
    pub fn new(body_a: &'body Body<'world>, body_b: &'body Body<'world>) -> Self {
        let raw = unsafe { sys::b3DefaultMotorJointDef() };
        Self {
            base: JointDef::from_raw(body_a, body_b, raw.base),
            linear_velocity: raw.linearVelocity.into(),
            max_velocity_force: raw.maxVelocityForce,
            angular_velocity: raw.angularVelocity.into(),
            max_velocity_torque: raw.maxVelocityTorque,
            linear_hertz: raw.linearHertz,
            linear_damping_ratio: raw.linearDampingRatio,
            max_spring_force: raw.maxSpringForce,
            angular_hertz: raw.angularHertz,
            angular_damping_ratio: raw.angularDampingRatio,
            max_spring_torque: raw.maxSpringTorque,
        }
    }

    fn raw(&self, world: sys::b3WorldId) -> Result<sys::b3MotorJointDef> {
        let mut raw = unsafe { sys::b3DefaultMotorJointDef() };
        self.base.apply_to(world, &mut raw.base)?;
        raw.linearVelocity = self.linear_velocity.into();
        raw.maxVelocityForce = self.max_velocity_force;
        raw.angularVelocity = self.angular_velocity.into();
        raw.maxVelocityTorque = self.max_velocity_torque;
        raw.linearHertz = self.linear_hertz;
        raw.linearDampingRatio = self.linear_damping_ratio;
        raw.maxSpringForce = self.max_spring_force;
        raw.angularHertz = self.angular_hertz;
        raw.angularDampingRatio = self.angular_damping_ratio;
        raw.maxSpringTorque = self.max_spring_torque;
        Ok(raw)
    }
}

#[derive(Clone, Copy)]
pub struct ParallelJointDef<'body, 'world> {
    pub base: JointDef<'body, 'world>,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_torque: f32,
}

impl<'body, 'world> ParallelJointDef<'body, 'world> {
    pub fn new(body_a: &'body Body<'world>, body_b: &'body Body<'world>) -> Self {
        let raw = unsafe { sys::b3DefaultParallelJointDef() };
        Self {
            base: JointDef::from_raw(body_a, body_b, raw.base),
            hertz: raw.hertz,
            damping_ratio: raw.dampingRatio,
            max_torque: raw.maxTorque,
        }
    }

    fn raw(&self, world: sys::b3WorldId) -> Result<sys::b3ParallelJointDef> {
        let mut raw = unsafe { sys::b3DefaultParallelJointDef() };
        self.base.apply_to(world, &mut raw.base)?;
        raw.hertz = self.hertz;
        raw.dampingRatio = self.damping_ratio;
        raw.maxTorque = self.max_torque;
        Ok(raw)
    }
}

#[derive(Clone, Copy)]
pub struct PrismaticJointDef<'body, 'world> {
    pub base: JointDef<'body, 'world>,
    pub enable_spring: bool,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub target_translation: f32,
    pub enable_limit: bool,
    pub lower_translation: f32,
    pub upper_translation: f32,
    pub enable_motor: bool,
    pub max_motor_force: f32,
    pub motor_speed: f32,
}

impl<'body, 'world> PrismaticJointDef<'body, 'world> {
    pub fn new(body_a: &'body Body<'world>, body_b: &'body Body<'world>) -> Self {
        let raw = unsafe { sys::b3DefaultPrismaticJointDef() };
        Self {
            base: JointDef::from_raw(body_a, body_b, raw.base),
            enable_spring: raw.enableSpring,
            hertz: raw.hertz,
            damping_ratio: raw.dampingRatio,
            target_translation: raw.targetTranslation,
            enable_limit: raw.enableLimit,
            lower_translation: raw.lowerTranslation,
            upper_translation: raw.upperTranslation,
            enable_motor: raw.enableMotor,
            max_motor_force: raw.maxMotorForce,
            motor_speed: raw.motorSpeed,
        }
    }

    fn raw(&self, world: sys::b3WorldId) -> Result<sys::b3PrismaticJointDef> {
        let mut raw = unsafe { sys::b3DefaultPrismaticJointDef() };
        self.base.apply_to(world, &mut raw.base)?;
        raw.enableSpring = self.enable_spring;
        raw.hertz = self.hertz;
        raw.dampingRatio = self.damping_ratio;
        raw.targetTranslation = self.target_translation;
        raw.enableLimit = self.enable_limit;
        raw.lowerTranslation = self.lower_translation;
        raw.upperTranslation = self.upper_translation;
        raw.enableMotor = self.enable_motor;
        raw.maxMotorForce = self.max_motor_force;
        raw.motorSpeed = self.motor_speed;
        Ok(raw)
    }
}

#[derive(Clone, Copy)]
pub struct RevoluteJointDef<'body, 'world> {
    pub base: JointDef<'body, 'world>,
    pub target_angle: f32,
    pub enable_spring: bool,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub enable_limit: bool,
    pub lower_angle: f32,
    pub upper_angle: f32,
    pub enable_motor: bool,
    pub max_motor_torque: f32,
    pub motor_speed: f32,
}

impl<'body, 'world> RevoluteJointDef<'body, 'world> {
    pub fn new(body_a: &'body Body<'world>, body_b: &'body Body<'world>) -> Self {
        let raw = unsafe { sys::b3DefaultRevoluteJointDef() };
        Self {
            base: JointDef::from_raw(body_a, body_b, raw.base),
            target_angle: raw.targetAngle,
            enable_spring: raw.enableSpring,
            hertz: raw.hertz,
            damping_ratio: raw.dampingRatio,
            enable_limit: raw.enableLimit,
            lower_angle: raw.lowerAngle,
            upper_angle: raw.upperAngle,
            enable_motor: raw.enableMotor,
            max_motor_torque: raw.maxMotorTorque,
            motor_speed: raw.motorSpeed,
        }
    }

    fn raw(&self, world: sys::b3WorldId) -> Result<sys::b3RevoluteJointDef> {
        let mut raw = unsafe { sys::b3DefaultRevoluteJointDef() };
        self.base.apply_to(world, &mut raw.base)?;
        raw.targetAngle = self.target_angle;
        raw.enableSpring = self.enable_spring;
        raw.hertz = self.hertz;
        raw.dampingRatio = self.damping_ratio;
        raw.enableLimit = self.enable_limit;
        raw.lowerAngle = self.lower_angle;
        raw.upperAngle = self.upper_angle;
        raw.enableMotor = self.enable_motor;
        raw.maxMotorTorque = self.max_motor_torque;
        raw.motorSpeed = self.motor_speed;
        Ok(raw)
    }
}

#[derive(Clone, Copy)]
pub struct SphericalJointDef<'body, 'world> {
    pub base: JointDef<'body, 'world>,
    pub enable_spring: bool,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub target_rotation: Quat,
    pub enable_cone_limit: bool,
    pub cone_angle: f32,
    pub enable_twist_limit: bool,
    pub lower_twist_angle: f32,
    pub upper_twist_angle: f32,
    pub enable_motor: bool,
    pub max_motor_torque: f32,
    pub motor_velocity: Vec3,
}

impl<'body, 'world> SphericalJointDef<'body, 'world> {
    pub fn new(body_a: &'body Body<'world>, body_b: &'body Body<'world>) -> Self {
        let raw = unsafe { sys::b3DefaultSphericalJointDef() };
        Self {
            base: JointDef::from_raw(body_a, body_b, raw.base),
            enable_spring: raw.enableSpring,
            hertz: raw.hertz,
            damping_ratio: raw.dampingRatio,
            target_rotation: raw.targetRotation.into(),
            enable_cone_limit: raw.enableConeLimit,
            cone_angle: raw.coneAngle,
            enable_twist_limit: raw.enableTwistLimit,
            lower_twist_angle: raw.lowerTwistAngle,
            upper_twist_angle: raw.upperTwistAngle,
            enable_motor: raw.enableMotor,
            max_motor_torque: raw.maxMotorTorque,
            motor_velocity: raw.motorVelocity.into(),
        }
    }

    fn raw(&self, world: sys::b3WorldId) -> Result<sys::b3SphericalJointDef> {
        let mut raw = unsafe { sys::b3DefaultSphericalJointDef() };
        self.base.apply_to(world, &mut raw.base)?;
        raw.enableSpring = self.enable_spring;
        raw.hertz = self.hertz;
        raw.dampingRatio = self.damping_ratio;
        raw.targetRotation = self.target_rotation.into();
        raw.enableConeLimit = self.enable_cone_limit;
        raw.coneAngle = self.cone_angle;
        raw.enableTwistLimit = self.enable_twist_limit;
        raw.lowerTwistAngle = self.lower_twist_angle;
        raw.upperTwistAngle = self.upper_twist_angle;
        raw.enableMotor = self.enable_motor;
        raw.maxMotorTorque = self.max_motor_torque;
        raw.motorVelocity = self.motor_velocity.into();
        Ok(raw)
    }
}

#[derive(Clone, Copy)]
pub struct WeldJointDef<'body, 'world> {
    pub base: JointDef<'body, 'world>,
    pub linear_hertz: f32,
    pub angular_hertz: f32,
    pub linear_damping_ratio: f32,
    pub angular_damping_ratio: f32,
}

impl<'body, 'world> WeldJointDef<'body, 'world> {
    pub fn new(body_a: &'body Body<'world>, body_b: &'body Body<'world>) -> Self {
        let raw = unsafe { sys::b3DefaultWeldJointDef() };
        Self {
            base: JointDef::from_raw(body_a, body_b, raw.base),
            linear_hertz: raw.linearHertz,
            angular_hertz: raw.angularHertz,
            linear_damping_ratio: raw.linearDampingRatio,
            angular_damping_ratio: raw.angularDampingRatio,
        }
    }

    fn raw(&self, world: sys::b3WorldId) -> Result<sys::b3WeldJointDef> {
        let mut raw = unsafe { sys::b3DefaultWeldJointDef() };
        self.base.apply_to(world, &mut raw.base)?;
        raw.linearHertz = self.linear_hertz;
        raw.angularHertz = self.angular_hertz;
        raw.linearDampingRatio = self.linear_damping_ratio;
        raw.angularDampingRatio = self.angular_damping_ratio;
        Ok(raw)
    }
}

#[derive(Clone, Copy)]
pub struct WheelJointDef<'body, 'world> {
    pub base: JointDef<'body, 'world>,
    pub enable_suspension_spring: bool,
    pub suspension_hertz: f32,
    pub suspension_damping_ratio: f32,
    pub enable_suspension_limit: bool,
    pub lower_suspension_limit: f32,
    pub upper_suspension_limit: f32,
    pub enable_spin_motor: bool,
    pub max_spin_torque: f32,
    pub spin_speed: f32,
    pub enable_steering: bool,
    pub steering_hertz: f32,
    pub steering_damping_ratio: f32,
    pub target_steering_angle: f32,
    pub max_steering_torque: f32,
    pub enable_steering_limit: bool,
    pub lower_steering_limit: f32,
    pub upper_steering_limit: f32,
}

impl<'body, 'world> WheelJointDef<'body, 'world> {
    pub fn new(body_a: &'body Body<'world>, body_b: &'body Body<'world>) -> Self {
        let raw = unsafe { sys::b3DefaultWheelJointDef() };
        Self {
            base: JointDef::from_raw(body_a, body_b, raw.base),
            enable_suspension_spring: raw.enableSuspensionSpring,
            suspension_hertz: raw.suspensionHertz,
            suspension_damping_ratio: raw.suspensionDampingRatio,
            enable_suspension_limit: raw.enableSuspensionLimit,
            lower_suspension_limit: raw.lowerSuspensionLimit,
            upper_suspension_limit: raw.upperSuspensionLimit,
            enable_spin_motor: raw.enableSpinMotor,
            max_spin_torque: raw.maxSpinTorque,
            spin_speed: raw.spinSpeed,
            enable_steering: raw.enableSteering,
            steering_hertz: raw.steeringHertz,
            steering_damping_ratio: raw.steeringDampingRatio,
            target_steering_angle: raw.targetSteeringAngle,
            max_steering_torque: raw.maxSteeringTorque,
            enable_steering_limit: raw.enableSteeringLimit,
            lower_steering_limit: raw.lowerSteeringLimit,
            upper_steering_limit: raw.upperSteeringLimit,
        }
    }

    fn raw(&self, world: sys::b3WorldId) -> Result<sys::b3WheelJointDef> {
        let mut raw = unsafe { sys::b3DefaultWheelJointDef() };
        self.base.apply_to(world, &mut raw.base)?;
        raw.enableSuspensionSpring = self.enable_suspension_spring;
        raw.suspensionHertz = self.suspension_hertz;
        raw.suspensionDampingRatio = self.suspension_damping_ratio;
        raw.enableSuspensionLimit = self.enable_suspension_limit;
        raw.lowerSuspensionLimit = self.lower_suspension_limit;
        raw.upperSuspensionLimit = self.upper_suspension_limit;
        raw.enableSpinMotor = self.enable_spin_motor;
        raw.maxSpinTorque = self.max_spin_torque;
        raw.spinSpeed = self.spin_speed;
        raw.enableSteering = self.enable_steering;
        raw.steeringHertz = self.steering_hertz;
        raw.steeringDampingRatio = self.steering_damping_ratio;
        raw.targetSteeringAngle = self.target_steering_angle;
        raw.maxSteeringTorque = self.max_steering_torque;
        raw.enableSteeringLimit = self.enable_steering_limit;
        raw.lowerSteeringLimit = self.lower_steering_limit;
        raw.upperSteeringLimit = self.upper_steering_limit;
        Ok(raw)
    }
}

#[derive(Clone, Copy)]
pub struct FilterJointDef<'body, 'world> {
    pub base: JointDef<'body, 'world>,
}

impl<'body, 'world> FilterJointDef<'body, 'world> {
    pub fn new(body_a: &'body Body<'world>, body_b: &'body Body<'world>) -> Self {
        let raw = unsafe { sys::b3DefaultFilterJointDef() };
        Self {
            base: JointDef::from_raw(body_a, body_b, raw.base),
        }
    }

    fn raw(&self, world: sys::b3WorldId) -> Result<sys::b3FilterJointDef> {
        let mut raw = unsafe { sys::b3DefaultFilterJointDef() };
        self.base.apply_to(world, &mut raw.base)?;
        Ok(raw)
    }
}

pub struct Joint<'world> {
    raw: sys::b3JointId,
    _world: PhantomData<&'world World>,
}

impl<'world> Joint<'world> {
    fn from_raw(raw: sys::b3JointId) -> Self {
        Self {
            raw,
            _world: PhantomData,
        }
    }

    fn raw(&self) -> sys::b3JointId {
        self.raw
    }

    pub fn destroy(self) {
        drop(self);
    }

    pub fn is_valid(&self) -> bool {
        handle::is_joint_valid(self.raw)
    }

    pub fn joint_type(&self) -> JointType {
        unsafe { sys::b3Joint_GetType(self.raw) }.into()
    }

    pub fn set_local_frame_a(&self, frame: Transform) {
        unsafe { sys::b3Joint_SetLocalFrameA(self.raw, frame.into()) };
    }

    pub fn local_frame_a(&self) -> Transform {
        unsafe { sys::b3Joint_GetLocalFrameA(self.raw) }.into()
    }

    pub fn set_local_frame_b(&self, frame: Transform) {
        unsafe { sys::b3Joint_SetLocalFrameB(self.raw, frame.into()) };
    }

    pub fn local_frame_b(&self) -> Transform {
        unsafe { sys::b3Joint_GetLocalFrameB(self.raw) }.into()
    }

    pub fn set_collide_connected(&self, collide: bool) {
        unsafe { sys::b3Joint_SetCollideConnected(self.raw, collide) };
    }

    pub fn collide_connected(&self) -> bool {
        unsafe { sys::b3Joint_GetCollideConnected(self.raw) }
    }

    pub fn wake_bodies(&self) {
        unsafe { sys::b3Joint_WakeBodies(self.raw) };
    }

    pub fn constraint_force(&self) -> Vec3 {
        unsafe { sys::b3Joint_GetConstraintForce(self.raw) }.into()
    }

    pub fn constraint_torque(&self) -> Vec3 {
        unsafe { sys::b3Joint_GetConstraintTorque(self.raw) }.into()
    }

    pub fn linear_separation(&self) -> f32 {
        unsafe { sys::b3Joint_GetLinearSeparation(self.raw) }
    }

    pub fn angular_separation(&self) -> Option<f32> {
        if self.joint_type() == JointType::Wheel {
            None
        } else {
            Some(unsafe { sys::b3Joint_GetAngularSeparation(self.raw) })
        }
    }

    pub fn set_constraint_tuning(&self, hertz: f32, damping_ratio: f32) {
        unsafe { sys::b3Joint_SetConstraintTuning(self.raw, hertz, damping_ratio) };
    }

    pub fn constraint_tuning(&self) -> (f32, f32) {
        let mut hertz = 0.0;
        let mut damping_ratio = 0.0;
        unsafe { sys::b3Joint_GetConstraintTuning(self.raw, &mut hertz, &mut damping_ratio) };
        (hertz, damping_ratio)
    }

    pub fn set_force_threshold(&self, threshold: f32) {
        unsafe { sys::b3Joint_SetForceThreshold(self.raw, threshold) };
    }

    pub fn force_threshold(&self) -> f32 {
        unsafe { sys::b3Joint_GetForceThreshold(self.raw) }
    }

    pub fn set_torque_threshold(&self, threshold: f32) {
        unsafe { sys::b3Joint_SetTorqueThreshold(self.raw, threshold) };
    }

    pub fn torque_threshold(&self) -> f32 {
        unsafe { sys::b3Joint_GetTorqueThreshold(self.raw) }
    }
}

impl Drop for Joint<'_> {
    fn drop(&mut self) {
        if handle::is_joint_valid(self.raw) {
            unsafe { sys::b3DestroyJoint(self.raw, true) };
        }
    }
}

macro_rules! typed_joint {
    ($name:ident) => {
        pub struct $name<'world> {
            joint: Joint<'world>,
        }

        impl<'world> $name<'world> {
            fn from_raw(raw: sys::b3JointId) -> Self {
                Self {
                    joint: Joint::from_raw(raw),
                }
            }

            pub fn destroy(self) {
                drop(self);
            }
        }

        impl<'world> Deref for $name<'world> {
            type Target = Joint<'world>;

            fn deref(&self) -> &Self::Target {
                &self.joint
            }
        }
    };
}

typed_joint!(DistanceJoint);
typed_joint!(MotorJoint);
typed_joint!(ParallelJoint);
typed_joint!(PrismaticJoint);
typed_joint!(RevoluteJoint);
typed_joint!(SphericalJoint);
typed_joint!(WeldJoint);
typed_joint!(WheelJoint);
typed_joint!(FilterJoint);

impl DistanceJoint<'_> {
    pub fn set_length(&self, length: f32) {
        unsafe { sys::b3DistanceJoint_SetLength(self.raw(), length) };
    }

    pub fn length(&self) -> f32 {
        unsafe { sys::b3DistanceJoint_GetLength(self.raw()) }
    }

    pub fn set_spring_enabled(&self, enabled: bool) {
        unsafe { sys::b3DistanceJoint_EnableSpring(self.raw(), enabled) };
    }

    pub fn is_spring_enabled(&self) -> bool {
        unsafe { sys::b3DistanceJoint_IsSpringEnabled(self.raw()) }
    }

    pub fn set_spring_force_range(&self, lower_force: f32, upper_force: f32) {
        unsafe { sys::b3DistanceJoint_SetSpringForceRange(self.raw(), lower_force, upper_force) };
    }

    pub fn spring_force_range(&self) -> (f32, f32) {
        let mut lower_force = 0.0;
        let mut upper_force = 0.0;
        unsafe {
            sys::b3DistanceJoint_GetSpringForceRange(self.raw(), &mut lower_force, &mut upper_force)
        };
        (lower_force, upper_force)
    }

    pub fn set_spring_hertz(&self, hertz: f32) {
        unsafe { sys::b3DistanceJoint_SetSpringHertz(self.raw(), hertz) };
    }

    pub fn spring_hertz(&self) -> f32 {
        unsafe { sys::b3DistanceJoint_GetSpringHertz(self.raw()) }
    }

    pub fn set_spring_damping_ratio(&self, damping_ratio: f32) {
        unsafe { sys::b3DistanceJoint_SetSpringDampingRatio(self.raw(), damping_ratio) };
    }

    pub fn spring_damping_ratio(&self) -> f32 {
        unsafe { sys::b3DistanceJoint_GetSpringDampingRatio(self.raw()) }
    }

    pub fn set_limit_enabled(&self, enabled: bool) {
        unsafe { sys::b3DistanceJoint_EnableLimit(self.raw(), enabled) };
    }

    pub fn is_limit_enabled(&self) -> bool {
        unsafe { sys::b3DistanceJoint_IsLimitEnabled(self.raw()) }
    }

    pub fn set_length_range(&self, min_length: f32, max_length: f32) {
        unsafe { sys::b3DistanceJoint_SetLengthRange(self.raw(), min_length, max_length) };
    }

    pub fn min_length(&self) -> f32 {
        unsafe { sys::b3DistanceJoint_GetMinLength(self.raw()) }
    }

    pub fn max_length(&self) -> f32 {
        unsafe { sys::b3DistanceJoint_GetMaxLength(self.raw()) }
    }

    pub fn current_length(&self) -> f32 {
        unsafe { sys::b3DistanceJoint_GetCurrentLength(self.raw()) }
    }

    pub fn set_motor_enabled(&self, enabled: bool) {
        unsafe { sys::b3DistanceJoint_EnableMotor(self.raw(), enabled) };
    }

    pub fn is_motor_enabled(&self) -> bool {
        unsafe { sys::b3DistanceJoint_IsMotorEnabled(self.raw()) }
    }

    pub fn set_motor_speed(&self, motor_speed: f32) {
        unsafe { sys::b3DistanceJoint_SetMotorSpeed(self.raw(), motor_speed) };
    }

    pub fn motor_speed(&self) -> f32 {
        unsafe { sys::b3DistanceJoint_GetMotorSpeed(self.raw()) }
    }

    pub fn set_max_motor_force(&self, force: f32) {
        unsafe { sys::b3DistanceJoint_SetMaxMotorForce(self.raw(), force) };
    }

    pub fn max_motor_force(&self) -> f32 {
        unsafe { sys::b3DistanceJoint_GetMaxMotorForce(self.raw()) }
    }

    pub fn motor_force(&self) -> f32 {
        unsafe { sys::b3DistanceJoint_GetMotorForce(self.raw()) }
    }
}

impl MotorJoint<'_> {
    pub fn set_linear_velocity(&self, velocity: Vec3) {
        unsafe { sys::b3MotorJoint_SetLinearVelocity(self.raw(), velocity.into()) };
    }

    pub fn linear_velocity(&self) -> Vec3 {
        unsafe { sys::b3MotorJoint_GetLinearVelocity(self.raw()) }.into()
    }

    pub fn set_angular_velocity(&self, velocity: Vec3) {
        unsafe { sys::b3MotorJoint_SetAngularVelocity(self.raw(), velocity.into()) };
    }

    pub fn angular_velocity(&self) -> Vec3 {
        unsafe { sys::b3MotorJoint_GetAngularVelocity(self.raw()) }.into()
    }

    pub fn set_max_velocity_force(&self, max_force: f32) {
        unsafe { sys::b3MotorJoint_SetMaxVelocityForce(self.raw(), max_force) };
    }

    pub fn max_velocity_force(&self) -> f32 {
        unsafe { sys::b3MotorJoint_GetMaxVelocityForce(self.raw()) }
    }

    pub fn set_max_velocity_torque(&self, max_torque: f32) {
        unsafe { sys::b3MotorJoint_SetMaxVelocityTorque(self.raw(), max_torque) };
    }

    pub fn max_velocity_torque(&self) -> f32 {
        unsafe { sys::b3MotorJoint_GetMaxVelocityTorque(self.raw()) }
    }

    pub fn set_linear_hertz(&self, hertz: f32) {
        unsafe { sys::b3MotorJoint_SetLinearHertz(self.raw(), hertz) };
    }

    pub fn linear_hertz(&self) -> f32 {
        unsafe { sys::b3MotorJoint_GetLinearHertz(self.raw()) }
    }

    pub fn set_linear_damping_ratio(&self, damping: f32) {
        unsafe { sys::b3MotorJoint_SetLinearDampingRatio(self.raw(), damping) };
    }

    pub fn linear_damping_ratio(&self) -> f32 {
        unsafe { sys::b3MotorJoint_GetLinearDampingRatio(self.raw()) }
    }

    pub fn set_angular_hertz(&self, hertz: f32) {
        unsafe { sys::b3MotorJoint_SetAngularHertz(self.raw(), hertz) };
    }

    pub fn angular_hertz(&self) -> f32 {
        unsafe { sys::b3MotorJoint_GetAngularHertz(self.raw()) }
    }

    pub fn set_angular_damping_ratio(&self, damping: f32) {
        unsafe { sys::b3MotorJoint_SetAngularDampingRatio(self.raw(), damping) };
    }

    pub fn angular_damping_ratio(&self) -> f32 {
        unsafe { sys::b3MotorJoint_GetAngularDampingRatio(self.raw()) }
    }

    pub fn set_max_spring_force(&self, max_force: f32) {
        unsafe { sys::b3MotorJoint_SetMaxSpringForce(self.raw(), max_force) };
    }

    pub fn max_spring_force(&self) -> f32 {
        unsafe { sys::b3MotorJoint_GetMaxSpringForce(self.raw()) }
    }

    pub fn set_max_spring_torque(&self, max_torque: f32) {
        unsafe { sys::b3MotorJoint_SetMaxSpringTorque(self.raw(), max_torque) };
    }

    pub fn max_spring_torque(&self) -> f32 {
        unsafe { sys::b3MotorJoint_GetMaxSpringTorque(self.raw()) }
    }
}

impl ParallelJoint<'_> {
    pub fn set_spring_hertz(&self, hertz: f32) {
        unsafe { sys::b3ParallelJoint_SetSpringHertz(self.raw(), hertz) };
    }

    pub fn spring_hertz(&self) -> f32 {
        unsafe { sys::b3ParallelJoint_GetSpringHertz(self.raw()) }
    }

    pub fn set_spring_damping_ratio(&self, damping_ratio: f32) {
        unsafe { sys::b3ParallelJoint_SetSpringDampingRatio(self.raw(), damping_ratio) };
    }

    pub fn spring_damping_ratio(&self) -> f32 {
        unsafe { sys::b3ParallelJoint_GetSpringDampingRatio(self.raw()) }
    }

    pub fn set_max_torque(&self, torque: f32) {
        unsafe { sys::b3ParallelJoint_SetMaxTorque(self.raw(), torque) };
    }

    pub fn max_torque(&self) -> f32 {
        unsafe { sys::b3ParallelJoint_GetMaxTorque(self.raw()) }
    }
}

impl PrismaticJoint<'_> {
    pub fn set_spring_enabled(&self, enabled: bool) {
        unsafe { sys::b3PrismaticJoint_EnableSpring(self.raw(), enabled) };
    }

    pub fn is_spring_enabled(&self) -> bool {
        unsafe { sys::b3PrismaticJoint_IsSpringEnabled(self.raw()) }
    }

    pub fn set_spring_hertz(&self, hertz: f32) {
        unsafe { sys::b3PrismaticJoint_SetSpringHertz(self.raw(), hertz) };
    }

    pub fn spring_hertz(&self) -> f32 {
        unsafe { sys::b3PrismaticJoint_GetSpringHertz(self.raw()) }
    }

    pub fn set_spring_damping_ratio(&self, damping_ratio: f32) {
        unsafe { sys::b3PrismaticJoint_SetSpringDampingRatio(self.raw(), damping_ratio) };
    }

    pub fn spring_damping_ratio(&self) -> f32 {
        unsafe { sys::b3PrismaticJoint_GetSpringDampingRatio(self.raw()) }
    }

    pub fn set_target_translation(&self, target_translation: f32) {
        unsafe { sys::b3PrismaticJoint_SetTargetTranslation(self.raw(), target_translation) };
    }

    pub fn target_translation(&self) -> f32 {
        unsafe { sys::b3PrismaticJoint_GetTargetTranslation(self.raw()) }
    }

    pub fn set_limit_enabled(&self, enabled: bool) {
        unsafe { sys::b3PrismaticJoint_EnableLimit(self.raw(), enabled) };
    }

    pub fn is_limit_enabled(&self) -> bool {
        unsafe { sys::b3PrismaticJoint_IsLimitEnabled(self.raw()) }
    }

    pub fn lower_limit(&self) -> f32 {
        unsafe { sys::b3PrismaticJoint_GetLowerLimit(self.raw()) }
    }

    pub fn upper_limit(&self) -> f32 {
        unsafe { sys::b3PrismaticJoint_GetUpperLimit(self.raw()) }
    }

    pub fn set_limits(&self, lower: f32, upper: f32) {
        unsafe { sys::b3PrismaticJoint_SetLimits(self.raw(), lower, upper) };
    }

    pub fn set_motor_enabled(&self, enabled: bool) {
        unsafe { sys::b3PrismaticJoint_EnableMotor(self.raw(), enabled) };
    }

    pub fn is_motor_enabled(&self) -> bool {
        unsafe { sys::b3PrismaticJoint_IsMotorEnabled(self.raw()) }
    }

    pub fn set_motor_speed(&self, motor_speed: f32) {
        unsafe { sys::b3PrismaticJoint_SetMotorSpeed(self.raw(), motor_speed) };
    }

    pub fn motor_speed(&self) -> f32 {
        unsafe { sys::b3PrismaticJoint_GetMotorSpeed(self.raw()) }
    }

    pub fn set_max_motor_force(&self, force: f32) {
        unsafe { sys::b3PrismaticJoint_SetMaxMotorForce(self.raw(), force) };
    }

    pub fn max_motor_force(&self) -> f32 {
        unsafe { sys::b3PrismaticJoint_GetMaxMotorForce(self.raw()) }
    }

    pub fn motor_force(&self) -> f32 {
        unsafe { sys::b3PrismaticJoint_GetMotorForce(self.raw()) }
    }

    pub fn translation(&self) -> f32 {
        unsafe { sys::b3PrismaticJoint_GetTranslation(self.raw()) }
    }

    pub fn speed(&self) -> f32 {
        unsafe { sys::b3PrismaticJoint_GetSpeed(self.raw()) }
    }
}

impl RevoluteJoint<'_> {
    pub fn set_spring_enabled(&self, enabled: bool) {
        unsafe { sys::b3RevoluteJoint_EnableSpring(self.raw(), enabled) };
    }

    pub fn is_spring_enabled(&self) -> bool {
        unsafe { sys::b3RevoluteJoint_IsSpringEnabled(self.raw()) }
    }

    pub fn set_spring_hertz(&self, hertz: f32) {
        unsafe { sys::b3RevoluteJoint_SetSpringHertz(self.raw(), hertz) };
    }

    pub fn spring_hertz(&self) -> f32 {
        unsafe { sys::b3RevoluteJoint_GetSpringHertz(self.raw()) }
    }

    pub fn set_spring_damping_ratio(&self, damping_ratio: f32) {
        unsafe { sys::b3RevoluteJoint_SetSpringDampingRatio(self.raw(), damping_ratio) };
    }

    pub fn spring_damping_ratio(&self) -> f32 {
        unsafe { sys::b3RevoluteJoint_GetSpringDampingRatio(self.raw()) }
    }

    pub fn set_target_angle(&self, target_radians: f32) {
        unsafe { sys::b3RevoluteJoint_SetTargetAngle(self.raw(), target_radians) };
    }

    pub fn target_angle(&self) -> f32 {
        unsafe { sys::b3RevoluteJoint_GetTargetAngle(self.raw()) }
    }

    pub fn angle(&self) -> f32 {
        unsafe { sys::b3RevoluteJoint_GetAngle(self.raw()) }
    }

    pub fn set_limit_enabled(&self, enabled: bool) {
        unsafe { sys::b3RevoluteJoint_EnableLimit(self.raw(), enabled) };
    }

    pub fn is_limit_enabled(&self) -> bool {
        unsafe { sys::b3RevoluteJoint_IsLimitEnabled(self.raw()) }
    }

    pub fn lower_limit(&self) -> f32 {
        unsafe { sys::b3RevoluteJoint_GetLowerLimit(self.raw()) }
    }

    pub fn upper_limit(&self) -> f32 {
        unsafe { sys::b3RevoluteJoint_GetUpperLimit(self.raw()) }
    }

    pub fn set_limits(&self, lower_limit_radians: f32, upper_limit_radians: f32) {
        unsafe {
            sys::b3RevoluteJoint_SetLimits(self.raw(), lower_limit_radians, upper_limit_radians)
        };
    }

    pub fn set_motor_enabled(&self, enabled: bool) {
        unsafe { sys::b3RevoluteJoint_EnableMotor(self.raw(), enabled) };
    }

    pub fn is_motor_enabled(&self) -> bool {
        unsafe { sys::b3RevoluteJoint_IsMotorEnabled(self.raw()) }
    }

    pub fn set_motor_speed(&self, motor_speed: f32) {
        unsafe { sys::b3RevoluteJoint_SetMotorSpeed(self.raw(), motor_speed) };
    }

    pub fn motor_speed(&self) -> f32 {
        unsafe { sys::b3RevoluteJoint_GetMotorSpeed(self.raw()) }
    }

    pub fn motor_torque(&self) -> f32 {
        unsafe { sys::b3RevoluteJoint_GetMotorTorque(self.raw()) }
    }

    pub fn set_max_motor_torque(&self, torque: f32) {
        unsafe { sys::b3RevoluteJoint_SetMaxMotorTorque(self.raw(), torque) };
    }

    pub fn max_motor_torque(&self) -> f32 {
        unsafe { sys::b3RevoluteJoint_GetMaxMotorTorque(self.raw()) }
    }
}

impl SphericalJoint<'_> {
    pub fn set_cone_limit_enabled(&self, enabled: bool) {
        unsafe { sys::b3SphericalJoint_EnableConeLimit(self.raw(), enabled) };
    }

    pub fn is_cone_limit_enabled(&self) -> bool {
        unsafe { sys::b3SphericalJoint_IsConeLimitEnabled(self.raw()) }
    }

    pub fn cone_limit(&self) -> f32 {
        unsafe { sys::b3SphericalJoint_GetConeLimit(self.raw()) }
    }

    pub fn set_cone_limit(&self, angle_radians: f32) {
        unsafe { sys::b3SphericalJoint_SetConeLimit(self.raw(), angle_radians) };
    }

    pub fn cone_angle(&self) -> f32 {
        unsafe { sys::b3SphericalJoint_GetConeAngle(self.raw()) }
    }

    pub fn set_twist_limit_enabled(&self, enabled: bool) {
        unsafe { sys::b3SphericalJoint_EnableTwistLimit(self.raw(), enabled) };
    }

    pub fn is_twist_limit_enabled(&self) -> bool {
        unsafe { sys::b3SphericalJoint_IsTwistLimitEnabled(self.raw()) }
    }

    pub fn lower_twist_limit(&self) -> f32 {
        unsafe { sys::b3SphericalJoint_GetLowerTwistLimit(self.raw()) }
    }

    pub fn upper_twist_limit(&self) -> f32 {
        unsafe { sys::b3SphericalJoint_GetUpperTwistLimit(self.raw()) }
    }

    pub fn set_twist_limits(&self, lower_limit_radians: f32, upper_limit_radians: f32) {
        unsafe {
            sys::b3SphericalJoint_SetTwistLimits(
                self.raw(),
                lower_limit_radians,
                upper_limit_radians,
            )
        };
    }

    pub fn twist_angle(&self) -> f32 {
        unsafe { sys::b3SphericalJoint_GetTwistAngle(self.raw()) }
    }

    pub fn set_spring_enabled(&self, enabled: bool) {
        unsafe { sys::b3SphericalJoint_EnableSpring(self.raw(), enabled) };
    }

    pub fn is_spring_enabled(&self) -> bool {
        unsafe { sys::b3SphericalJoint_IsSpringEnabled(self.raw()) }
    }

    pub fn set_spring_hertz(&self, hertz: f32) {
        unsafe { sys::b3SphericalJoint_SetSpringHertz(self.raw(), hertz) };
    }

    pub fn spring_hertz(&self) -> f32 {
        unsafe { sys::b3SphericalJoint_GetSpringHertz(self.raw()) }
    }

    pub fn set_spring_damping_ratio(&self, damping_ratio: f32) {
        unsafe { sys::b3SphericalJoint_SetSpringDampingRatio(self.raw(), damping_ratio) };
    }

    pub fn spring_damping_ratio(&self) -> f32 {
        unsafe { sys::b3SphericalJoint_GetSpringDampingRatio(self.raw()) }
    }

    pub fn set_target_rotation(&self, target_rotation: Quat) {
        unsafe { sys::b3SphericalJoint_SetTargetRotation(self.raw(), target_rotation.into()) };
    }

    pub fn target_rotation(&self) -> Quat {
        unsafe { sys::b3SphericalJoint_GetTargetRotation(self.raw()) }.into()
    }

    pub fn set_motor_enabled(&self, enabled: bool) {
        unsafe { sys::b3SphericalJoint_EnableMotor(self.raw(), enabled) };
    }

    pub fn is_motor_enabled(&self) -> bool {
        unsafe { sys::b3SphericalJoint_IsMotorEnabled(self.raw()) }
    }

    pub fn set_motor_velocity(&self, motor_velocity: Vec3) {
        unsafe { sys::b3SphericalJoint_SetMotorVelocity(self.raw(), motor_velocity.into()) };
    }

    pub fn motor_velocity(&self) -> Vec3 {
        unsafe { sys::b3SphericalJoint_GetMotorVelocity(self.raw()) }.into()
    }

    pub fn motor_torque(&self) -> Vec3 {
        unsafe { sys::b3SphericalJoint_GetMotorTorque(self.raw()) }.into()
    }

    pub fn set_max_motor_torque(&self, torque: f32) {
        unsafe { sys::b3SphericalJoint_SetMaxMotorTorque(self.raw(), torque) };
    }

    pub fn max_motor_torque(&self) -> f32 {
        unsafe { sys::b3SphericalJoint_GetMaxMotorTorque(self.raw()) }
    }
}

impl WeldJoint<'_> {
    pub fn set_linear_hertz(&self, hertz: f32) {
        unsafe { sys::b3WeldJoint_SetLinearHertz(self.raw(), hertz) };
    }

    pub fn linear_hertz(&self) -> f32 {
        unsafe { sys::b3WeldJoint_GetLinearHertz(self.raw()) }
    }

    pub fn set_linear_damping_ratio(&self, damping_ratio: f32) {
        unsafe { sys::b3WeldJoint_SetLinearDampingRatio(self.raw(), damping_ratio) };
    }

    pub fn linear_damping_ratio(&self) -> f32 {
        unsafe { sys::b3WeldJoint_GetLinearDampingRatio(self.raw()) }
    }

    pub fn set_angular_hertz(&self, hertz: f32) {
        unsafe { sys::b3WeldJoint_SetAngularHertz(self.raw(), hertz) };
    }

    pub fn angular_hertz(&self) -> f32 {
        unsafe { sys::b3WeldJoint_GetAngularHertz(self.raw()) }
    }

    pub fn set_angular_damping_ratio(&self, damping_ratio: f32) {
        unsafe { sys::b3WeldJoint_SetAngularDampingRatio(self.raw(), damping_ratio) };
    }

    pub fn angular_damping_ratio(&self) -> f32 {
        unsafe { sys::b3WeldJoint_GetAngularDampingRatio(self.raw()) }
    }
}

impl WheelJoint<'_> {
    pub fn set_suspension_enabled(&self, enabled: bool) {
        unsafe { sys::b3WheelJoint_EnableSuspension(self.raw(), enabled) };
    }

    pub fn is_suspension_enabled(&self) -> bool {
        unsafe { sys::b3WheelJoint_IsSuspensionEnabled(self.raw()) }
    }

    pub fn set_suspension_hertz(&self, hertz: f32) {
        unsafe { sys::b3WheelJoint_SetSuspensionHertz(self.raw(), hertz) };
    }

    pub fn suspension_hertz(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetSuspensionHertz(self.raw()) }
    }

    pub fn set_suspension_damping_ratio(&self, damping_ratio: f32) {
        unsafe { sys::b3WheelJoint_SetSuspensionDampingRatio(self.raw(), damping_ratio) };
    }

    pub fn suspension_damping_ratio(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetSuspensionDampingRatio(self.raw()) }
    }

    pub fn set_suspension_limit_enabled(&self, enabled: bool) {
        unsafe { sys::b3WheelJoint_EnableSuspensionLimit(self.raw(), enabled) };
    }

    pub fn is_suspension_limit_enabled(&self) -> bool {
        unsafe { sys::b3WheelJoint_IsSuspensionLimitEnabled(self.raw()) }
    }

    pub fn lower_suspension_limit(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetLowerSuspensionLimit(self.raw()) }
    }

    pub fn upper_suspension_limit(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetUpperSuspensionLimit(self.raw()) }
    }

    pub fn set_suspension_limits(&self, lower: f32, upper: f32) {
        unsafe { sys::b3WheelJoint_SetSuspensionLimits(self.raw(), lower, upper) };
    }

    pub fn set_spin_motor_enabled(&self, enabled: bool) {
        unsafe { sys::b3WheelJoint_EnableSpinMotor(self.raw(), enabled) };
    }

    pub fn is_spin_motor_enabled(&self) -> bool {
        unsafe { sys::b3WheelJoint_IsSpinMotorEnabled(self.raw()) }
    }

    pub fn set_spin_motor_speed(&self, speed: f32) {
        unsafe { sys::b3WheelJoint_SetSpinMotorSpeed(self.raw(), speed) };
    }

    pub fn spin_motor_speed(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetSpinMotorSpeed(self.raw()) }
    }

    pub fn set_max_spin_torque(&self, torque: f32) {
        unsafe { sys::b3WheelJoint_SetMaxSpinTorque(self.raw(), torque) };
    }

    pub fn max_spin_torque(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetMaxSpinTorque(self.raw()) }
    }

    pub fn spin_speed(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetSpinSpeed(self.raw()) }
    }

    pub fn spin_torque(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetSpinTorque(self.raw()) }
    }

    pub fn set_steering_enabled(&self, enabled: bool) {
        unsafe { sys::b3WheelJoint_EnableSteering(self.raw(), enabled) };
    }

    pub fn is_steering_enabled(&self) -> bool {
        unsafe { sys::b3WheelJoint_IsSteeringEnabled(self.raw()) }
    }

    pub fn set_steering_hertz(&self, hertz: f32) {
        unsafe { sys::b3WheelJoint_SetSteeringHertz(self.raw(), hertz) };
    }

    pub fn steering_hertz(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetSteeringHertz(self.raw()) }
    }

    pub fn set_steering_damping_ratio(&self, damping_ratio: f32) {
        unsafe { sys::b3WheelJoint_SetSteeringDampingRatio(self.raw(), damping_ratio) };
    }

    pub fn steering_damping_ratio(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetSteeringDampingRatio(self.raw()) }
    }

    pub fn set_max_steering_torque(&self, torque: f32) {
        unsafe { sys::b3WheelJoint_SetMaxSteeringTorque(self.raw(), torque) };
    }

    pub fn max_steering_torque(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetMaxSteeringTorque(self.raw()) }
    }

    pub fn set_steering_limit_enabled(&self, enabled: bool) {
        unsafe { sys::b3WheelJoint_EnableSteeringLimit(self.raw(), enabled) };
    }

    pub fn is_steering_limit_enabled(&self) -> bool {
        unsafe { sys::b3WheelJoint_IsSteeringLimitEnabled(self.raw()) }
    }

    pub fn lower_steering_limit(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetLowerSteeringLimit(self.raw()) }
    }

    pub fn upper_steering_limit(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetUpperSteeringLimit(self.raw()) }
    }

    pub fn set_steering_limits(&self, lower_radians: f32, upper_radians: f32) {
        unsafe { sys::b3WheelJoint_SetSteeringLimits(self.raw(), lower_radians, upper_radians) };
    }

    pub fn set_target_steering_angle(&self, radians: f32) {
        unsafe { sys::b3WheelJoint_SetTargetSteeringAngle(self.raw(), radians) };
    }

    pub fn target_steering_angle(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetTargetSteeringAngle(self.raw()) }
    }

    pub fn steering_angle(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetSteeringAngle(self.raw()) }
    }

    pub fn steering_torque(&self) -> f32 {
        unsafe { sys::b3WheelJoint_GetSteeringTorque(self.raw()) }
    }
}

fn validate_joint(raw: sys::b3JointId) -> Result<sys::b3JointId> {
    if handle::is_joint_valid(raw) {
        Ok(raw)
    } else {
        Err(Error::InvalidInput)
    }
}

impl World {
    pub fn create_distance_joint<'world>(
        &'world self,
        def: DistanceJointDef<'_, 'world>,
    ) -> DistanceJoint<'world> {
        self.try_create_distance_joint(def)
            .expect("box3d returned an invalid distance joint")
    }

    pub fn try_create_distance_joint<'world>(
        &'world self,
        def: DistanceJointDef<'_, 'world>,
    ) -> Result<DistanceJoint<'world>> {
        let raw_def = def.raw(self.raw())?;
        let raw = validate_joint(unsafe { sys::b3CreateDistanceJoint(self.raw(), &raw_def) })?;
        Ok(DistanceJoint::from_raw(raw))
    }

    pub fn create_motor_joint<'world>(
        &'world self,
        def: MotorJointDef<'_, 'world>,
    ) -> MotorJoint<'world> {
        self.try_create_motor_joint(def)
            .expect("box3d returned an invalid motor joint")
    }

    pub fn try_create_motor_joint<'world>(
        &'world self,
        def: MotorJointDef<'_, 'world>,
    ) -> Result<MotorJoint<'world>> {
        let raw_def = def.raw(self.raw())?;
        let raw = validate_joint(unsafe { sys::b3CreateMotorJoint(self.raw(), &raw_def) })?;
        Ok(MotorJoint::from_raw(raw))
    }

    pub fn create_parallel_joint<'world>(
        &'world self,
        def: ParallelJointDef<'_, 'world>,
    ) -> ParallelJoint<'world> {
        self.try_create_parallel_joint(def)
            .expect("box3d returned an invalid parallel joint")
    }

    pub fn try_create_parallel_joint<'world>(
        &'world self,
        def: ParallelJointDef<'_, 'world>,
    ) -> Result<ParallelJoint<'world>> {
        let raw_def = def.raw(self.raw())?;
        let raw = validate_joint(unsafe { sys::b3CreateParallelJoint(self.raw(), &raw_def) })?;
        Ok(ParallelJoint::from_raw(raw))
    }

    pub fn create_prismatic_joint<'world>(
        &'world self,
        def: PrismaticJointDef<'_, 'world>,
    ) -> PrismaticJoint<'world> {
        self.try_create_prismatic_joint(def)
            .expect("box3d returned an invalid prismatic joint")
    }

    pub fn try_create_prismatic_joint<'world>(
        &'world self,
        def: PrismaticJointDef<'_, 'world>,
    ) -> Result<PrismaticJoint<'world>> {
        let raw_def = def.raw(self.raw())?;
        let raw = validate_joint(unsafe { sys::b3CreatePrismaticJoint(self.raw(), &raw_def) })?;
        Ok(PrismaticJoint::from_raw(raw))
    }

    pub fn create_revolute_joint<'world>(
        &'world self,
        def: RevoluteJointDef<'_, 'world>,
    ) -> RevoluteJoint<'world> {
        self.try_create_revolute_joint(def)
            .expect("box3d returned an invalid revolute joint")
    }

    pub fn try_create_revolute_joint<'world>(
        &'world self,
        def: RevoluteJointDef<'_, 'world>,
    ) -> Result<RevoluteJoint<'world>> {
        let raw_def = def.raw(self.raw())?;
        let raw = validate_joint(unsafe { sys::b3CreateRevoluteJoint(self.raw(), &raw_def) })?;
        Ok(RevoluteJoint::from_raw(raw))
    }

    pub fn create_spherical_joint<'world>(
        &'world self,
        def: SphericalJointDef<'_, 'world>,
    ) -> SphericalJoint<'world> {
        self.try_create_spherical_joint(def)
            .expect("box3d returned an invalid spherical joint")
    }

    pub fn try_create_spherical_joint<'world>(
        &'world self,
        def: SphericalJointDef<'_, 'world>,
    ) -> Result<SphericalJoint<'world>> {
        let raw_def = def.raw(self.raw())?;
        let raw = validate_joint(unsafe { sys::b3CreateSphericalJoint(self.raw(), &raw_def) })?;
        Ok(SphericalJoint::from_raw(raw))
    }

    pub fn create_weld_joint<'world>(
        &'world self,
        def: WeldJointDef<'_, 'world>,
    ) -> WeldJoint<'world> {
        self.try_create_weld_joint(def)
            .expect("box3d returned an invalid weld joint")
    }

    pub fn try_create_weld_joint<'world>(
        &'world self,
        def: WeldJointDef<'_, 'world>,
    ) -> Result<WeldJoint<'world>> {
        let raw_def = def.raw(self.raw())?;
        let raw = validate_joint(unsafe { sys::b3CreateWeldJoint(self.raw(), &raw_def) })?;
        Ok(WeldJoint::from_raw(raw))
    }

    pub fn create_wheel_joint<'world>(
        &'world self,
        def: WheelJointDef<'_, 'world>,
    ) -> WheelJoint<'world> {
        self.try_create_wheel_joint(def)
            .expect("box3d returned an invalid wheel joint")
    }

    pub fn try_create_wheel_joint<'world>(
        &'world self,
        def: WheelJointDef<'_, 'world>,
    ) -> Result<WheelJoint<'world>> {
        let raw_def = def.raw(self.raw())?;
        let raw = validate_joint(unsafe { sys::b3CreateWheelJoint(self.raw(), &raw_def) })?;
        Ok(WheelJoint::from_raw(raw))
    }

    pub fn create_filter_joint<'world>(
        &'world self,
        def: FilterJointDef<'_, 'world>,
    ) -> FilterJoint<'world> {
        self.try_create_filter_joint(def)
            .expect("box3d returned an invalid filter joint")
    }

    pub fn try_create_filter_joint<'world>(
        &'world self,
        def: FilterJointDef<'_, 'world>,
    ) -> Result<FilterJoint<'world>> {
        let raw_def = def.raw(self.raw())?;
        let raw = validate_joint(unsafe { sys::b3CreateFilterJoint(self.raw(), &raw_def) })?;
        Ok(FilterJoint::from_raw(raw))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{body::BodyDef, shape::ShapeDef};

    fn box_shape<'body>(body: &'body Body<'_>) -> crate::shape::Shape<'body> {
        body.create_box(
            Vec3::new(0.5, 0.5, 0.5),
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                ..ShapeDef::default()
            },
        )
    }

    #[test]
    fn distance_joint_keeps_length() {
        let world = World::new(Vec3::ZERO);
        let body_a = world.create_body(BodyDef::dynamic_at(Vec3::new(-1.0, 0.0, 0.0)));
        let body_b = world.create_body(BodyDef::dynamic_at(Vec3::new(1.0, 0.0, 0.0)));
        let _shape_a = box_shape(&body_a);
        let _shape_b = box_shape(&body_b);

        let mut def = DistanceJointDef::new(&body_a, &body_b);
        def.length = 2.0;
        let joint = world.create_distance_joint(def);

        for _ in 0..30 {
            world.step(1.0 / 60.0, 4);
        }

        assert!((joint.current_length() - 2.0).abs() < 0.05);
    }

    #[test]
    fn typed_joint_setters_round_trip() {
        let world = World::new(Vec3::ZERO);
        let ground = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let body = world.create_body(BodyDef::dynamic_at(Vec3::new(0.0, 4.0, 0.0)));
        let _shape = box_shape(&body);

        {
            let mut def = RevoluteJointDef::new(&ground, &body);
            def.base.local_frame_a.p = Vec3::new(0.0, 4.0, 0.0);
            let joint = world.create_revolute_joint(def);
            assert_eq!(joint.joint_type(), JointType::Revolute);
            joint.set_limit_enabled(true);
            joint.set_limits(-1.0, 1.0);
            joint.set_motor_enabled(true);
            joint.set_motor_speed(2.0);
            joint.set_max_motor_torque(40.0);
            assert!(joint.is_limit_enabled());
            assert_eq!(joint.lower_limit(), -1.0);
            assert_eq!(joint.upper_limit(), 1.0);
            assert!(joint.is_motor_enabled());
            assert_eq!(joint.motor_speed(), 2.0);
            assert_eq!(joint.max_motor_torque(), 40.0);
        }

        {
            let mut def = SphericalJointDef::new(&ground, &body);
            def.base.local_frame_a.p = Vec3::new(0.0, 4.0, 0.0);
            let joint = world.create_spherical_joint(def);
            joint.set_cone_limit_enabled(true);
            joint.set_cone_limit(0.5);
            joint.set_motor_enabled(true);
            joint.set_motor_velocity(Vec3::new(0.1, 0.2, 0.3));
            assert!(joint.is_cone_limit_enabled());
            assert_eq!(joint.cone_limit(), 0.5);
            assert!(joint.is_motor_enabled());
            assert_eq!(joint.motor_velocity(), Vec3::new(0.1, 0.2, 0.3));
        }

        {
            let mut def = WheelJointDef::new(&ground, &body);
            def.base.local_frame_a.p = Vec3::new(0.0, 4.0, 0.0);
            let joint = world.create_wheel_joint(def);
            joint.set_suspension_hertz(5.0);
            joint.set_spin_motor_enabled(true);
            joint.set_spin_motor_speed(6.0);
            assert_eq!(joint.suspension_hertz(), 5.0);
            assert!(joint.is_spin_motor_enabled());
            assert_eq!(joint.spin_motor_speed(), 6.0);
            assert_eq!(joint.angular_separation(), None);
        }

        {
            let joint = world.create_filter_joint(FilterJointDef::new(&ground, &body));
            assert_eq!(joint.joint_type(), JointType::Filter);
            assert!(joint.is_valid());
        }
    }
}
