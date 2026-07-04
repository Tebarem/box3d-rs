use std::{marker::PhantomData, slice};

use box3d_sys as sys;

use crate::{
    handle,
    math::{is_valid_transform, SurfaceMaterial, Transform, Vec3},
    mesh::Mesh,
    shape::{raw_shape_def, ShapeDef},
    world::World,
    Error, Result,
};

#[derive(Clone, Copy, Debug)]
pub struct WorldId {
    raw: sys::b3WorldId,
}

impl WorldId {
    pub(crate) fn from_raw(raw: sys::b3WorldId) -> Self {
        Self { raw }
    }

    pub const fn to_bits(self) -> u32 {
        ((self.raw.index1 as u32) << 16) | self.raw.generation as u32
    }

    pub const fn from_bits(bits: u32) -> Self {
        Self {
            raw: sys::b3WorldId {
                index1: (bits >> 16) as u16,
                generation: bits as u16,
            },
        }
    }

    pub fn is_valid(self) -> bool {
        handle::is_world_valid(self.raw)
    }
}

impl PartialEq for WorldId {
    fn eq(&self, other: &Self) -> bool {
        self.to_bits() == other.to_bits()
    }
}

impl Eq for WorldId {}

#[derive(Clone, Copy, Debug)]
pub struct BodyId {
    raw: sys::b3BodyId,
}

impl BodyId {
    pub(crate) fn from_raw(raw: sys::b3BodyId) -> Self {
        Self { raw }
    }

    pub const fn to_bits(self) -> u64 {
        ((self.raw.index1 as u64) << 32)
            | ((self.raw.world0 as u64) << 16)
            | self.raw.generation as u64
    }

    pub const fn from_bits(bits: u64) -> Self {
        Self {
            raw: sys::b3BodyId {
                index1: (bits >> 32) as i32,
                world0: (bits >> 16) as u16,
                generation: bits as u16,
            },
        }
    }

    pub fn is_valid(self) -> bool {
        handle::is_body_valid(self.raw)
    }

    pub fn destroy(self) {
        handle::destroy_body(self.raw);
    }

    pub fn wake(self) {
        assert!(self.is_valid());
        unsafe { sys::b3Body_SetAwake(self.raw, true) };
    }

    pub fn transform(self) -> Option<Transform> {
        self.is_valid()
            .then(|| unsafe { sys::b3Body_GetTransform(self.raw) }.into())
    }

    pub fn set_transform(self, position: Vec3, rotation: crate::Quat) {
        assert!(self.is_valid());
        unsafe { sys::b3Body_SetTransform(self.raw, position.into(), rotation.into()) };
    }

    pub fn set_linear_velocity(self, velocity: Vec3) {
        assert!(self.is_valid());
        unsafe { sys::b3Body_SetLinearVelocity(self.raw, velocity.into()) };
    }

    pub fn set_angular_velocity(self, velocity: Vec3) {
        assert!(self.is_valid());
        unsafe { sys::b3Body_SetAngularVelocity(self.raw, velocity.into()) };
    }

    pub fn set_linear_damping(self, damping: f32) {
        assert!(damping.is_finite() && damping >= 0.0);
        assert!(self.is_valid());
        unsafe { sys::b3Body_SetLinearDamping(self.raw, damping) };
    }

    pub fn set_angular_damping(self, damping: f32) {
        assert!(damping.is_finite() && damping >= 0.0);
        assert!(self.is_valid());
        unsafe { sys::b3Body_SetAngularDamping(self.raw, damping) };
    }

    pub fn set_sleep_threshold(self, threshold: f32) {
        assert!(threshold.is_finite() && threshold >= 0.0);
        assert!(self.is_valid());
        unsafe { sys::b3Body_SetSleepThreshold(self.raw, threshold) };
    }

    pub fn sleep_threshold(self) -> f32 {
        assert!(self.is_valid());
        unsafe { sys::b3Body_GetSleepThreshold(self.raw) }
    }

    pub fn create_box(self, half_extents: Vec3, def: ShapeDef) -> ShapeId {
        self.try_create_box(half_extents, def)
            .expect("box3d returned an invalid shape")
    }

    pub fn try_create_box(self, half_extents: Vec3, def: ShapeDef) -> Result<ShapeId> {
        assert!(self.is_valid());
        let raw_def = raw_shape_def(def);
        let hull = unsafe { sys::b3MakeBoxHull(half_extents.x, half_extents.y, half_extents.z) };
        handle::shape(unsafe { sys::b3CreateHullShape(self.raw, &raw_def, &hull.base) })
            .map(ShapeId::from_raw)
    }

    pub fn create_transformed_box(
        self,
        half_extents: Vec3,
        transform: Transform,
        def: ShapeDef,
    ) -> ShapeId {
        self.try_create_transformed_box(half_extents, transform, def)
            .expect("box3d returned an invalid shape")
    }

    pub fn try_create_transformed_box(
        self,
        half_extents: Vec3,
        transform: Transform,
        def: ShapeDef,
    ) -> Result<ShapeId> {
        assert!(self.is_valid());
        let raw_def = raw_shape_def(def);
        let hull = unsafe { sys::b3MakeBoxHull(half_extents.x, half_extents.y, half_extents.z) };
        handle::shape(unsafe {
            sys::b3CreateTransformedHullShape(
                self.raw,
                &raw_def,
                &hull.base,
                transform.into(),
                Vec3::new(1.0, 1.0, 1.0).into(),
            )
        })
        .map(ShapeId::from_raw)
    }

    pub fn create_sphere(self, center: Vec3, radius: f32, def: ShapeDef) -> ShapeId {
        self.try_create_sphere(center, radius, def)
            .expect("box3d returned an invalid shape")
    }

    pub fn try_create_sphere(self, center: Vec3, radius: f32, def: ShapeDef) -> Result<ShapeId> {
        assert!(self.is_valid());
        assert!(radius > 0.0);
        let raw_def = raw_shape_def(def);
        let sphere = sys::b3Sphere {
            center: center.into(),
            radius,
        };
        handle::shape(unsafe { sys::b3CreateSphereShape(self.raw, &raw_def, &sphere) })
            .map(ShapeId::from_raw)
    }

    pub fn create_mesh(self, mesh: &Mesh, scale: Vec3, def: ShapeDef) -> ShapeId {
        self.try_create_mesh(mesh, scale, def)
            .expect("box3d returned an invalid shape")
    }

    pub fn try_create_mesh(self, mesh: &Mesh, scale: Vec3, def: ShapeDef) -> Result<ShapeId> {
        assert!(self.is_valid());
        assert!(scale.x.is_finite() && scale.y.is_finite() && scale.z.is_finite());
        let raw_def = raw_shape_def(def);
        handle::shape(unsafe {
            sys::b3CreateMeshShape(self.raw, &raw_def, mesh.raw(), scale.into())
        })
        .map(ShapeId::from_raw)
    }
}

impl PartialEq for BodyId {
    fn eq(&self, other: &Self) -> bool {
        self.to_bits() == other.to_bits()
    }
}

impl Eq for BodyId {}

#[derive(Clone, Copy, Debug)]
pub struct ShapeId {
    raw: sys::b3ShapeId,
}

impl ShapeId {
    pub(crate) fn from_raw(raw: sys::b3ShapeId) -> Self {
        Self { raw }
    }

    pub const fn to_bits(self) -> u64 {
        ((self.raw.index1 as u64) << 32)
            | ((self.raw.world0 as u64) << 16)
            | self.raw.generation as u64
    }

    pub const fn from_bits(bits: u64) -> Self {
        Self {
            raw: sys::b3ShapeId {
                index1: (bits >> 32) as i32,
                world0: (bits >> 16) as u16,
                generation: bits as u16,
            },
        }
    }

    pub fn is_valid(self) -> bool {
        handle::is_shape_valid(self.raw)
    }

    pub fn destroy(self, update_body_mass: bool) {
        handle::destroy_shape(self.raw, update_body_mass);
    }

    pub fn set_surface_material(self, material: SurfaceMaterial) {
        assert!(self.is_valid());
        assert!(material.friction.is_finite() && material.friction >= 0.0);
        assert!(material.restitution.is_finite() && material.restitution >= 0.0);
        assert!(material.rolling_resistance.is_finite() && material.rolling_resistance >= 0.0);
        unsafe { sys::b3Shape_SetSurfaceMaterial(self.raw, material.into()) };
    }

    pub fn enable_contact_events(self, enabled: bool) {
        assert!(self.is_valid());
        unsafe { sys::b3Shape_EnableContactEvents(self.raw, enabled) };
    }

    pub fn enable_sensor_events(self, enabled: bool) {
        assert!(self.is_valid());
        unsafe { sys::b3Shape_EnableSensorEvents(self.raw, enabled) };
    }
}

impl PartialEq for ShapeId {
    fn eq(&self, other: &Self) -> bool {
        self.to_bits() == other.to_bits()
    }
}

impl Eq for ShapeId {}

#[derive(Clone, Copy, Debug)]
pub struct ContactId {
    raw: sys::b3ContactId,
}

impl ContactId {
    pub(crate) fn from_raw(raw: sys::b3ContactId) -> Self {
        Self { raw }
    }

    pub(crate) fn raw(self) -> sys::b3ContactId {
        self.raw
    }

    pub const fn to_bits(self) -> [u32; 3] {
        [
            self.raw.index1 as u32,
            self.raw.world0 as u32,
            self.raw.generation,
        ]
    }

    pub const fn from_bits(bits: [u32; 3]) -> Self {
        Self {
            raw: sys::b3ContactId {
                index1: bits[0] as i32,
                world0: bits[1] as u16,
                padding: 0,
                generation: bits[2],
            },
        }
    }

    pub fn is_valid(self) -> bool {
        handle::is_contact_valid(self.raw)
    }
}

impl PartialEq for ContactId {
    fn eq(&self, other: &Self) -> bool {
        self.to_bits() == other.to_bits()
    }
}

impl Eq for ContactId {}

#[derive(Clone, Copy, Debug)]
pub struct JointId {
    raw: sys::b3JointId,
}

impl JointId {
    pub(crate) fn from_raw(raw: sys::b3JointId) -> Self {
        Self { raw }
    }

    pub const fn to_bits(self) -> u64 {
        ((self.raw.index1 as u64) << 32)
            | ((self.raw.world0 as u64) << 16)
            | self.raw.generation as u64
    }

    pub const fn from_bits(bits: u64) -> Self {
        Self {
            raw: sys::b3JointId {
                index1: (bits >> 32) as i32,
                world0: (bits >> 16) as u16,
                generation: bits as u16,
            },
        }
    }

    pub fn is_valid(self) -> bool {
        handle::is_joint_valid(self.raw)
    }

    pub fn destroy(self, wake_attached: bool) {
        if self.is_valid() {
            unsafe { sys::b3DestroyJoint(self.raw, wake_attached) };
        }
    }

    pub fn set_wheel_spin_motor_speed(self, speed: f32) {
        assert!(speed.is_finite());
        assert!(self.is_valid());
        unsafe { sys::b3WheelJoint_SetSpinMotorSpeed(self.raw, speed) };
    }

    pub fn set_wheel_target_steering_angle(self, radians: f32) {
        assert!(radians.is_finite());
        assert!(self.is_valid());
        unsafe { sys::b3WheelJoint_SetTargetSteeringAngle(self.raw, radians) };
    }
}

impl PartialEq for JointId {
    fn eq(&self, other: &Self) -> bool {
        self.to_bits() == other.to_bits()
    }
}

impl Eq for JointId {}

#[derive(Clone, Copy, Debug)]
pub struct ParallelJointIdDef {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub local_frame_a: Transform,
    pub local_frame_b: Transform,
    pub collide_connected: bool,
    pub draw_scale: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_torque: f32,
}

impl ParallelJointIdDef {
    pub fn new(body_a: BodyId, body_b: BodyId) -> Self {
        let raw = unsafe { sys::b3DefaultParallelJointDef() };
        Self {
            body_a,
            body_b,
            local_frame_a: raw.base.localFrameA.into(),
            local_frame_b: raw.base.localFrameB.into(),
            collide_connected: raw.base.collideConnected,
            draw_scale: raw.base.drawScale,
            hertz: raw.hertz,
            damping_ratio: raw.dampingRatio,
            max_torque: raw.maxTorque,
        }
    }

    fn raw(self) -> Result<sys::b3ParallelJointDef> {
        if !self.body_a.is_valid()
            || !self.body_b.is_valid()
            || !is_valid_transform(self.local_frame_a)
            || !is_valid_transform(self.local_frame_b)
            || !is_non_negative_finite(self.draw_scale)
            || !is_non_negative_finite(self.hertz)
            || !is_non_negative_finite(self.damping_ratio)
            || !is_non_negative_finite(self.max_torque)
        {
            return Err(Error::InvalidInput);
        }

        let mut raw = unsafe { sys::b3DefaultParallelJointDef() };
        raw.base.bodyIdA = self.body_a.raw;
        raw.base.bodyIdB = self.body_b.raw;
        raw.base.localFrameA = self.local_frame_a.into();
        raw.base.localFrameB = self.local_frame_b.into();
        raw.base.collideConnected = self.collide_connected;
        raw.base.drawScale = self.draw_scale;
        raw.hertz = self.hertz;
        raw.dampingRatio = self.damping_ratio;
        raw.maxTorque = self.max_torque;
        Ok(raw)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct WheelJointIdDef {
    pub body_a: BodyId,
    pub body_b: BodyId,
    pub local_frame_a: Transform,
    pub local_frame_b: Transform,
    pub collide_connected: bool,
    pub draw_scale: f32,
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

impl WheelJointIdDef {
    pub fn new(body_a: BodyId, body_b: BodyId) -> Self {
        let raw = unsafe { sys::b3DefaultWheelJointDef() };
        Self {
            body_a,
            body_b,
            local_frame_a: raw.base.localFrameA.into(),
            local_frame_b: raw.base.localFrameB.into(),
            collide_connected: raw.base.collideConnected,
            draw_scale: raw.base.drawScale,
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

    fn raw(self) -> Result<sys::b3WheelJointDef> {
        if !self.body_a.is_valid()
            || !self.body_b.is_valid()
            || !is_valid_transform(self.local_frame_a)
            || !is_valid_transform(self.local_frame_b)
            || !self.draw_scale.is_finite()
            || self.draw_scale < 0.0
            || !is_non_negative_finite(self.suspension_hertz)
            || !is_non_negative_finite(self.suspension_damping_ratio)
            || !is_non_negative_finite(self.max_spin_torque)
            || !self.spin_speed.is_finite()
            || !is_non_negative_finite(self.steering_hertz)
            || !is_non_negative_finite(self.steering_damping_ratio)
            || !self.target_steering_angle.is_finite()
            || !is_non_negative_finite(self.max_steering_torque)
            || !finite_ordered(self.lower_suspension_limit, self.upper_suspension_limit)
            || !finite_ordered(self.lower_steering_limit, self.upper_steering_limit)
        {
            return Err(Error::InvalidInput);
        }

        let mut raw = unsafe { sys::b3DefaultWheelJointDef() };
        raw.base.bodyIdA = self.body_a.raw;
        raw.base.bodyIdB = self.body_b.raw;
        raw.base.localFrameA = self.local_frame_a.into();
        raw.base.localFrameB = self.local_frame_b.into();
        raw.base.collideConnected = self.collide_connected;
        raw.base.drawScale = self.draw_scale;
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

impl World {
    pub fn create_parallel_joint_id(&self, def: ParallelJointIdDef) -> JointId {
        self.try_create_parallel_joint_id(def)
            .expect("box3d returned an invalid parallel joint")
    }

    pub fn try_create_parallel_joint_id(&self, def: ParallelJointIdDef) -> Result<JointId> {
        let raw_def = def.raw()?;
        let raw = unsafe { sys::b3CreateParallelJoint(self.raw(), &raw_def) };
        valid_joint_id(raw)
    }

    pub fn create_wheel_joint_id(&self, def: WheelJointIdDef) -> JointId {
        self.try_create_wheel_joint_id(def)
            .expect("box3d returned an invalid wheel joint")
    }

    pub fn try_create_wheel_joint_id(&self, def: WheelJointIdDef) -> Result<JointId> {
        let raw_def = def.raw()?;
        let raw = unsafe { sys::b3CreateWheelJoint(self.raw(), &raw_def) };
        valid_joint_id(raw)
    }
}

fn valid_joint_id(raw: sys::b3JointId) -> Result<JointId> {
    if handle::is_joint_valid(raw) {
        Ok(JointId::from_raw(raw))
    } else {
        Err(Error::InvalidInput)
    }
}

fn is_non_negative_finite(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

fn finite_ordered(lower: f32, upper: f32) -> bool {
    lower.is_finite() && upper.is_finite() && lower <= upper
}

pub struct BodyEvents<'world> {
    raw: sys::b3BodyEvents,
    _world: PhantomData<&'world World>,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyMoveEvent {
    pub body: BodyId,
    pub user_data: usize,
    pub transform: Transform,
    pub fell_asleep: bool,
}

impl BodyEvents<'_> {
    pub fn moves(&self) -> impl Iterator<Item = BodyMoveEvent> + '_ {
        events(self.raw.moveEvents, self.raw.moveCount)
            .iter()
            .map(|event| BodyMoveEvent {
                body: BodyId::from_raw(event.bodyId),
                user_data: event.userData as usize,
                transform: event.transform.into(),
                fell_asleep: event.fellAsleep,
            })
    }
}

pub struct SensorEvents<'world> {
    raw: sys::b3SensorEvents,
    _world: PhantomData<&'world World>,
}

#[derive(Clone, Copy, Debug)]
pub struct SensorTouchEvent {
    pub sensor: ShapeId,
    pub visitor: ShapeId,
}

impl SensorEvents<'_> {
    pub fn begins(&self) -> impl Iterator<Item = SensorTouchEvent> + '_ {
        events(self.raw.beginEvents, self.raw.beginCount)
            .iter()
            .map(sensor_event)
    }

    pub fn ends(&self) -> impl Iterator<Item = SensorTouchEvent> + '_ {
        events(self.raw.endEvents, self.raw.endCount)
            .iter()
            .map(sensor_event)
    }
}

pub struct ContactEvents<'world> {
    raw: sys::b3ContactEvents,
    _world: PhantomData<&'world World>,
}

#[derive(Clone, Copy, Debug)]
pub struct ContactTouchEvent {
    pub shape_a: ShapeId,
    pub shape_b: ShapeId,
    pub contact: ContactId,
}

#[derive(Clone, Copy, Debug)]
pub struct ContactHitEvent {
    pub shape_a: ShapeId,
    pub shape_b: ShapeId,
    pub contact: ContactId,
    pub point: Vec3,
    pub normal: Vec3,
    pub approach_speed: f32,
    pub user_material_id_a: u64,
    pub user_material_id_b: u64,
}

impl ContactEvents<'_> {
    pub fn begins(&self) -> impl Iterator<Item = ContactTouchEvent> + '_ {
        events(self.raw.beginEvents, self.raw.beginCount)
            .iter()
            .map(contact_begin_event)
    }

    pub fn ends(&self) -> impl Iterator<Item = ContactTouchEvent> + '_ {
        events(self.raw.endEvents, self.raw.endCount)
            .iter()
            .map(contact_end_event)
    }

    pub fn hits(&self) -> impl Iterator<Item = ContactHitEvent> + '_ {
        events(self.raw.hitEvents, self.raw.hitCount)
            .iter()
            .map(|event| ContactHitEvent {
                shape_a: ShapeId::from_raw(event.shapeIdA),
                shape_b: ShapeId::from_raw(event.shapeIdB),
                contact: ContactId::from_raw(event.contactId),
                point: event.point.into(),
                normal: event.normal.into(),
                approach_speed: event.approachSpeed,
                user_material_id_a: event.userMaterialIdA,
                user_material_id_b: event.userMaterialIdB,
            })
    }
}

pub struct JointEvents<'world> {
    raw: sys::b3JointEvents,
    _world: PhantomData<&'world World>,
}

#[derive(Clone, Copy, Debug)]
pub struct JointEvent {
    pub joint: JointId,
    pub user_data: usize,
}

impl JointEvents<'_> {
    pub fn iter(&self) -> impl Iterator<Item = JointEvent> + '_ {
        events(self.raw.jointEvents, self.raw.count)
            .iter()
            .map(|event| JointEvent {
                joint: JointId::from_raw(event.jointId),
                user_data: event.userData as usize,
            })
    }
}

impl World {
    pub fn id(&self) -> WorldId {
        WorldId::from_raw(self.raw())
    }

    pub fn body_events(&self) -> BodyEvents<'_> {
        BodyEvents {
            raw: unsafe { sys::b3World_GetBodyEvents(self.raw()) },
            _world: PhantomData,
        }
    }

    pub fn sensor_events(&self) -> SensorEvents<'_> {
        SensorEvents {
            raw: unsafe { sys::b3World_GetSensorEvents(self.raw()) },
            _world: PhantomData,
        }
    }

    pub fn contact_events(&self) -> ContactEvents<'_> {
        ContactEvents {
            raw: unsafe { sys::b3World_GetContactEvents(self.raw()) },
            _world: PhantomData,
        }
    }

    pub fn joint_events(&self) -> JointEvents<'_> {
        JointEvents {
            raw: unsafe { sys::b3World_GetJointEvents(self.raw()) },
            _world: PhantomData,
        }
    }
}

fn events<'a, T>(events: *const T, count: i32) -> &'a [T] {
    if count <= 0 {
        &[]
    } else {
        unsafe { slice::from_raw_parts(events, count as usize) }
    }
}

fn sensor_event<T: SensorTouch>(event: &T) -> SensorTouchEvent {
    SensorTouchEvent {
        sensor: ShapeId::from_raw(event.sensor()),
        visitor: ShapeId::from_raw(event.visitor()),
    }
}

trait SensorTouch {
    fn sensor(&self) -> sys::b3ShapeId;
    fn visitor(&self) -> sys::b3ShapeId;
}

impl SensorTouch for sys::b3SensorBeginTouchEvent {
    fn sensor(&self) -> sys::b3ShapeId {
        self.sensorShapeId
    }

    fn visitor(&self) -> sys::b3ShapeId {
        self.visitorShapeId
    }
}

impl SensorTouch for sys::b3SensorEndTouchEvent {
    fn sensor(&self) -> sys::b3ShapeId {
        self.sensorShapeId
    }

    fn visitor(&self) -> sys::b3ShapeId {
        self.visitorShapeId
    }
}

fn contact_begin_event(event: &sys::b3ContactBeginTouchEvent) -> ContactTouchEvent {
    ContactTouchEvent {
        shape_a: ShapeId::from_raw(event.shapeIdA),
        shape_b: ShapeId::from_raw(event.shapeIdB),
        contact: ContactId::from_raw(event.contactId),
    }
}

fn contact_end_event(event: &sys::b3ContactEndTouchEvent) -> ContactTouchEvent {
    ContactTouchEvent {
        shape_a: ShapeId::from_raw(event.shapeIdA),
        shape_b: ShapeId::from_raw(event.shapeIdB),
        contact: ContactId::from_raw(event.contactId),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{compute_quat_between_unit_vectors, BodyCreateOptions, BodyDef, Mesh, ShapeDef};

    #[test]
    fn ids_round_trip_through_stable_bits() {
        let world_bits = 0x0012_0034;
        let body_bits = 0x0000_0042_0007_0011;
        let shape_bits = 0x0000_0043_0007_0012;
        let joint_bits = 0x0000_0044_0007_0013;
        let contact_bits = [42, 7, 99];

        let world = WorldId::from_bits(world_bits);
        assert_eq!(world.to_bits(), world_bits);
        assert_eq!(WorldId::from_bits(world.to_bits()), world);

        let body = BodyId::from_bits(body_bits);
        assert_eq!(body.to_bits(), body_bits);
        assert_eq!(BodyId::from_bits(body.to_bits()), body);

        let shape = ShapeId::from_bits(shape_bits);
        assert_eq!(shape.to_bits(), shape_bits);
        assert_eq!(ShapeId::from_bits(shape.to_bits()), shape);

        let joint = JointId::from_bits(joint_bits);
        assert_eq!(joint.to_bits(), joint_bits);
        assert_eq!(JointId::from_bits(joint.to_bits()), joint);

        let contact = ContactId::from_bits(contact_bits);
        assert_eq!(contact.to_bits(), contact_bits);
        assert_eq!(ContactId::from_bits(contact.to_bits()), contact);
    }

    #[test]
    fn world_id_reports_valid_world() {
        let world = World::default();
        let id = world.id();
        assert!(id.is_valid());
        assert_eq!(WorldId::from_bits(id.to_bits()), id);
    }

    #[test]
    fn body_events_report_moving_dynamic_body() {
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

        body.set_user_data(0x1234);
        body.set_linear_velocity(Vec3::new(1.0, 0.0, 0.0));
        world.step(1.0 / 60.0, 4);

        let moved = world.body_events().moves().any(|event| {
            event.body.is_valid()
                && event.user_data == 0x1234
                && event.transform.p.x > 0.0
                && !event.fell_asleep
        });
        assert!(moved);
    }

    #[test]
    fn body_id_methods_can_own_detached_body() {
        let world = World::new(Vec3::ZERO);
        let id = world.spawn_body(BodyDef::dynamic_at(Vec3::ZERO));
        let mesh = Mesh::box_mesh(Vec3::ZERO, Vec3::new(0.25, 0.25, 0.25), true);
        let shape = id.create_sphere(
            Vec3::ZERO,
            0.5,
            ShapeDef {
                density: 1.0,
                ..ShapeDef::default()
            },
        );
        let box_shape = id.create_transformed_box(
            Vec3::new(0.25, 0.25, 0.25),
            Transform::new(Vec3::new(0.25, 0.0, 0.0), crate::math::Quat::IDENTITY),
            ShapeDef {
                density: 1.0,
                ..ShapeDef::default()
            },
        );
        let mesh_shape = id.create_mesh(&mesh, Vec3::new(1.0, 1.0, 1.0), ShapeDef::default());

        id.set_linear_velocity(Vec3::new(1.0, 0.0, 0.0));
        world.step(1.0 / 60.0, 4);

        assert!(shape.is_valid());
        assert!(box_shape.is_valid());
        assert!(mesh_shape.is_valid());
        assert!(id.transform().unwrap().p.x > 0.0);

        id.destroy();
        assert!(!id.is_valid());
    }

    #[test]
    fn detached_joint_ids_attach_spawned_bodies() {
        let world = World::new(Vec3::ZERO);
        let ground = world.spawn_body(BodyDef::static_at(Vec3::ZERO));
        let chassis = world.spawn_body(BodyDef::dynamic_at(Vec3::new(0.0, 2.5, 0.0)));
        let wheel = world.spawn_body_with_options(
            BodyDef::dynamic_at(Vec3::new(1.5, 2.0, 0.8)),
            BodyCreateOptions {
                allow_fast_rotation: true,
            },
        );

        let _ground_shape = ground.create_box(Vec3::new(10.0, 0.5, 10.0), ShapeDef::default());
        let _chassis_shape = chassis.create_box(
            Vec3::new(2.0, 0.5, 1.0),
            ShapeDef {
                density: 0.5,
                ..ShapeDef::default()
            },
        );
        let _wheel_shape = wheel.create_sphere(
            Vec3::ZERO,
            0.4,
            ShapeDef {
                density: 2.0,
                friction: 3.0,
                ..ShapeDef::default()
            },
        );

        let upright_rotation =
            compute_quat_between_unit_vectors(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 1.0, 0.0));
        let mut upright_def = ParallelJointIdDef::new(ground, chassis);
        upright_def.local_frame_a.q = upright_rotation;
        upright_def.local_frame_b.q = upright_rotation;
        upright_def.collide_connected = true;
        upright_def.hertz = 0.5;
        upright_def.damping_ratio = 1.0;
        let upright = world.create_parallel_joint_id(upright_def);

        let mut wheel_def = WheelJointIdDef::new(chassis, wheel);
        wheel_def.local_frame_a = Transform::new(
            Vec3::new(1.5, -0.5, 0.8),
            compute_quat_between_unit_vectors(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
        );
        wheel_def.local_frame_b.q =
            compute_quat_between_unit_vectors(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 1.0, 0.0));
        wheel_def.enable_suspension_limit = true;
        wheel_def.lower_suspension_limit = -0.2;
        wheel_def.upper_suspension_limit = 0.2;
        wheel_def.enable_spin_motor = true;
        wheel_def.max_spin_torque = 5.0;
        wheel_def.enable_steering = true;
        wheel_def.steering_hertz = 10.0;
        wheel_def.steering_damping_ratio = 0.7;
        wheel_def.max_steering_torque = 5.0;
        wheel_def.enable_steering_limit = true;
        wheel_def.lower_steering_limit = -std::f32::consts::FRAC_PI_4;
        wheel_def.upper_steering_limit = std::f32::consts::FRAC_PI_4;

        let wheel_joint = world.create_wheel_joint_id(wheel_def);
        wheel_joint.set_wheel_target_steering_angle(0.1);
        wheel_joint.set_wheel_spin_motor_speed(-30.0);
        chassis.wake();
        world.step(1.0 / 60.0, 4);

        assert!(upright.is_valid());
        assert!(wheel_joint.is_valid());

        wheel_joint.destroy(true);
        upright.destroy(true);
        assert!(!wheel_joint.is_valid());
        assert!(!upright.is_valid());
    }

    #[test]
    fn contact_events_report_begin_touch() {
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
                enable_contact_events: true,
                ..ShapeDef::default()
            },
        );

        let mut saw_begin = false;
        for _ in 0..120 {
            world.step(1.0 / 60.0, 4);
            saw_begin |= world
                .contact_events()
                .begins()
                .any(|event| event.shape_a.is_valid() && event.shape_b.is_valid());
        }

        assert!(saw_begin);
    }

    #[test]
    fn sensor_events_report_begin_overlap() {
        let world = World::new(Vec3::ZERO);
        let sensor_body = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let _sensor = sensor_body.create_box(
            Vec3::new(1.0, 1.0, 1.0),
            ShapeDef {
                is_sensor: true,
                enable_sensor_events: true,
                ..ShapeDef::default()
            },
        );
        let visitor_body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let _visitor = visitor_body.create_sphere(
            Vec3::ZERO,
            0.25,
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                enable_sensor_events: true,
                ..ShapeDef::default()
            },
        );

        world.step(1.0 / 60.0, 4);

        assert!(world
            .sensor_events()
            .begins()
            .any(|event| event.sensor.is_valid() && event.visitor.is_valid()));
    }
}
