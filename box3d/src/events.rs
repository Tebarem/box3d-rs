use std::{marker::PhantomData, slice};

use box3d_sys as sys;

use crate::{
    handle,
    math::{Transform, Vec3},
    world::World,
};

#[derive(Clone, Copy, Debug)]
pub struct BodyId {
    raw: sys::b3BodyId,
}

impl BodyId {
    pub fn is_valid(self) -> bool {
        handle::is_body_valid(self.raw)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ShapeId {
    raw: sys::b3ShapeId,
}

impl ShapeId {
    pub fn is_valid(self) -> bool {
        handle::is_shape_valid(self.raw)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ContactId {
    raw: sys::b3ContactId,
}

impl ContactId {
    pub fn is_valid(self) -> bool {
        handle::is_contact_valid(self.raw)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct JointId {
    raw: sys::b3JointId,
}

impl JointId {
    pub fn is_valid(self) -> bool {
        handle::is_joint_valid(self.raw)
    }
}

pub struct BodyEvents<'world> {
    raw: sys::b3BodyEvents,
    _world: PhantomData<&'world World>,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyMoveEvent {
    pub body: BodyId,
    pub transform: Transform,
    pub fell_asleep: bool,
}

impl BodyEvents<'_> {
    pub fn moves(&self) -> impl Iterator<Item = BodyMoveEvent> + '_ {
        events(self.raw.moveEvents, self.raw.moveCount)
            .iter()
            .map(|event| BodyMoveEvent {
                body: BodyId { raw: event.bodyId },
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
                shape_a: ShapeId { raw: event.shapeIdA },
                shape_b: ShapeId { raw: event.shapeIdB },
                contact: ContactId {
                    raw: event.contactId,
                },
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
}

impl JointEvents<'_> {
    pub fn iter(&self) -> impl Iterator<Item = JointEvent> + '_ {
        events(self.raw.jointEvents, self.raw.count)
            .iter()
            .map(|event| JointEvent {
                joint: JointId { raw: event.jointId },
            })
    }
}

impl World {
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
        sensor: ShapeId {
            raw: event.sensor(),
        },
        visitor: ShapeId {
            raw: event.visitor(),
        },
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
        shape_a: ShapeId { raw: event.shapeIdA },
        shape_b: ShapeId { raw: event.shapeIdB },
        contact: ContactId {
            raw: event.contactId,
        },
    }
}

fn contact_end_event(event: &sys::b3ContactEndTouchEvent) -> ContactTouchEvent {
    ContactTouchEvent {
        shape_a: ShapeId { raw: event.shapeIdA },
        shape_b: ShapeId { raw: event.shapeIdB },
        contact: ContactId {
            raw: event.contactId,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, ShapeDef};

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

        body.set_linear_velocity(Vec3::new(1.0, 0.0, 0.0));
        world.step(1.0 / 60.0, 4);

        let moved = world.body_events().moves().any(|event| {
            event.body.is_valid() && event.transform.p.x > 0.0 && !event.fell_asleep
        });
        assert!(moved);
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
