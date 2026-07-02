use std::marker::PhantomData;

use box3d_sys as sys;

use crate::{
    body::Body,
    handle,
    math::{Aabb, Filter, Vec3},
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ShapeType {
    Capsule,
    Compound,
    HeightField,
    Hull,
    Mesh,
    Sphere,
}

impl From<sys::b3ShapeType> for ShapeType {
    fn from(value: sys::b3ShapeType) -> Self {
        match value {
            sys::b3ShapeType_b3_capsuleShape => Self::Capsule,
            sys::b3ShapeType_b3_compoundShape => Self::Compound,
            sys::b3ShapeType_b3_heightShape => Self::HeightField,
            sys::b3ShapeType_b3_hullShape => Self::Hull,
            sys::b3ShapeType_b3_meshShape => Self::Mesh,
            sys::b3ShapeType_b3_sphereShape => Self::Sphere,
            _ => panic!("unknown box3d shape type {value}"),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ShapeDef {
    pub density: f32,
    pub friction: f32,
    pub is_sensor: bool,
    pub enable_sensor_events: bool,
    pub enable_contact_events: bool,
    pub enable_hit_events: bool,
    pub enable_pre_solve_events: bool,
}

impl Default for ShapeDef {
    fn default() -> Self {
        Self {
            density: 0.0,
            friction: 0.6,
            is_sensor: false,
            enable_sensor_events: false,
            enable_contact_events: false,
            enable_hit_events: false,
            enable_pre_solve_events: false,
        }
    }
}

pub struct Shape<'body> {
    raw: sys::b3ShapeId,
    _body: PhantomData<&'body ()>,
}

impl<'body> Shape<'body> {
    pub(crate) fn from_raw(raw: sys::b3ShapeId) -> Self {
        Self {
            raw,
            _body: PhantomData,
        }
    }

    pub fn is_valid(&self) -> bool {
        handle::is_shape_valid(self.raw)
    }

    pub fn shape_type(&self) -> ShapeType {
        unsafe { sys::b3Shape_GetType(self.raw) }.into()
    }

    pub fn is_sensor(&self) -> bool {
        unsafe { sys::b3Shape_IsSensor(self.raw) }
    }

    pub fn aabb(&self) -> Aabb {
        unsafe { sys::b3Shape_GetAABB(self.raw) }.into()
    }

    pub fn closest_point(&self, target: Vec3) -> Vec3 {
        unsafe { sys::b3Shape_GetClosestPoint(self.raw, target.into()) }.into()
    }

    pub fn density(&self) -> f32 {
        unsafe { sys::b3Shape_GetDensity(self.raw) }
    }

    pub fn set_density(&self, density: f32, update_body_mass: bool) {
        unsafe { sys::b3Shape_SetDensity(self.raw, density, update_body_mass) };
    }

    pub fn friction(&self) -> f32 {
        unsafe { sys::b3Shape_GetFriction(self.raw) }
    }

    pub fn set_friction(&self, friction: f32) {
        unsafe { sys::b3Shape_SetFriction(self.raw, friction) };
    }

    pub fn restitution(&self) -> f32 {
        unsafe { sys::b3Shape_GetRestitution(self.raw) }
    }

    pub fn set_restitution(&self, restitution: f32) {
        unsafe { sys::b3Shape_SetRestitution(self.raw, restitution) };
    }

    pub fn filter(&self) -> Filter {
        unsafe { sys::b3Shape_GetFilter(self.raw) }.into()
    }

    pub fn set_filter(&self, filter: Filter) {
        self.set_filter_with_contact_update(filter, false);
    }

    pub fn set_filter_with_contact_update(&self, filter: Filter, invoke_contacts: bool) {
        unsafe { sys::b3Shape_SetFilter(self.raw, filter.into(), invoke_contacts) };
    }

    pub fn enable_sensor_events(&self, enabled: bool) {
        unsafe { sys::b3Shape_EnableSensorEvents(self.raw, enabled) };
    }

    pub fn are_sensor_events_enabled(&self) -> bool {
        unsafe { sys::b3Shape_AreSensorEventsEnabled(self.raw) }
    }

    pub fn enable_contact_events(&self, enabled: bool) {
        unsafe { sys::b3Shape_EnableContactEvents(self.raw, enabled) };
    }

    pub fn are_contact_events_enabled(&self) -> bool {
        unsafe { sys::b3Shape_AreContactEventsEnabled(self.raw) }
    }

    pub fn enable_hit_events(&self, enabled: bool) {
        unsafe { sys::b3Shape_EnableHitEvents(self.raw, enabled) };
    }

    pub fn are_hit_events_enabled(&self) -> bool {
        unsafe { sys::b3Shape_AreHitEventsEnabled(self.raw) }
    }

    pub fn enable_pre_solve_events(&self, enabled: bool) {
        unsafe { sys::b3Shape_EnablePreSolveEvents(self.raw, enabled) };
    }

    pub fn are_pre_solve_events_enabled(&self) -> bool {
        unsafe { sys::b3Shape_ArePreSolveEventsEnabled(self.raw) }
    }
}

impl Drop for Shape<'_> {
    fn drop(&mut self) {
        handle::destroy_shape(self.raw, true);
    }
}

impl Body<'_> {
    pub fn create_sphere(&self, center: Vec3, radius: f32, def: ShapeDef) -> Shape<'_> {
        assert!(radius > 0.0);
        let raw_def = raw_shape_def(def);
        let sphere = sys::b3Sphere {
            center: center.into(),
            radius,
        };
        let raw = handle::shape(unsafe { sys::b3CreateSphereShape(self.raw(), &raw_def, &sphere) })
            .expect("box3d returned an invalid shape");

        Shape::from_raw(raw)
    }

    pub fn create_capsule(
        &self,
        point1: Vec3,
        point2: Vec3,
        radius: f32,
        def: ShapeDef,
    ) -> Shape<'_> {
        assert!(radius > 0.0);
        let raw_def = raw_shape_def(def);
        let capsule = sys::b3Capsule {
            center1: point1.into(),
            center2: point2.into(),
            radius,
        };
        let raw =
            handle::shape(unsafe { sys::b3CreateCapsuleShape(self.raw(), &raw_def, &capsule) })
                .expect("box3d returned an invalid shape");

        Shape::from_raw(raw)
    }
}

pub(crate) fn raw_shape_def(def: ShapeDef) -> sys::b3ShapeDef {
    let mut raw_def = unsafe { sys::b3DefaultShapeDef() };
    raw_def.density = def.density;
    raw_def.baseMaterial.friction = def.friction;
    raw_def.isSensor = def.is_sensor;
    raw_def.enableSensorEvents = def.enable_sensor_events;
    raw_def.enableContactEvents = def.enable_contact_events;
    raw_def.enableHitEvents = def.enable_hit_events;
    raw_def.enablePreSolveEvents = def.enable_pre_solve_events;
    raw_def
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, World};

    #[test]
    fn sphere_and_capsule_shapes_step() {
        let world = World::default();
        let ground = world.create_body(BodyDef::static_at(Vec3::new(0.0, -10.0, 0.0)));
        let _ground_shape = ground.create_box(Vec3::new(50.0, 10.0, 50.0), ShapeDef::default());

        let sphere = world.create_body(BodyDef::dynamic_at(Vec3::new(-1.0, 4.0, 0.0)));
        let sphere_shape = sphere.create_sphere(
            Vec3::ZERO,
            0.5,
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                ..ShapeDef::default()
            },
        );

        let capsule = world.create_body(BodyDef::dynamic_at(Vec3::new(1.0, 4.0, 0.0)));
        let capsule_shape = capsule.create_capsule(
            Vec3::new(0.0, -0.5, 0.0),
            Vec3::new(0.0, 0.5, 0.0),
            0.25,
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                ..ShapeDef::default()
            },
        );

        world.step(1.0 / 60.0, 4);

        assert!(sphere_shape.is_valid());
        assert!(capsule_shape.is_valid());
        assert!(sphere.position().y.is_finite());
        assert!(capsule.position().y.is_finite());
    }

    #[test]
    fn runtime_properties_round_trip() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let shape = body.create_sphere(
            Vec3::ZERO,
            0.5,
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                ..ShapeDef::default()
            },
        );

        assert_eq!(shape.shape_type(), ShapeType::Sphere);
        let aabb = shape.aabb();
        assert!(aabb.lower_bound.x <= -0.5 && aabb.upper_bound.x >= 0.5);
        assert!((shape.closest_point(Vec3::new(2.0, 0.0, 0.0)).x - 0.5).abs() < 1e-6);

        shape.set_density(2.0, true);
        shape.set_friction(0.4);
        shape.set_restitution(0.25);
        assert_eq!(shape.density(), 2.0);
        assert_eq!(shape.friction(), 0.4);
        assert_eq!(shape.restitution(), 0.25);

        let filter = Filter {
            category_bits: 0x2,
            mask_bits: 0x4,
            group_index: -3,
        };
        shape.set_filter(filter);
        assert_eq!(shape.filter(), filter);

        shape.enable_sensor_events(true);
        shape.enable_contact_events(true);
        shape.enable_hit_events(true);
        shape.enable_pre_solve_events(true);
        assert!(shape.are_sensor_events_enabled());
        assert!(shape.are_contact_events_enabled());
        assert!(shape.are_hit_events_enabled());
        assert!(shape.are_pre_solve_events_enabled());
    }
}
