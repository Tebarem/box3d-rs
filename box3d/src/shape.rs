use std::{ffi::c_void, marker::PhantomData};

use box3d_sys as sys;

use crate::{
    body::Body,
    collision::{Capsule, ShapeCastOutput, Sphere},
    contact::ContactData,
    events::{BodyId, ShapeId, WorldId},
    handle,
    hull::HullRef,
    math::{Aabb, Filter, MassData, SurfaceMaterial, Vec3},
    mesh::Mesh,
    Error, Result,
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
    pub surface_material: Option<SurfaceMaterial>,
    pub filter: Filter,
    pub is_sensor: bool,
    pub enable_sensor_events: bool,
    pub enable_contact_events: bool,
    pub enable_hit_events: bool,
    pub enable_pre_solve_events: bool,
    pub enable_custom_filtering: bool,
    pub invoke_contact_creation: bool,
}

impl Default for ShapeDef {
    fn default() -> Self {
        let raw = unsafe { sys::b3DefaultShapeDef() };
        Self {
            density: raw.density,
            friction: raw.baseMaterial.friction,
            surface_material: None,
            filter: raw.filter.into(),
            is_sensor: raw.isSensor,
            enable_sensor_events: raw.enableSensorEvents,
            enable_contact_events: raw.enableContactEvents,
            enable_hit_events: raw.enableHitEvents,
            enable_pre_solve_events: raw.enablePreSolveEvents,
            enable_custom_filtering: raw.enableCustomFiltering,
            invoke_contact_creation: raw.invokeContactCreation,
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

    pub fn id(&self) -> ShapeId {
        ShapeId::from_raw(self.raw)
    }

    pub fn body_id(&self) -> BodyId {
        BodyId::from_raw(unsafe { sys::b3Shape_GetBody(self.raw) })
    }

    pub fn world_id(&self) -> WorldId {
        WorldId::from_raw(unsafe { sys::b3Shape_GetWorld(self.raw) })
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

    pub fn surface_material(&self) -> SurfaceMaterial {
        unsafe { sys::b3Shape_GetSurfaceMaterial(self.raw) }.into()
    }

    pub fn set_surface_material(&self, material: SurfaceMaterial) {
        assert_valid_surface_material(material);
        assert_ne!(self.shape_type(), ShapeType::Compound);
        unsafe { sys::b3Shape_SetSurfaceMaterial(self.raw, material.into()) };
    }

    pub fn mesh_material_count(&self) -> i32 {
        unsafe { sys::b3Shape_GetMeshMaterialCount(self.raw) }
    }

    pub fn set_mesh_material(&self, index: usize, material: SurfaceMaterial) -> Result<()> {
        assert_valid_surface_material(material);
        assert_ne!(self.shape_type(), ShapeType::Compound);
        let index = material_index(index, self.mesh_material_count())?;
        unsafe { sys::b3Shape_SetMeshMaterial(self.raw, material.into(), index) };
        Ok(())
    }

    pub fn mesh_surface_material(&self, index: usize) -> Result<SurfaceMaterial> {
        let index = material_index(index, self.mesh_material_count())?;
        Ok(unsafe { sys::b3Shape_GetMeshSurfaceMaterial(self.raw, index) }.into())
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

    pub fn set_user_data(&self, user_data: usize) {
        unsafe { sys::b3Shape_SetUserData(self.raw, user_data as *mut c_void) };
    }

    pub fn user_data(&self) -> usize {
        unsafe { sys::b3Shape_GetUserData(self.raw) as usize }
    }

    pub fn sphere(&self) -> Option<Sphere> {
        (self.shape_type() == ShapeType::Sphere)
            .then(|| unsafe { sys::b3Shape_GetSphere(self.raw) }.into())
    }

    pub fn capsule(&self) -> Option<Capsule> {
        (self.shape_type() == ShapeType::Capsule)
            .then(|| unsafe { sys::b3Shape_GetCapsule(self.raw) }.into())
    }

    pub fn set_sphere(&self, sphere: Sphere) {
        let raw = sphere.raw();
        unsafe { sys::b3Shape_SetSphere(self.raw, &raw) };
    }

    pub fn set_capsule(&self, capsule: Capsule) {
        let raw = capsule.raw();
        unsafe { sys::b3Shape_SetCapsule(self.raw, &raw) };
    }

    pub fn set_hull<'a>(&self, hull: impl Into<HullRef<'a>>) {
        unsafe { sys::b3Shape_SetHull(self.raw, hull.into().raw()) };
    }

    pub fn set_mesh(&self, mesh: &'body Mesh, scale: Vec3) {
        assert_valid_vec3(scale);
        unsafe { sys::b3Shape_SetMesh(self.raw, mesh.raw(), scale.into()) };
    }

    pub fn contact_capacity(&self) -> i32 {
        unsafe { sys::b3Shape_GetContactCapacity(self.raw) }
    }

    pub fn contact_data(&self) -> Vec<ContactData> {
        let capacity = self.contact_capacity();
        if capacity <= 0 {
            return Vec::new();
        }

        let mut raw = vec![sys::b3ContactData::default(); capacity as usize];
        let count = unsafe { sys::b3Shape_GetContactData(self.raw, raw.as_mut_ptr(), capacity) };
        raw.truncate(count.max(0) as usize);
        raw.into_iter().map(ContactData::from).collect()
    }

    pub fn sensor_capacity(&self) -> i32 {
        unsafe { sys::b3Shape_GetSensorCapacity(self.raw) }
    }

    pub fn sensor_ids(&self) -> Vec<ShapeId> {
        let capacity = self.sensor_capacity();
        if capacity <= 0 {
            return Vec::new();
        }

        let mut raw = vec![sys::b3ShapeId::default(); capacity as usize];
        let count = unsafe { sys::b3Shape_GetSensorData(self.raw, raw.as_mut_ptr(), capacity) };
        raw.truncate(count.max(0) as usize);
        raw.into_iter().map(ShapeId::from_raw).collect()
    }

    pub fn ray_cast(&self, origin: Vec3, translation: Vec3) -> Option<ShapeCastOutput> {
        assert_valid_vec3(origin);
        assert_valid_vec3(translation);
        ShapeCastOutput::from_raw(unsafe {
            sys::b3Shape_RayCast(self.raw, origin.into(), translation.into())
        })
    }

    pub fn compute_mass_data(&self) -> MassData {
        unsafe { sys::b3Shape_ComputeMassData(self.raw) }.into()
    }

    pub fn apply_wind(&self, wind: Vec3, drag: f32, lift: f32, max_speed: f32, wake: bool) {
        assert_valid_vec3(wind);
        assert!(drag.is_finite() && drag >= 0.0);
        assert!(lift.is_finite() && lift >= 0.0);
        assert!(max_speed.is_finite() && max_speed >= 0.0);
        unsafe { sys::b3Shape_ApplyWind(self.raw, wind.into(), drag, lift, max_speed, wake) };
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
    raw_def.baseMaterial = match def.surface_material {
        Some(material) => {
            assert_valid_surface_material(material);
            material.into()
        }
        None => {
            let mut material = unsafe { sys::b3DefaultSurfaceMaterial() };
            material.friction = def.friction;
            material
        }
    };
    raw_def.filter = def.filter.into();
    raw_def.isSensor = def.is_sensor;
    raw_def.enableSensorEvents = def.enable_sensor_events;
    raw_def.enableContactEvents = def.enable_contact_events;
    raw_def.enableHitEvents = def.enable_hit_events;
    raw_def.enablePreSolveEvents = def.enable_pre_solve_events;
    raw_def.enableCustomFiltering = def.enable_custom_filtering;
    raw_def.invokeContactCreation = def.invoke_contact_creation;
    raw_def
}

fn material_index(index: usize, count: i32) -> Result<i32> {
    if count <= 0 || index >= count as usize || index > i32::MAX as usize {
        Err(Error::InvalidInput)
    } else {
        Ok(index as i32)
    }
}

fn assert_valid_surface_material(material: SurfaceMaterial) {
    assert!(material.friction.is_finite() && material.friction >= 0.0);
    assert!(material.restitution.is_finite() && material.restitution >= 0.0);
    assert!(material.rolling_resistance.is_finite() && material.rolling_resistance >= 0.0);
    assert_valid_vec3(material.tangent_velocity);
}

fn assert_valid_vec3(value: Vec3) {
    assert!(value.x.is_finite() && value.y.is_finite() && value.z.is_finite());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, BoxHull, Mesh, SurfaceMaterial, World};

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
        let hull = BoxHull::new(Vec3::new(0.25, 0.25, 0.25));
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

        assert!(shape.id().is_valid());
        assert_eq!(shape.body_id(), body.id());
        assert_eq!(shape.world_id(), world.id());
        assert_eq!(shape.shape_type(), ShapeType::Sphere);
        assert_eq!(shape.sphere().unwrap().radius, 0.5);
        assert!(shape.capsule().is_none());
        let aabb = shape.aabb();
        assert!(aabb.lower_bound.x <= -0.5 && aabb.upper_bound.x >= 0.5);
        assert!((shape.closest_point(Vec3::new(2.0, 0.0, 0.0)).x - 0.5).abs() < 1e-6);

        shape.set_user_data(0x1234);
        assert_eq!(shape.user_data(), 0x1234);

        let material = SurfaceMaterial {
            friction: 0.45,
            restitution: 0.25,
            rolling_resistance: 0.05,
            tangent_velocity: Vec3::new(1.0, 0.0, 0.0),
            user_material_id: 7,
            custom_color: 0x112233,
        };
        shape.set_surface_material(material);
        assert_eq!(shape.surface_material(), material);

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

        let mass = shape.compute_mass_data();
        assert!(mass.mass > 0.0);
        let cast = shape
            .ray_cast(Vec3::new(-2.0, 0.0, 0.0), Vec3::new(4.0, 0.0, 0.0))
            .expect("ray should hit sphere");
        assert!(cast.fraction > 0.0 && cast.fraction < 1.0, "{cast:?}");
        shape.apply_wind(Vec3::new(1.0, 0.0, 0.0), 1.0, 0.0, 10.0, true);

        shape.set_sphere(Sphere::new(Vec3::ZERO, 0.75));
        assert_eq!(shape.sphere().unwrap().radius, 0.75);
        shape.set_capsule(Capsule::new(
            Vec3::new(0.0, -0.25, 0.0),
            Vec3::new(0.0, 0.25, 0.0),
            0.2,
        ));
        assert_eq!(shape.shape_type(), ShapeType::Capsule);
        assert!(shape.capsule().is_some());
        shape.set_hull(&hull);
        assert_eq!(shape.shape_type(), ShapeType::Hull);
    }

    #[test]
    fn shape_def_surface_material_applies_at_creation() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::dynamic_at(Vec3::ZERO));
        let material = SurfaceMaterial {
            friction: 0.75,
            restitution: 0.35,
            rolling_resistance: 0.03,
            ..SurfaceMaterial::default()
        };
        let shape = body.create_sphere(
            Vec3::ZERO,
            0.5,
            ShapeDef {
                density: 1.0,
                surface_material: Some(material),
                ..ShapeDef::default()
            },
        );

        assert_eq!(shape.surface_material(), material);
        assert_eq!(shape.friction(), material.friction);
        assert_eq!(shape.restitution(), material.restitution);
    }

    #[test]
    fn shape_def_default_matches_native_dynamic_density() {
        let def = ShapeDef::default();
        assert!(def.density > 0.0);
        assert_eq!(def.friction, SurfaceMaterial::default().friction);
        assert_eq!(def.surface_material, None);
        assert_eq!(def.filter, Filter::default());
        assert!(def.invoke_contact_creation);
    }

    #[test]
    fn mesh_materials_round_trip() {
        let world = World::new(Vec3::ZERO);
        let mesh = Mesh::box_mesh(Vec3::ZERO, Vec3::new(0.5, 0.5, 0.5), true);
        let body = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let shape = body.create_mesh(&mesh, Vec3::new(1.0, 1.0, 1.0), ShapeDef::default());
        let material = SurfaceMaterial {
            friction: 0.2,
            restitution: 0.1,
            rolling_resistance: 0.0,
            tangent_velocity: Vec3::new(0.0, 0.0, 0.0),
            user_material_id: 9,
            custom_color: 0x445566,
        };

        assert_eq!(shape.shape_type(), ShapeType::Mesh);
        assert_eq!(shape.mesh_material_count(), 1);
        shape.set_mesh_material(0, material).unwrap();
        assert_eq!(shape.mesh_surface_material(0).unwrap(), material);
        assert_eq!(
            shape.mesh_surface_material(1).err(),
            Some(Error::InvalidInput)
        );
        shape.set_mesh(&mesh, Vec3::new(-1.0, 1.0, 1.0));
        assert_eq!(shape.shape_type(), ShapeType::Mesh);
    }

    #[test]
    fn shape_contact_and_sensor_data_are_copied() {
        let world = World::default();
        let ground = world.create_body(BodyDef::static_at(Vec3::new(0.0, -0.5, 0.0)));
        let _ground_shape = ground.create_box(Vec3::new(10.0, 0.5, 10.0), ShapeDef::default());
        let body = world.create_body(BodyDef::dynamic_at(Vec3::new(0.0, 4.0, 0.0)));
        let shape = body.create_sphere(
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
            contacts = shape.contact_data();
            if !contacts.is_empty() {
                break;
            }
        }

        let contact = contacts.first().expect("shape should touch ground");
        assert!(contact.contact.is_valid());
        assert!(contact.shape_a.is_valid());
        assert!(contact.shape_b.is_valid());
        assert!(!contact.manifolds.is_empty());

        let sensor_body = world.create_body(BodyDef::static_at(Vec3::new(3.0, 0.0, 0.0)));
        let sensor = sensor_body.create_box(
            Vec3::new(1.0, 1.0, 1.0),
            ShapeDef {
                is_sensor: true,
                enable_sensor_events: true,
                ..ShapeDef::default()
            },
        );
        let visitor_body = world.create_body(BodyDef::dynamic_at(Vec3::new(3.0, 0.0, 0.0)));
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

        let mut visitors = Vec::new();
        for _ in 0..10 {
            world.step(1.0 / 60.0, 4);
            visitors = sensor.sensor_ids();
            if !visitors.is_empty() {
                break;
            }
        }

        assert!(visitors.iter().any(|id| id.is_valid()));
    }
}
