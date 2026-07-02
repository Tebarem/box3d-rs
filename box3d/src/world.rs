use std::{cell::Cell, ffi::c_void, marker::PhantomData, sync::Arc};

use box3d_sys as sys;

use crate::{
    body::{Body, BodyDef, BodyType},
    callbacks::CallbackState,
    handle,
    math::{Aabb, Vec3},
    Result,
};

pub struct World {
    raw: sys::b3WorldId,
    pub(crate) callbacks: Arc<CallbackState>,
    _not_sync: PhantomData<Cell<()>>,
}

pub fn world_count() -> i32 {
    unsafe { sys::b3GetWorldCount() }
}

pub fn max_world_count() -> i32 {
    unsafe { sys::b3GetMaxWorldCount() }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Capacity {
    pub static_shape_count: i32,
    pub dynamic_shape_count: i32,
    pub static_body_count: i32,
    pub dynamic_body_count: i32,
    pub contact_count: i32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactTuning {
    pub hertz: f32,
    pub damping_ratio: f32,
    pub contact_speed: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExplosionDef {
    pub mask_bits: u64,
    pub position: Vec3,
    pub radius: f32,
    pub falloff: f32,
    pub impulse_per_area: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Counters {
    pub body_count: i32,
    pub shape_count: i32,
    pub contact_count: i32,
    pub joint_count: i32,
    pub island_count: i32,
    pub stack_used: i32,
    pub arena_capacity: i32,
    pub static_tree_height: i32,
    pub tree_height: i32,
    pub sat_call_count: i32,
    pub sat_cache_hit_count: i32,
    pub byte_count: i32,
    pub task_count: i32,
    pub color_counts: [i32; 24],
    pub manifold_counts: [i32; 8],
    pub awake_contact_count: i32,
    pub recycled_contact_count: i32,
    pub distance_iterations: i32,
    pub push_back_iterations: i32,
    pub root_iterations: i32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Profile {
    pub step: f32,
    pub pairs: f32,
    pub collide: f32,
    pub solve: f32,
    pub solver_setup: f32,
    pub constraints: f32,
    pub prepare_constraints: f32,
    pub integrate_velocities: f32,
    pub warm_start: f32,
    pub solve_impulses: f32,
    pub integrate_positions: f32,
    pub relax_impulses: f32,
    pub apply_restitution: f32,
    pub store_impulses: f32,
    pub split_islands: f32,
    pub transforms: f32,
    pub sensor_hits: f32,
    pub joint_events: f32,
    pub hit_events: f32,
    pub refit: f32,
    pub bullets: f32,
    pub sleep_islands: f32,
    pub sensors: f32,
}

impl World {
    pub fn new(gravity: Vec3) -> Self {
        Self::try_new(gravity).expect("box3d returned an invalid world")
    }

    pub(crate) fn raw(&self) -> sys::b3WorldId {
        self.raw
    }

    pub fn try_new(gravity: Vec3) -> Result<Self> {
        let mut def = unsafe { sys::b3DefaultWorldDef() };
        def.gravity = gravity.into();

        let raw = handle::create_world(&def)?;

        Ok(Self {
            raw,
            callbacks: Self::callback_state(),
            _not_sync: PhantomData,
        })
    }

    pub fn create_body(&self, def: BodyDef) -> Body<'_> {
        self.try_create_body(def)
            .expect("box3d returned an invalid body")
    }

    pub fn try_create_body(&self, def: BodyDef) -> Result<Body<'_>> {
        let mut raw_def = unsafe { sys::b3DefaultBodyDef() };
        raw_def.type_ = raw_body_type(def.body_type);
        raw_def.position = def.position.into();

        let raw = handle::body(unsafe { sys::b3CreateBody(self.raw, &raw_def) })?;

        Ok(Body::from_raw(raw))
    }

    pub fn step(&self, time_step: f32, sub_step_count: i32) {
        unsafe { sys::b3World_Step(self.raw, time_step, sub_step_count) };
        self.resume_callback_panic();
    }

    pub fn gravity(&self) -> Vec3 {
        unsafe { sys::b3World_GetGravity(self.raw) }.into()
    }

    pub fn set_gravity(&self, gravity: Vec3) {
        unsafe { sys::b3World_SetGravity(self.raw, gravity.into()) };
    }

    pub fn set_sleeping_enabled(&self, enabled: bool) {
        unsafe { sys::b3World_EnableSleeping(self.raw, enabled) };
    }

    pub fn is_sleeping_enabled(&self) -> bool {
        unsafe { sys::b3World_IsSleepingEnabled(self.raw) }
    }

    pub fn set_continuous_enabled(&self, enabled: bool) {
        unsafe { sys::b3World_EnableContinuous(self.raw, enabled) };
    }

    pub fn is_continuous_enabled(&self) -> bool {
        unsafe { sys::b3World_IsContinuousEnabled(self.raw) }
    }

    pub fn awake_body_count(&self) -> i32 {
        unsafe { sys::b3World_GetAwakeBodyCount(self.raw) }
    }

    pub fn bounds(&self) -> Aabb {
        unsafe { sys::b3World_GetBounds(self.raw) }.into()
    }

    pub fn max_capacity(&self) -> Capacity {
        unsafe { sys::b3World_GetMaxCapacity(self.raw) }.into()
    }

    pub fn restitution_threshold(&self) -> f32 {
        unsafe { sys::b3World_GetRestitutionThreshold(self.raw) }
    }

    pub fn set_restitution_threshold(&self, value: f32) {
        assert!(value.is_finite() && value >= 0.0);
        unsafe { sys::b3World_SetRestitutionThreshold(self.raw, value) };
    }

    pub fn hit_event_threshold(&self) -> f32 {
        unsafe { sys::b3World_GetHitEventThreshold(self.raw) }
    }

    pub fn set_hit_event_threshold(&self, value: f32) {
        assert!(value.is_finite() && value >= 0.0);
        unsafe { sys::b3World_SetHitEventThreshold(self.raw, value) };
    }

    pub fn set_contact_tuning(&self, tuning: ContactTuning) {
        assert!(tuning.hertz.is_finite() && tuning.hertz >= 0.0);
        assert!(tuning.damping_ratio.is_finite() && tuning.damping_ratio >= 0.0);
        assert!(tuning.contact_speed.is_finite() && tuning.contact_speed >= 0.0);
        unsafe {
            sys::b3World_SetContactTuning(
                self.raw,
                tuning.hertz,
                tuning.damping_ratio,
                tuning.contact_speed,
            )
        };
    }

    pub fn contact_recycle_distance(&self) -> f32 {
        unsafe { sys::b3World_GetContactRecycleDistance(self.raw) }
    }

    pub fn set_contact_recycle_distance(&self, distance: f32) {
        assert!(distance.is_finite() && distance >= 0.0);
        unsafe { sys::b3World_SetContactRecycleDistance(self.raw, distance) };
    }

    pub fn maximum_linear_speed(&self) -> f32 {
        unsafe { sys::b3World_GetMaximumLinearSpeed(self.raw) }
    }

    pub fn set_maximum_linear_speed(&self, speed: f32) {
        assert!(speed.is_finite() && speed > 0.0);
        unsafe { sys::b3World_SetMaximumLinearSpeed(self.raw, speed) };
    }

    pub fn set_warm_starting_enabled(&self, enabled: bool) {
        unsafe { sys::b3World_EnableWarmStarting(self.raw, enabled) };
    }

    pub fn is_warm_starting_enabled(&self) -> bool {
        unsafe { sys::b3World_IsWarmStartingEnabled(self.raw) }
    }

    pub fn set_speculative_enabled(&self, enabled: bool) {
        unsafe { sys::b3World_EnableSpeculative(self.raw, enabled) };
    }

    pub fn set_user_data(&self, user_data: usize) {
        unsafe { sys::b3World_SetUserData(self.raw, user_data as *mut c_void) };
    }

    pub fn user_data(&self) -> usize {
        unsafe { sys::b3World_GetUserData(self.raw) as usize }
    }

    pub fn explode(&self, def: ExplosionDef) {
        assert!(
            def.position.x.is_finite() && def.position.y.is_finite() && def.position.z.is_finite()
        );
        assert!(def.radius.is_finite() && def.radius >= 0.0);
        assert!(def.falloff.is_finite() && def.falloff >= 0.0);
        assert!(def.impulse_per_area.is_finite());
        let raw = def.into();
        unsafe { sys::b3World_Explode(self.raw, &raw) };
    }

    pub fn rebuild_static_tree(&self) {
        unsafe { sys::b3World_RebuildStaticTree(self.raw) };
    }

    pub fn dump_memory_stats(&self) {
        unsafe { sys::b3World_DumpMemoryStats(self.raw) };
    }

    /// Writes `box3d_bounds.txt` in the current process working directory.
    pub fn dump_shape_bounds(&self, body_type: BodyType) {
        unsafe { sys::b3World_DumpShapeBounds(self.raw, raw_body_type(body_type)) };
    }

    /// Writes `box3d_dump.inl` in the current process working directory.
    pub fn dump_awake(&self) {
        unsafe { sys::b3World_DumpAwake(self.raw) };
    }

    /// Writes `box3d_dump.inl` and any native mesh dump files in the current process working directory.
    pub fn dump(&self) {
        unsafe { sys::b3World_Dump(self.raw) };
    }

    pub fn counters(&self) -> Counters {
        unsafe { sys::b3World_GetCounters(self.raw) }.into()
    }

    pub fn profile(&self) -> Profile {
        unsafe { sys::b3World_GetProfile(self.raw) }.into()
    }
}

impl From<sys::b3Capacity> for Capacity {
    fn from(value: sys::b3Capacity) -> Self {
        Self {
            static_shape_count: value.staticShapeCount,
            dynamic_shape_count: value.dynamicShapeCount,
            static_body_count: value.staticBodyCount,
            dynamic_body_count: value.dynamicBodyCount,
            contact_count: value.contactCount,
        }
    }
}

impl From<sys::b3ExplosionDef> for ExplosionDef {
    fn from(value: sys::b3ExplosionDef) -> Self {
        Self {
            mask_bits: value.maskBits,
            position: value.position.into(),
            radius: value.radius,
            falloff: value.falloff,
            impulse_per_area: value.impulsePerArea,
        }
    }
}

impl From<ExplosionDef> for sys::b3ExplosionDef {
    fn from(value: ExplosionDef) -> Self {
        Self {
            maskBits: value.mask_bits,
            position: value.position.into(),
            radius: value.radius,
            falloff: value.falloff,
            impulsePerArea: value.impulse_per_area,
        }
    }
}

impl Default for ExplosionDef {
    fn default() -> Self {
        unsafe { sys::b3DefaultExplosionDef() }.into()
    }
}

impl From<sys::b3Counters> for Counters {
    fn from(value: sys::b3Counters) -> Self {
        Self {
            body_count: value.bodyCount,
            shape_count: value.shapeCount,
            contact_count: value.contactCount,
            joint_count: value.jointCount,
            island_count: value.islandCount,
            stack_used: value.stackUsed,
            arena_capacity: value.arenaCapacity,
            static_tree_height: value.staticTreeHeight,
            tree_height: value.treeHeight,
            sat_call_count: value.satCallCount,
            sat_cache_hit_count: value.satCacheHitCount,
            byte_count: value.byteCount,
            task_count: value.taskCount,
            color_counts: value.colorCounts,
            manifold_counts: value.manifoldCounts,
            awake_contact_count: value.awakeContactCount,
            recycled_contact_count: value.recycledContactCount,
            distance_iterations: value.distanceIterations,
            push_back_iterations: value.pushBackIterations,
            root_iterations: value.rootIterations,
        }
    }
}

impl From<sys::b3Profile> for Profile {
    fn from(value: sys::b3Profile) -> Self {
        Self {
            step: value.step,
            pairs: value.pairs,
            collide: value.collide,
            solve: value.solve,
            solver_setup: value.solverSetup,
            constraints: value.constraints,
            prepare_constraints: value.prepareConstraints,
            integrate_velocities: value.integrateVelocities,
            warm_start: value.warmStart,
            solve_impulses: value.solveImpulses,
            integrate_positions: value.integratePositions,
            relax_impulses: value.relaxImpulses,
            apply_restitution: value.applyRestitution,
            store_impulses: value.storeImpulses,
            split_islands: value.splitIslands,
            transforms: value.transforms,
            sensor_hits: value.sensorHits,
            joint_events: value.jointEvents,
            hit_events: value.hitEvents,
            refit: value.refit,
            bullets: value.bullets,
            sleep_islands: value.sleepIslands,
            sensors: value.sensors,
        }
    }
}

fn raw_body_type(body_type: BodyType) -> sys::b3BodyType {
    match body_type {
        BodyType::Static => sys::b3BodyType_b3_staticBody,
        BodyType::Kinematic => sys::b3BodyType_b3_kinematicBody,
        BodyType::Dynamic => sys::b3BodyType_b3_dynamicBody,
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new(Vec3::new(0.0, -10.0, 0.0))
    }
}

impl Drop for World {
    fn drop(&mut self) {
        unsafe {
            sys::b3World_SetCustomFilterCallback(self.raw, None, std::ptr::null_mut());
            sys::b3World_SetPreSolveCallback(self.raw, None, std::ptr::null_mut());
        }
        handle::destroy_world(self.raw);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        fs,
        path::{Path, PathBuf},
        sync::Mutex,
    };

    static CWD_LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn settings_and_diagnostics_work_on_empty_world() {
        let world = World::default();
        let gravity = Vec3::new(1.0, -2.0, 3.0);

        world.set_gravity(gravity);
        assert_eq!(world.gravity(), gravity);

        world.set_sleeping_enabled(false);
        assert!(!world.is_sleeping_enabled());
        world.set_sleeping_enabled(true);
        assert!(world.is_sleeping_enabled());

        world.set_continuous_enabled(false);
        assert!(!world.is_continuous_enabled());
        world.set_continuous_enabled(true);
        assert!(world.is_continuous_enabled());

        assert_eq!(world.awake_body_count(), 0);
        assert_eq!(world.counters().body_count, 0);
        assert!(world_count() >= 1);
        assert!(max_world_count() >= world_count());
        assert_eq!(world.max_capacity(), Capacity::default());

        let bounds = world.bounds();
        assert!(bounds.lower_bound.x <= bounds.upper_bound.x);
        assert!(bounds.lower_bound.y <= bounds.upper_bound.y);
        assert!(bounds.lower_bound.z <= bounds.upper_bound.z);

        world.set_restitution_threshold(1.25);
        assert_eq!(world.restitution_threshold(), 1.25);

        world.set_hit_event_threshold(2.5);
        assert_eq!(world.hit_event_threshold(), 2.5);

        world.set_contact_tuning(ContactTuning {
            hertz: 30.0,
            damping_ratio: 0.8,
            contact_speed: 2.0,
        });

        world.set_contact_recycle_distance(0.25);
        assert_eq!(world.contact_recycle_distance(), 0.25);

        world.set_maximum_linear_speed(60.0);
        assert_eq!(world.maximum_linear_speed(), 60.0);

        world.set_warm_starting_enabled(false);
        assert!(!world.is_warm_starting_enabled());
        world.set_warm_starting_enabled(true);
        assert!(world.is_warm_starting_enabled());

        world.set_speculative_enabled(false);
        world.set_speculative_enabled(true);

        world.set_user_data(0x1234);
        assert_eq!(world.user_data(), 0x1234);

        world.explode(ExplosionDef {
            radius: 0.0,
            falloff: 0.0,
            impulse_per_area: 0.0,
            ..ExplosionDef::default()
        });

        world.rebuild_static_tree();
        world.dump_memory_stats();

        let _ = world.profile();

        world.step(1.0 / 60.0, 4);
        let _ = world.counters();
        let _ = world.profile();
    }

    #[test]
    fn dump_diagnostics_write_native_files_in_current_directory() {
        let _guard = CWD_LOCK.lock().expect("world dump cwd mutex poisoned");
        let world = World::default();
        let dir = temp_dump_dir();
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).expect("create dump temp dir");

        let cwd = CurrentDirGuard::enter(&dir);

        world.dump_shape_bounds(BodyType::Static);
        assert!(Path::new("box3d_bounds.txt").exists());

        world.dump_awake();
        assert!(Path::new("box3d_dump.inl").exists());

        fs::remove_file("box3d_dump.inl").expect("remove awake dump");
        world.dump();
        assert!(Path::new("box3d_dump.inl").exists());

        drop(cwd);
        fs::remove_dir_all(&dir).expect("remove dump temp dir");
    }

    fn temp_dump_dir() -> PathBuf {
        std::env::temp_dir().join(format!("box3d-rs-world-dumps-{}", std::process::id()))
    }

    struct CurrentDirGuard {
        previous: PathBuf,
    }

    impl CurrentDirGuard {
        fn enter(path: &Path) -> Self {
            let previous = std::env::current_dir().expect("read current dir");
            std::env::set_current_dir(path).expect("enter dump temp dir");
            Self { previous }
        }
    }

    impl Drop for CurrentDirGuard {
        fn drop(&mut self) {
            let _ = std::env::set_current_dir(&self.previous);
        }
    }
}
