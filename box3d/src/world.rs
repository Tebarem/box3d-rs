use std::{cell::Cell, marker::PhantomData};

use box3d_sys as sys;

use crate::{
    body::{Body, BodyDef, BodyType},
    handle,
    math::Vec3,
    Result,
};

pub struct World {
    raw: sys::b3WorldId,
    _not_sync: PhantomData<Cell<()>>,
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

    pub fn try_new(gravity: Vec3) -> Result<Self> {
        let mut def = unsafe { sys::b3DefaultWorldDef() };
        def.gravity = gravity.into();

        let raw = handle::create_world(&def)?;

        Ok(Self {
            raw,
            _not_sync: PhantomData,
        })
    }

    pub fn create_body(&self, def: BodyDef) -> Body<'_> {
        self.try_create_body(def)
            .expect("box3d returned an invalid body")
    }

    pub fn try_create_body(&self, def: BodyDef) -> Result<Body<'_>> {
        let mut raw_def = unsafe { sys::b3DefaultBodyDef() };
        raw_def.type_ = match def.body_type {
            BodyType::Static => sys::b3BodyType_b3_staticBody,
            BodyType::Kinematic => sys::b3BodyType_b3_kinematicBody,
            BodyType::Dynamic => sys::b3BodyType_b3_dynamicBody,
        };
        raw_def.position = def.position.into();

        let raw = handle::body(unsafe { sys::b3CreateBody(self.raw, &raw_def) })?;

        Ok(Body::from_raw(raw))
    }

    pub fn step(&self, time_step: f32, sub_step_count: i32) {
        unsafe { sys::b3World_Step(self.raw, time_step, sub_step_count) };
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

    pub fn counters(&self) -> Counters {
        unsafe { sys::b3World_GetCounters(self.raw) }.into()
    }

    pub fn profile(&self) -> Profile {
        unsafe { sys::b3World_GetProfile(self.raw) }.into()
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

impl Default for World {
    fn default() -> Self {
        Self::new(Vec3::new(0.0, -10.0, 0.0))
    }
}

impl Drop for World {
    fn drop(&mut self) {
        handle::destroy_world(self.raw);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let _ = world.profile();

        world.step(1.0 / 60.0, 4);
        let _ = world.counters();
        let _ = world.profile();
    }
}
