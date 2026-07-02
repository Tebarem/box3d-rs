use box3d_sys as sys;

use crate::world::World;

pub const MAX_WORKERS: u32 = sys::B3_MAX_WORKERS;

impl World {
    pub fn set_worker_count(&self, count: u32) {
        let count = i32::try_from(count).unwrap_or(i32::MAX);
        unsafe { sys::b3World_SetWorkerCount(self.raw(), count) };
    }

    pub fn worker_count(&self) -> u32 {
        unsafe { sys::b3World_GetWorkerCount(self.raw()) as u32 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn worker_count_round_trips_and_clamps() {
        let world = World::default();

        assert_eq!(world.worker_count(), 1);

        world.set_worker_count(2);
        assert_eq!(world.worker_count(), 2);

        world.set_worker_count(0);
        assert_eq!(world.worker_count(), 1);

        world.set_worker_count(MAX_WORKERS + 10);
        assert_eq!(world.worker_count(), MAX_WORKERS);
    }
}
