use box3d_sys as sys;

use crate::world::World;
use std::ffi::{c_char, c_void};

pub const MAX_WORKERS: u32 = sys::B3_MAX_WORKERS;

/// A Box3D task callback.
pub type TaskCallback = unsafe extern "C" fn(task_context: *mut c_void);

/// Enqueues a Box3D task on an external task system.
pub type EnqueueTaskCallback = unsafe extern "C" fn(
    task: Option<TaskCallback>,
    task_context: *mut c_void,
    user_context: *mut c_void,
    task_name: *const c_char,
) -> *mut c_void;

/// Waits for an externally enqueued Box3D task to finish.
pub type FinishTaskCallback =
    unsafe extern "C" fn(user_task: *mut c_void, user_context: *mut c_void);

/// Callbacks used to run Box3D work on an external worker pool.
///
/// The enqueue callback must invoke the supplied task exactly once. It may do
/// so synchronously and return a null handle, or return a handle that the
/// finish callback can block on.
#[derive(Clone, Copy)]
pub struct TaskSystem {
    pub enqueue_task: EnqueueTaskCallback,
    pub finish_task: FinishTaskCallback,
    pub user_context: *mut c_void,
}

impl TaskSystem {
    pub const fn new(
        enqueue_task: EnqueueTaskCallback,
        finish_task: FinishTaskCallback,
        user_context: *mut c_void,
    ) -> Self {
        Self {
            enqueue_task,
            finish_task,
            user_context,
        }
    }
}

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
