use std::{ffi::c_void, ptr::NonNull, slice};

use box3d_sys as sys;

use crate::{
    math::{Aabb, Vec3},
    world::World,
    Error, Result,
};

pub struct Recording {
    raw: NonNull<sys::b3Recording>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecPlayerInfo {
    pub frame_count: i32,
    pub worker_count: i32,
    pub time_step: f32,
    pub sub_step_count: i32,
    pub length_scale: f32,
    pub bounds: Aabb,
}

pub struct RecPlayer {
    raw: NonNull<sys::b3RecPlayer>,
}

impl Recording {
    pub fn new() -> Self {
        Self::try_new(0).expect("box3d returned a null recording")
    }

    pub fn try_new(byte_capacity: i32) -> Result<Self> {
        if byte_capacity < 0 {
            return Err(Error::InvalidInput);
        }

        let raw =
            NonNull::new(unsafe { sys::b3CreateRecording(byte_capacity) }).ok_or(Error::Null)?;
        Ok(Self { raw })
    }

    pub fn bytes(&self) -> &[u8] {
        let size = unsafe { sys::b3Recording_GetSize(self.raw.as_ptr()) };
        if size <= 0 {
            return &[];
        }

        let data = unsafe { sys::b3Recording_GetData(self.raw.as_ptr()) };
        if data.is_null() {
            &[]
        } else {
            unsafe { slice::from_raw_parts(data, size as usize) }
        }
    }

    pub fn validate_replay(&self, worker_count: u32) -> bool {
        validate_replay(self.bytes(), worker_count)
    }
}

impl Default for Recording {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for Recording {
    fn drop(&mut self) {
        unsafe { sys::b3DestroyRecording(self.raw.as_ptr()) };
    }
}

impl RecPlayer {
    pub fn new(bytes: &[u8], worker_count: u32) -> Self {
        Self::try_new(bytes, worker_count).expect("box3d returned a null recording player")
    }

    pub fn try_new(bytes: &[u8], worker_count: u32) -> Result<Self> {
        let size = i32::try_from(bytes.len()).map_err(|_| Error::InvalidInput)?;
        let raw = unsafe {
            sys::b3RecPlayer_Create(
                bytes.as_ptr().cast::<c_void>(),
                size,
                worker_count_i32(worker_count),
            )
        };
        let raw = NonNull::new(raw).ok_or(Error::InvalidInput)?;
        Ok(Self { raw })
    }

    pub fn step_frame(&mut self) -> bool {
        unsafe { sys::b3RecPlayer_StepFrame(self.raw.as_ptr()) }
    }

    pub fn restart(&mut self) {
        unsafe { sys::b3RecPlayer_Restart(self.raw.as_ptr()) };
    }

    pub fn seek_frame(&mut self, target_frame: i32) {
        unsafe { sys::b3RecPlayer_SeekFrame(self.raw.as_ptr(), target_frame) };
    }

    pub fn frame(&self) -> i32 {
        unsafe { sys::b3RecPlayer_GetFrame(self.raw.as_ptr()) }
    }

    pub fn frame_count(&self) -> i32 {
        unsafe { sys::b3RecPlayer_GetFrameCount(self.raw.as_ptr()) }
    }

    pub fn is_at_end(&self) -> bool {
        unsafe { sys::b3RecPlayer_IsAtEnd(self.raw.as_ptr()) }
    }

    pub fn has_diverged(&self) -> bool {
        unsafe { sys::b3RecPlayer_HasDiverged(self.raw.as_ptr()) }
    }

    pub fn diverge_frame(&self) -> i32 {
        unsafe { sys::b3RecPlayer_GetDivergeFrame(self.raw.as_ptr()) }
    }

    pub fn info(&self) -> RecPlayerInfo {
        unsafe { sys::b3RecPlayer_GetInfo(self.raw.as_ptr()) }.into()
    }

    pub fn set_worker_count(&mut self, count: u32) {
        unsafe { sys::b3RecPlayer_SetWorkerCount(self.raw.as_ptr(), worker_count_i32(count)) };
    }
}

impl Drop for RecPlayer {
    fn drop(&mut self) {
        unsafe { sys::b3RecPlayer_Destroy(self.raw.as_ptr()) };
    }
}

impl From<sys::b3RecPlayerInfo> for RecPlayerInfo {
    fn from(value: sys::b3RecPlayerInfo) -> Self {
        Self {
            frame_count: value.frameCount,
            worker_count: value.workerCount,
            time_step: value.timeStep,
            sub_step_count: value.subStepCount,
            length_scale: value.lengthScale,
            bounds: value.bounds.into(),
        }
    }
}

impl World {
    pub fn start_recording(&self, recording: &mut Recording) {
        unsafe { sys::b3World_StartRecording(self.raw(), recording.raw.as_ptr()) };
    }

    pub fn stop_recording(&self) {
        unsafe { sys::b3World_StopRecording(self.raw()) };
    }
}

pub fn validate_replay(bytes: &[u8], worker_count: u32) -> bool {
    let Ok(size) = i32::try_from(bytes.len()) else {
        return false;
    };

    unsafe {
        sys::b3ValidateReplay(
            bytes.as_ptr().cast::<c_void>(),
            size,
            worker_count_i32(worker_count),
        )
    }
}

fn worker_count_i32(count: u32) -> i32 {
    i32::try_from(count).unwrap_or(i32::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, ShapeDef};

    #[test]
    fn records_and_replays_tiny_world() {
        let mut recording = Recording::new();
        let world = World::default();

        world.start_recording(&mut recording);

        let ground = world.create_body(BodyDef::static_at(Vec3::new(0.0, -0.5, 0.0)));
        let _ground_shape = ground.create_box(Vec3::new(10.0, 0.5, 10.0), ShapeDef::default());
        let body = world.create_body(BodyDef::dynamic_at(Vec3::new(0.0, 4.0, 0.0)));
        let _shape = body.create_sphere(
            Vec3::ZERO,
            0.5,
            ShapeDef {
                density: 1.0,
                ..ShapeDef::default()
            },
        );

        for _ in 0..2 {
            world.step(1.0 / 60.0, 4);
        }

        world.stop_recording();

        let bytes = recording.bytes();
        assert!(!bytes.is_empty());
        assert!(recording.validate_replay(1));

        let mut player = RecPlayer::try_new(bytes, 1).unwrap();
        assert_eq!(player.frame_count(), 2);
        assert_eq!(player.info().frame_count, 2);

        while player.step_frame() {}

        assert!(player.is_at_end());
        assert!(!player.has_diverged(), "{}", player.diverge_frame());

        player.restart();
        assert_eq!(player.frame(), 0);
        player.seek_frame(1);
        assert_eq!(player.frame(), 1);
    }
}
