use std::{
    ffi::{c_void, CStr, CString},
    path::Path,
    ptr::NonNull,
    slice,
};

use box3d_sys as sys;

use crate::{
    debug_draw::{with_raw_debug_draw, DebugDraw},
    events::{BodyId, ShapeId, WorldId},
    math::{Aabb, Vec3},
    query::QueryFilter,
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RecQueryType {
    OverlapAabb,
    OverlapShape,
    CastRay,
    CastShape,
    CastRayClosest,
    CastMover,
    CollideMover,
    Unknown(i32),
}

#[derive(Clone, Debug, PartialEq)]
pub struct RecQueryInfo {
    pub query_type: RecQueryType,
    pub filter: QueryFilter,
    pub aabb: Aabb,
    pub origin: Vec3,
    pub translation: Vec3,
    pub hit_count: i32,
    pub key: u64,
    pub id: u64,
    pub name: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecQueryHit {
    pub shape: ShapeId,
    pub point: Vec3,
    pub normal: Vec3,
    pub fraction: f32,
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

    pub fn save_to_file(&self, path: impl AsRef<Path>) -> Result<()> {
        let path = path_to_cstring(path)?;
        if unsafe { sys::b3SaveRecordingToFile(self.raw.as_ptr(), path.as_ptr()) } {
            Ok(())
        } else {
            Err(Error::InvalidInput)
        }
    }

    pub fn load_from_file(path: impl AsRef<Path>) -> Result<Self> {
        let path = path_to_cstring(path)?;
        let raw = unsafe { sys::b3LoadRecordingFromFile(path.as_ptr()) };
        let raw = NonNull::new(raw).ok_or(Error::InvalidInput)?;
        Ok(Self { raw })
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

    pub fn world_id(&self) -> WorldId {
        WorldId::from_raw(unsafe { sys::b3RecPlayer_GetWorldId(self.raw.as_ptr()) })
    }

    pub fn body_count(&self) -> i32 {
        unsafe { sys::b3RecPlayer_GetBodyCount(self.raw.as_ptr()) }
    }

    pub fn body_id(&self, index: usize) -> Option<BodyId> {
        let index = i32::try_from(index).ok()?;
        let body =
            BodyId::from_raw(unsafe { sys::b3RecPlayer_GetBodyId(self.raw.as_ptr(), index) });
        body.is_valid().then_some(body)
    }

    pub fn set_worker_count(&mut self, count: u32) {
        unsafe { sys::b3RecPlayer_SetWorkerCount(self.raw.as_ptr(), worker_count_i32(count)) };
    }

    pub fn set_keyframe_policy(&mut self, budget_bytes: usize, min_interval_frames: u32) {
        unsafe {
            sys::b3RecPlayer_SetKeyframePolicy(
                self.raw.as_ptr(),
                budget_bytes,
                worker_count_i32(min_interval_frames),
            )
        };
    }

    pub fn keyframe_budget(&self) -> usize {
        unsafe { sys::b3RecPlayer_GetKeyframeBudget(self.raw.as_ptr()) }
    }

    pub fn keyframe_min_interval(&self) -> i32 {
        unsafe { sys::b3RecPlayer_GetKeyframeMinInterval(self.raw.as_ptr()) }
    }

    pub fn keyframe_interval(&self) -> i32 {
        unsafe { sys::b3RecPlayer_GetKeyframeInterval(self.raw.as_ptr()) }
    }

    pub fn keyframe_bytes(&self) -> usize {
        unsafe { sys::b3RecPlayer_GetKeyframeBytes(self.raw.as_ptr()) }
    }

    pub fn frame_query_count(&self) -> i32 {
        unsafe { sys::b3RecPlayer_GetFrameQueryCount(self.raw.as_ptr()) }
    }

    pub fn frame_query(&self, index: usize) -> Option<RecQueryInfo> {
        let index = query_index(index, self.frame_query_count())?;
        Some(unsafe { sys::b3RecPlayer_GetFrameQuery(self.raw.as_ptr(), index) }.into())
    }

    pub fn frame_query_hit(&self, query_index: usize, hit_index: usize) -> Option<RecQueryHit> {
        let query = self.frame_query(query_index)?;
        let hit_index = self::query_index(hit_index, query.hit_count)?;
        Some(
            unsafe {
                sys::b3RecPlayer_GetFrameQueryHit(self.raw.as_ptr(), query_index as i32, hit_index)
            }
            .into(),
        )
    }

    pub fn draw_frame_queries<D: DebugDraw>(
        &mut self,
        draw: &mut D,
        query_index: Option<usize>,
        selected_index: Option<usize>,
    ) {
        let query_index = optional_i32(query_index);
        let selected_index = optional_i32(selected_index);
        with_raw_debug_draw(draw, |raw| unsafe {
            sys::b3RecPlayer_DrawFrameQueries(self.raw.as_ptr(), raw, query_index, selected_index)
        });
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

impl From<sys::b3RecQueryType> for RecQueryType {
    fn from(value: sys::b3RecQueryType) -> Self {
        match value {
            sys::b3RecQueryType_b3_recQueryOverlapAABB => Self::OverlapAabb,
            sys::b3RecQueryType_b3_recQueryOverlapShape => Self::OverlapShape,
            sys::b3RecQueryType_b3_recQueryCastRay => Self::CastRay,
            sys::b3RecQueryType_b3_recQueryCastShape => Self::CastShape,
            sys::b3RecQueryType_b3_recQueryCastRayClosest => Self::CastRayClosest,
            sys::b3RecQueryType_b3_recQueryCastMover => Self::CastMover,
            sys::b3RecQueryType_b3_recQueryCollideMover => Self::CollideMover,
            _ => Self::Unknown(value),
        }
    }
}

impl From<sys::b3RecQueryInfo> for RecQueryInfo {
    fn from(value: sys::b3RecQueryInfo) -> Self {
        let name = if value.name.is_null() {
            None
        } else {
            Some(
                unsafe { CStr::from_ptr(value.name) }
                    .to_string_lossy()
                    .into_owned(),
            )
        };
        Self {
            query_type: value.type_.into(),
            filter: value.filter.into(),
            aabb: value.aabb.into(),
            origin: value.origin.into(),
            translation: value.translation.into(),
            hit_count: value.hitCount,
            key: value.key,
            id: value.id,
            name,
        }
    }
}

impl From<sys::b3RecQueryHit> for RecQueryHit {
    fn from(value: sys::b3RecQueryHit) -> Self {
        Self {
            shape: ShapeId::from_raw(value.shape),
            point: value.point.into(),
            normal: value.normal.into(),
            fraction: value.fraction,
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

fn query_index(index: usize, count: i32) -> Option<i32> {
    (count > 0 && index < count as usize)
        .then(|| i32::try_from(index).ok())
        .flatten()
}

fn optional_i32(index: Option<usize>) -> i32 {
    index
        .map(|index| i32::try_from(index).expect("index exceeds i32::MAX"))
        .unwrap_or(-1)
}

fn path_to_cstring(path: impl AsRef<Path>) -> Result<CString> {
    let path = path.as_ref().to_str().ok_or(Error::InvalidInput)?;
    CString::new(path).map_err(|_| Error::InvalidInput)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        fs,
        time::{SystemTime, UNIX_EPOCH},
    };

    use crate::{BodyDef, ShapeDef, Vec3};

    #[derive(Default)]
    struct DrawCollector {
        bounds: usize,
        points: usize,
        segments: usize,
    }

    impl DebugDraw for DrawCollector {
        fn draw_bounds(&mut self, _bounds: Aabb, _color: u32) {
            self.bounds += 1;
        }

        fn draw_point(&mut self, _point: Vec3, _size: f32, _color: u32) {
            self.points += 1;
        }

        fn draw_segment(&mut self, _p1: Vec3, _p2: Vec3, _color: u32) {
            self.segments += 1;
        }
    }

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

        let bounds = Aabb {
            lower_bound: Vec3::new(-1.0, -1.0, -1.0),
            upper_bound: Vec3::new(1.0, 1.0, 1.0),
        };
        world.overlap_aabb(bounds, QueryFilter::default(), |_| true);
        let _ = world.cast_ray_closest(
            Vec3::new(0.0, 5.0, 0.0),
            Vec3::new(0.0, -10.0, 0.0),
            QueryFilter::default(),
        );

        for _ in 0..2 {
            world.step(1.0 / 60.0, 4);
        }

        world.stop_recording();

        let bytes = recording.bytes();
        assert!(!bytes.is_empty());
        assert!(recording.validate_replay(1));

        let path = recording_path();
        recording.save_to_file(&path).unwrap();
        let loaded = Recording::load_from_file(&path).unwrap();
        fs::remove_file(&path).unwrap();
        assert_eq!(loaded.bytes(), bytes);

        let mut player = RecPlayer::try_new(loaded.bytes(), 1).unwrap();
        assert_eq!(player.frame_count(), 2);
        assert_eq!(player.info().frame_count, 2);
        assert!(player.world_id().is_valid());
        player.set_keyframe_policy(1024 * 1024, 1);
        assert_eq!(player.keyframe_budget(), 1024 * 1024);
        assert_eq!(player.keyframe_min_interval(), 1);
        assert_eq!(player.keyframe_interval(), 1);
        assert_eq!(player.keyframe_bytes(), 0);

        assert!(player.step_frame());
        assert!(player.body_count() >= 2);
        assert!(player.body_id(0).is_some());
        let query_count = player.frame_query_count();
        assert!(query_count >= 2, "{query_count}");
        let query = player.frame_query(0).unwrap();
        assert!(matches!(
            query.query_type,
            RecQueryType::OverlapAabb | RecQueryType::CastRayClosest
        ));
        if query.hit_count > 0 {
            let hit = player.frame_query_hit(0, 0).unwrap();
            assert!(hit.shape.is_valid());
            assert!(hit.fraction.is_finite());
        }
        let mut draw = DrawCollector::default();
        player.draw_frame_queries(&mut draw, None, None);
        assert!(draw.bounds + draw.points + draw.segments > 0);

        while player.step_frame() {}

        assert!(player.is_at_end());
        assert!(!player.has_diverged(), "{}", player.diverge_frame());

        player.restart();
        assert_eq!(player.frame(), 0);
        player.seek_frame(1);
        assert_eq!(player.frame(), 1);
    }

    fn recording_path() -> std::path::PathBuf {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        std::env::temp_dir().join(format!("box3d-recording-{nanos}.b3rec"))
    }
}
