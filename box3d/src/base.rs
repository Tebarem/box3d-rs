use std::time::Duration;

use box3d_sys as sys;

pub const HASH_INIT: u32 = 5381;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Version {
    pub major: i32,
    pub minor: i32,
    pub revision: i32,
}

impl From<sys::b3Version> for Version {
    fn from(value: sys::b3Version) -> Self {
        Self {
            major: value.major,
            minor: value.minor,
            revision: value.revision,
        }
    }
}

pub fn version() -> Version {
    unsafe { sys::b3GetVersion() }.into()
}

pub fn is_double_precision() -> bool {
    unsafe { sys::b3IsDoublePrecision() }
}

pub fn byte_count() -> i32 {
    unsafe { sys::b3GetByteCount() }
}

pub fn length_units_per_meter() -> f32 {
    unsafe { sys::b3GetLengthUnitsPerMeter() }
}

pub fn set_length_units_per_meter(length_units: f32) {
    assert!(length_units.is_finite() && length_units > 0.0);
    unsafe { sys::b3SetLengthUnitsPerMeter(length_units) };
}

pub fn stall_threshold() -> f32 {
    unsafe { sys::b3GetStallThreshold() }
}

pub fn set_stall_threshold(seconds: f32) {
    assert!(seconds.is_finite() && seconds > 0.0);
    unsafe { sys::b3SetStallThreshold(seconds) };
}

pub fn ticks() -> u64 {
    unsafe { sys::b3GetTicks() }
}

pub fn milliseconds(ticks: u64) -> f32 {
    unsafe { sys::b3GetMilliseconds(ticks) }
}

pub fn milliseconds_and_reset(ticks: &mut u64) -> f32 {
    unsafe { sys::b3GetMillisecondsAndReset(ticks) }
}

pub fn yield_now() {
    unsafe { sys::b3Yield() };
}

pub fn sleep(duration: Duration) {
    let milliseconds = duration.as_millis().min(i32::MAX as u128) as i32;
    unsafe { sys::b3Sleep(milliseconds) };
}

pub fn hash(seed: u32, data: &[u8]) -> u32 {
    let count = i32::try_from(data.len()).expect("box3d hash input exceeds i32::MAX bytes");
    unsafe { sys::b3Hash(seed, data.as_ptr(), count) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exposes_base_global_helpers() {
        let version = version();
        assert_eq!(version.major, 0);
        assert_eq!(version.minor, 1);
        assert!(version.revision >= 0);

        let _double_precision = is_double_precision();
        assert!(byte_count() >= 0);

        let length_units = length_units_per_meter();
        assert!(length_units.is_finite() && length_units > 0.0);
        set_length_units_per_meter(length_units);
        assert_eq!(length_units_per_meter(), length_units);

        let threshold = stall_threshold();
        assert!(threshold.is_finite() && threshold >= 0.0);
        set_stall_threshold(threshold);
        assert_eq!(stall_threshold(), threshold);
    }

    #[test]
    fn exposes_timer_sleep_and_hash_helpers() {
        let mut start = ticks();
        yield_now();
        assert!(milliseconds(start) >= 0.0);
        assert!(milliseconds_and_reset(&mut start) >= 0.0);

        sleep(Duration::from_millis(0));

        let data = b"box3d";
        assert_eq!(hash(HASH_INIT, data), hash(HASH_INIT, data));
    }
}
