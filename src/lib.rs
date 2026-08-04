#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals)]
#![allow(
    clippy::approx_constant,
    clippy::missing_safety_doc,
    clippy::ptr_offset_with_cast,
    clippy::useless_transmute
)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

#[cfg(feature = "double-precision")]
pub use b3CreateWorldDoublePrecision as b3CreateWorld;

#[cfg(test)]
mod tests {
    #[test]
    fn links_box3d() {
        unsafe {
            let version = crate::b3GetVersion();
            assert_eq!(version.major, 0);
            assert_eq!(version.minor, 1);
            assert_eq!(
                crate::b3IsDoublePrecision(),
                cfg!(feature = "double-precision")
            );
            assert_eq!(
                std::mem::size_of::<crate::b3Pos>(),
                if cfg!(feature = "double-precision") {
                    24
                } else {
                    12
                }
            );
            assert_eq!(
                std::mem::size_of::<crate::b3WorldTransform>(),
                if cfg!(feature = "double-precision") {
                    40
                } else {
                    28
                }
            );

            let world = crate::b3CreateWorld(&crate::b3DefaultWorldDef());
            assert!(crate::b3World_IsValid(world));
            crate::b3DestroyWorld(world);
            assert!(crate::b3GetWorldCount() >= 0);
        }
    }
}
