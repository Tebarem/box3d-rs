#![allow(non_camel_case_types, non_snake_case, non_upper_case_globals)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

#[cfg(test)]
mod tests {
    #[test]
    fn links_box3d() {
        unsafe {
            let version = crate::b3GetVersion();
            assert_eq!(version.major, 0);
            assert_eq!(version.minor, 1);
            assert!(crate::b3GetWorldCount() >= 0);
        }
    }
}
