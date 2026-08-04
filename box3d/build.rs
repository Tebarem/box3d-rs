fn main() {
    println!("cargo:rustc-check-cfg=cfg(box3d_double_precision)");
    println!("cargo:rerun-if-env-changed=DEP_BOX3D_DOUBLE_PRECISION");

    if matches!(
        std::env::var("DEP_BOX3D_DOUBLE_PRECISION").as_deref(),
        Ok("true")
    ) {
        println!("cargo:rustc-cfg=box3d_double_precision");
    }
}
