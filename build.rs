use std::{env, path::PathBuf};

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let box3d_dir = manifest_dir.join("vendor").join("box3d");

    if !box3d_dir.join("CMakeLists.txt").exists() {
        panic!("missing vendor/box3d; run `git submodule update --init --recursive`");
    }

    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=vendor/box3d/include");
    println!("cargo:rerun-if-changed=vendor/box3d/src");

    let dst = cmake::Config::new(&box3d_dir)
        .profile("Release")
        .define("BUILD_SHARED_LIBS", "OFF")
        .define("BOX3D_BENCHMARKS", "OFF")
        .define("BOX3D_BUILD_SHADERS", "OFF")
        .define("BOX3D_COMPILE_WARNING_AS_ERROR", "OFF")
        .define("BOX3D_DISABLE_SIMD", "OFF")
        .define("BOX3D_DOCS", "OFF")
        .define("BOX3D_DOUBLE_PRECISION", "OFF")
        .define("BOX3D_PROFILE", "OFF")
        .define("BOX3D_SAMPLES", "OFF")
        .define("BOX3D_SANITIZE", "OFF")
        .define("BOX3D_UNIT_TESTS", "OFF")
        .define("BOX3D_VALIDATE", "OFF")
        .build();

    let lib_dir = if dst.join("lib64").exists() {
        dst.join("lib64")
    } else {
        dst.join("lib")
    };

    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=static=box3d");

    let target = env::var("TARGET").unwrap();
    if target.contains("linux") || target.contains("freebsd") || target.contains("dragonfly") {
        println!("cargo:rustc-link-lib=m");
    }

    let include_dir = box3d_dir.join("include");
    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .clang_arg(format!("-I{}", include_dir.display()))
        .allowlist_function("b3.*")
        .allowlist_type("b3.*")
        .allowlist_var("b3.*")
        .allowlist_var("B3.*")
        .derive_default(true)
        .generate()
        .expect("failed to generate Box3D bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("failed to write Box3D bindings");
}
