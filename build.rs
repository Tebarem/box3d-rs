use std::{env, path::PathBuf};

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let box3d_dir = manifest_dir.join("vendor").join("box3d");
    let include_dir = env::var_os("BOX3D_INCLUDE_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| box3d_dir.join("include"));

    println!("cargo:rerun-if-changed=wrapper.h");

    #[cfg(feature = "build-from-source")]
    build_from_source(&box3d_dir);

    #[cfg(not(feature = "build-from-source"))]
    link_system_box3d();

    let target = env::var("TARGET").unwrap();
    if target.contains("linux") || target.contains("freebsd") || target.contains("dragonfly") {
        println!("cargo:rustc-link-lib=m");
    }

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

#[cfg(feature = "build-from-source")]
fn build_from_source(box3d_dir: &std::path::Path) {
    if !box3d_dir.join("CMakeLists.txt").exists() {
        panic!("missing vendor/box3d; run `git submodule update --init --recursive`");
    }

    println!("cargo:rerun-if-changed=vendor/box3d/include");
    println!("cargo:rerun-if-changed=vendor/box3d/src");

    let mut config = cmake::Config::new(box3d_dir);
    config
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
        .define("BOX3D_VALIDATE", "OFF");

    let target = env::var("TARGET").unwrap();
    if target.contains("msvc") {
        config.cflag("/O2").cflag("/Ob2").cflag("/DNDEBUG");
    } else {
        config.cflag("-O3").cflag("-DNDEBUG");
    }

    let dst = config.build();

    let lib_dir = if dst.join("lib64").exists() {
        dst.join("lib64")
    } else {
        dst.join("lib")
    };

    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=static=box3d");
}

#[cfg(not(feature = "build-from-source"))]
fn link_system_box3d() {
    if let Some(lib_dir) = env::var_os("BOX3D_LIB_DIR") {
        println!(
            "cargo:rustc-link-search=native={}",
            PathBuf::from(lib_dir).display()
        );
    }

    println!("cargo:rustc-link-lib=box3d");
}
