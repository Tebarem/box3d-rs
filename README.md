# box3d-sys

Raw Rust FFI bindings for Box3D.

## Build

```sh
git submodule update --init --recursive
cargo test
```

The build script compiles the vendored Box3D C library with CMake and generates bindings from `include/box3d/box3d.h` with bindgen.

You need CMake, a C/C++ toolchain, and libclang available to bindgen.
