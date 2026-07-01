# box3d-sys

Raw Rust FFI bindings for Box3D.

## Build

```sh
git submodule update --init --recursive
cargo test
```

The default `build-from-source` feature compiles the vendored Box3D C library with CMake and generates bindings from `include/box3d/box3d.h` with bindgen.

Disable default features to link an installed `box3d` instead. Set `BOX3D_INCLUDE_DIR` and `BOX3D_LIB_DIR` when the headers or library are outside the toolchain search paths.

You need CMake, a C/C++ toolchain, and libclang available to bindgen.
