# box3d-rs

Rust bindings for Box3D.

- `box3d-sys`: raw FFI bindings.
- `box3d`: small safe wrapper over the core world/body/shape flow.

## Build

```sh
git submodule update --init --recursive
cargo test
```

The default `build-from-source` feature compiles the vendored Box3D C library with CMake and generates bindings from `include/box3d/box3d.h` with bindgen.

Disable default features to link an installed `box3d` instead. Set `BOX3D_INCLUDE_DIR` and `BOX3D_LIB_DIR` when the headers or library are outside the toolchain search paths.

You need CMake, a C/C++ toolchain, and libclang available to bindgen.

## Safe Wrapper

```rust
use box3d::{BodyDef, ShapeDef, Vec3, World};

let world = World::default();
let body = world.create_body(BodyDef::dynamic_at(Vec3::new(0.0, 4.0, 0.0)));
let _shape = body.create_box(
    Vec3::new(0.5, 0.5, 0.5),
    ShapeDef { density: 1.0, friction: 0.3 },
);

world.step(1.0 / 60.0, 4);
```
