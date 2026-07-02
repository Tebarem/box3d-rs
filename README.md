# box3d-rs

Rust bindings for Box3D.

- `box3d-sys`: raw FFI bindings.
- `box3d`: thin safe wrapper over common world, body, shape, query, joint, event, debug draw, recording, and worker-count APIs.

## Build

```sh
git submodule update --init --recursive
cargo test
```

The default `build-from-source` feature compiles the vendored Box3D C library with CMake and generates bindings from `include/box3d/box3d.h` with bindgen.

Disable default features to link an installed `box3d` instead. Set `BOX3D_INCLUDE_DIR` and `BOX3D_LIB_DIR` when the headers or library are outside the toolchain search paths.

You need CMake, a C/C++ toolchain, and libclang available to bindgen.

## Safe Wrapper

`box3d` owns native resources through `World`; bodies borrow the world, shapes borrow bodies, and mesh, height-field, or compound-backed shapes also borrow their source geometry object. Callback/query/event values are short-lived handles tied to the native step or query that produced them. Use `box3d-sys` directly only when you need raw C FFI coverage that the safe wrapper has not exposed.

```rust
use box3d::{BodyDef, ShapeDef, Vec3, World};

let world = World::default();
let body = world.create_body(BodyDef::dynamic_at(Vec3::new(0.0, 4.0, 0.0)));
let _shape = body.create_box(
    Vec3::new(0.5, 0.5, 0.5),
    ShapeDef {
        density: 1.0,
        friction: 0.3,
        ..ShapeDef::default()
    },
);

world.step(1.0 / 60.0, 4);
```

Feature groups are intentionally small: worlds and bodies manage simulation state, shapes wrap common primitives and owned mesh data, queries/collision cover ray casts and mover helpers, joints/events expose typed handles, and advanced modules cover callbacks, debug draw, recording/replay, and worker-count control.

The safe wrapper also has opt-in Bevy integration. Use the `box3d` crate's `bevy_ecs` feature for plain ECS support, or `bevy` for plugin/transform helpers as they land. No Bevy crates are pulled into default builds.
