# box3d

Thin safe Rust wrapper for Box3D. The `box3d-sys` crate remains the raw FFI layer; this crate owns native resources through `World`, with bodies borrowing worlds, shapes borrowing bodies, and mesh, height-field, or compound-backed shapes also borrowing their source geometry object.

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

Feature groups include worlds/bodies/shapes, queries and standalone collision helpers, joints, events, character movement, debug draw, callbacks, recording/replay, and worker-count control.

## Bevy

Use the separate `bevy_box3d` crate for Bevy integration. The core `box3d` crate has no Bevy dependency.

```toml
box3d = "0.1"
bevy_box3d = "0.1"
```
