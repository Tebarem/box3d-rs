# box3d

Small safe Rust wrapper for Box3D.

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
