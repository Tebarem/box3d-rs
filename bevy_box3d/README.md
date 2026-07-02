# bevy_box3d

Bevy integration for `box3d`.

Supports Bevy 0.19 and Rust 1.95+.

```rust
use bevy::prelude::*;
use bevy_box3d::{Box3dPlugin, Collider, RigidBody};

App::new()
    .add_plugins((DefaultPlugins, Box3dPlugin::default()))
    .add_systems(Startup, |mut commands: Commands| {
        commands.spawn((
            RigidBody::Dynamic,
            Collider::sphere(0.5).with_density(1.0),
            Transform::from_xyz(0.0, 4.0, 0.0),
        ));
    })
    .run();
```

Use `Box3dConfig::fixed_hz` and `sub_steps` for timing control. Order gameplay with `Box3dSet::{Sync, Step, Writeback}`. Add `Box3dDebugPlugin` for Bevy gizmo collider wireframes.
