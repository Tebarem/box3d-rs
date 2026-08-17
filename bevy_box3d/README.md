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

Use `Box3dConfig::fixed_hz`, `sub_steps`, startup `worker_count`, `contact_tuning`, and `contact_recycle_distance` for timing, threading, and contact solver control. Box3D schedules its native work on Bevy's shared `ComputeTaskPool`, so `Box3dPlugin::default()` keeps Bevy's fixed schedules multithreaded; set `single_threaded_schedules: true` if a project needs fixed schedules to run on one thread. Use `SleepThreshold` for per-body sleep tuning. Order gameplay with `Box3dSet::{Sync, Step, Writeback}`. Add `Box3dDebugPlugin` for Bevy gizmo collider wireframes.
