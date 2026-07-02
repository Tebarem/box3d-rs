//! Optional Bevy ECS integration.
//!
//! This module is available with the `bevy_ecs` feature. The wider `bevy`
//! feature enables Bevy split crates for plugin and transform helpers.

use bevy_ecs::prelude::Resource;

use crate::{Capacity, Vec3};

/// Bevy minor version supported by this integration.
///
/// Bevy 0.19 currently requires a newer Rust compiler than this workspace uses,
/// so the feature is pinned to the latest compatible 0.18 release.
pub const SUPPORTED_BEVY_VERSION: &str = "0.18";

/// Initial settings for the Box3D world owned by a Bevy schedule.
#[derive(Clone, Copy, Debug, PartialEq, Resource)]
pub struct Box3dConfig {
    pub gravity: Vec3,
    pub sub_steps: i32,
    pub capacity: Capacity,
}

impl Default for Box3dConfig {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.8, 0.0),
            sub_steps: 4,
            capacity: Capacity::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use bevy_ecs::world::World as EcsWorld;

    use super::*;

    #[test]
    fn config_is_a_bevy_resource() {
        let mut world = EcsWorld::new();
        world.insert_resource(Box3dConfig::default());

        assert_eq!(world.resource::<Box3dConfig>().sub_steps, 4);
    }
}
