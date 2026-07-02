use std::ptr;

use box3d_sys as sys;

use crate::{math::Vec3, world::World};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QueryFilter {
    pub category_bits: u64,
    pub mask_bits: u64,
    pub id: u64,
}

impl Default for QueryFilter {
    fn default() -> Self {
        unsafe { sys::b3DefaultQueryFilter() }.into()
    }
}

impl From<sys::b3QueryFilter> for QueryFilter {
    fn from(value: sys::b3QueryFilter) -> Self {
        Self {
            category_bits: value.categoryBits,
            mask_bits: value.maskBits,
            id: value.id,
        }
    }
}

impl From<QueryFilter> for sys::b3QueryFilter {
    fn from(value: QueryFilter) -> Self {
        Self {
            categoryBits: value.category_bits,
            maskBits: value.mask_bits,
            id: value.id,
            name: ptr::null(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    pub point: Vec3,
    pub normal: Vec3,
    pub user_material_id: u64,
    pub fraction: f32,
    pub triangle_index: i32,
    pub child_index: i32,
    pub node_visits: i32,
    pub leaf_visits: i32,
}

impl RayHit {
    fn from_raw(value: sys::b3RayResult) -> Option<Self> {
        value.hit.then(|| Self {
            point: value.point.into(),
            normal: value.normal.into(),
            user_material_id: value.userMaterialId,
            fraction: value.fraction,
            triangle_index: value.triangleIndex,
            child_index: value.childIndex,
            node_visits: value.nodeVisits,
            leaf_visits: value.leafVisits,
        })
    }
}

impl World {
    pub fn cast_ray_closest(
        &self,
        origin: Vec3,
        translation: Vec3,
        filter: QueryFilter,
    ) -> Option<RayHit> {
        let hit = unsafe {
            sys::b3World_CastRayClosest(
                self.raw(),
                origin.into(),
                translation.into(),
                filter.into(),
            )
        };
        RayHit::from_raw(hit)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, ShapeDef};

    #[test]
    fn closest_ray_hits_static_box() {
        let world = World::new(Vec3::ZERO);
        let body = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let _shape = body.create_box(Vec3::new(1.0, 1.0, 1.0), ShapeDef::default());

        let hit = world
            .cast_ray_closest(
                Vec3::new(-3.0, 0.0, 0.0),
                Vec3::new(6.0, 0.0, 0.0),
                QueryFilter::default(),
            )
            .expect("ray should hit box");

        assert!((hit.point.x + 1.0).abs() < 0.01, "{hit:?}");
        assert!(hit.normal.x < 0.0, "{hit:?}");
        assert!(hit.fraction > 0.0 && hit.fraction < 1.0, "{hit:?}");
    }

    #[test]
    fn closest_ray_misses_empty_world() {
        let world = World::new(Vec3::ZERO);

        let hit = world.cast_ray_closest(
            Vec3::new(-3.0, 0.0, 0.0),
            Vec3::new(6.0, 0.0, 0.0),
            QueryFilter::default(),
        );

        assert!(hit.is_none());
    }
}
