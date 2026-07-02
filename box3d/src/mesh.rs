use std::{ptr, ptr::NonNull};

use box3d_sys as sys;

use crate::{
    body::Body,
    handle,
    math::Vec3,
    shape::{raw_shape_def, Shape, ShapeDef},
    Error, Result,
};

pub struct Mesh {
    raw: NonNull<sys::b3MeshData>,
}

impl Mesh {
    pub fn from_triangles(vertices: &[Vec3], indices: &[u32]) -> Result<Self> {
        if vertices.len() < 3 || indices.is_empty() || indices.len() % 3 != 0 {
            return Err(Error::InvalidInput);
        }

        let vertex_count = i32::try_from(vertices.len()).map_err(|_| Error::InvalidInput)?;
        let triangle_count = i32::try_from(indices.len() / 3).map_err(|_| Error::InvalidInput)?;
        let mut raw_vertices = vertices.iter().copied().map(Into::into).collect::<Vec<_>>();
        let mut raw_indices = indices
            .iter()
            .copied()
            .map(|index| {
                if index as usize >= vertices.len() {
                    return Err(Error::InvalidInput);
                }
                i32::try_from(index).map_err(|_| Error::InvalidInput)
            })
            .collect::<Result<Vec<_>>>()?;

        let def = sys::b3MeshDef {
            vertices: raw_vertices.as_mut_ptr(),
            indices: raw_indices.as_mut_ptr(),
            materialIndices: ptr::null_mut(),
            weldTolerance: 0.0,
            vertexCount: vertex_count,
            triangleCount: triangle_count,
            weldVertices: false,
            useMedianSplit: false,
            identifyEdges: true,
        };
        let raw = NonNull::new(unsafe { sys::b3CreateMesh(&def, ptr::null_mut(), 0) })
            .ok_or(Error::Null)?;

        Ok(Self { raw })
    }

    pub fn box_mesh(center: Vec3, half_extents: Vec3, weld: bool) -> Self {
        assert!(half_extents.x > 0.0 && half_extents.y > 0.0 && half_extents.z > 0.0);
        let raw = unsafe { sys::b3CreateBoxMesh(center.into(), half_extents.into(), weld) };
        let raw = NonNull::new(raw).expect("box3d returned a null mesh");

        Self { raw }
    }

    pub(crate) fn raw(&self) -> *const sys::b3MeshData {
        self.raw.as_ptr()
    }
}

impl Drop for Mesh {
    fn drop(&mut self) {
        unsafe { sys::b3DestroyMesh(self.raw.as_ptr()) };
    }
}

pub struct HeightField {
    raw: NonNull<sys::b3HeightFieldData>,
}

impl HeightField {
    pub fn new(heights: &[f32], count_x: i32, count_z: i32, scale: Vec3) -> Result<Self> {
        if count_x < 2
            || count_z < 2
            || scale.x <= 0.0
            || scale.y <= 0.0
            || scale.z <= 0.0
            || heights.len() != (count_x as usize) * (count_z as usize)
            || heights.iter().any(|height| !height.is_finite())
        {
            return Err(Error::InvalidInput);
        }

        let (minimum, maximum) = heights.iter().copied().fold(
            (f32::INFINITY, f32::NEG_INFINITY),
            |(minimum, maximum), height| (minimum.min(height), maximum.max(height)),
        );
        let mut raw_heights = heights.to_vec();
        let def = sys::b3HeightFieldDef {
            heights: raw_heights.as_mut_ptr(),
            materialIndices: ptr::null_mut(),
            scale: scale.into(),
            countX: count_x,
            countZ: count_z,
            globalMinimumHeight: minimum,
            globalMaximumHeight: maximum,
            clockwiseWinding: false,
        };
        let raw = NonNull::new(unsafe { sys::b3CreateHeightField(&def) }).ok_or(Error::Null)?;

        Ok(Self { raw })
    }

    pub(crate) fn raw(&self) -> *const sys::b3HeightFieldData {
        self.raw.as_ptr()
    }
}

impl Drop for HeightField {
    fn drop(&mut self) {
        unsafe { sys::b3DestroyHeightField(self.raw.as_ptr()) };
    }
}

impl Body<'_> {
    pub fn create_mesh(&self, mesh: &Mesh, scale: Vec3, def: ShapeDef) -> Shape<'_> {
        let raw_def = raw_shape_def(def);
        let raw = handle::shape(unsafe {
            sys::b3CreateMeshShape(self.raw(), &raw_def, mesh.raw(), scale.into())
        })
        .expect("box3d returned an invalid shape");

        Shape::from_raw(raw)
    }

    pub fn create_height_field(&self, height_field: &HeightField, def: ShapeDef) -> Shape<'_> {
        let raw_def = raw_shape_def(def);
        let raw = handle::shape(unsafe {
            sys::b3CreateHeightFieldShape(self.raw(), &raw_def, height_field.raw())
        })
        .expect("box3d returned an invalid shape");

        Shape::from_raw(raw)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, World};

    #[test]
    fn mesh_ground_supports_dynamic_sphere() {
        let world = World::default();
        let ground = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let mesh = Mesh::box_mesh(
            Vec3::new(0.0, -10.0, 0.0),
            Vec3::new(50.0, 10.0, 50.0),
            true,
        );
        let _ground_shape =
            ground.create_mesh(&mesh, Vec3::new(1.0, 1.0, 1.0), ShapeDef::default());

        let body = world.create_body(BodyDef::dynamic_at(Vec3::new(0.0, 4.0, 0.0)));
        let shape = body.create_sphere(
            Vec3::ZERO,
            0.5,
            ShapeDef {
                density: 1.0,
                friction: 0.3,
            },
        );

        for _ in 0..90 {
            world.step(1.0 / 60.0, 4);
        }

        assert!(shape.is_valid());
        assert!(
            (body.position().y - 0.5).abs() < 0.05,
            "{:?}",
            body.position()
        );
    }

    #[test]
    fn height_field_attaches_to_static_body() {
        let world = World::default();
        let body = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let field =
            HeightField::new(&[0.0, 0.0, 0.0, 0.0], 2, 2, Vec3::new(1.0, 1.0, 1.0)).unwrap();

        let shape = body.create_height_field(&field, ShapeDef::default());

        assert!(shape.is_valid());
    }

    #[test]
    fn mesh_rejects_bad_indices() {
        let vertices = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        ];

        assert_eq!(
            Mesh::from_triangles(&vertices, &[0, 1, 3]).err(),
            Some(Error::InvalidInput)
        );
    }
}
