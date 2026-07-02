use std::{
    any::Any,
    ffi::{c_void, CString},
    panic::{catch_unwind, resume_unwind, AssertUnwindSafe},
    path::Path,
    ptr,
    ptr::NonNull,
    slice,
};

use box3d_sys as sys;

use crate::{
    body::Body,
    collision::{compute_height_field_aabb, compute_mesh_aabb},
    handle,
    math::{Aabb, Transform, Vec3},
    shape::{raw_shape_def, Shape, ShapeDef},
    Error, Result,
};

type CallbackPanic = Box<dyn Any + Send + 'static>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeshTriangle {
    pub indices: [u32; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshQueryTriangle {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
    pub triangle_index: i32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshCreateOptions {
    pub weld_tolerance: f32,
    pub weld_vertices: bool,
    pub use_median_split: bool,
    pub identify_edges: bool,
}

impl Default for MeshCreateOptions {
    fn default() -> Self {
        Self {
            weld_tolerance: 0.0,
            weld_vertices: false,
            use_median_split: false,
            identify_edges: true,
        }
    }
}

pub struct Mesh {
    raw: NonNull<sys::b3MeshData>,
}

impl Mesh {
    pub fn from_triangles(vertices: &[Vec3], indices: &[u32]) -> Result<Self> {
        Self::from_triangles_with_options(vertices, indices, None, MeshCreateOptions::default())
    }

    pub fn from_triangles_with_options(
        vertices: &[Vec3],
        indices: &[u32],
        material_indices: Option<&[u8]>,
        options: MeshCreateOptions,
    ) -> Result<Self> {
        if vertices.len() < 3 || indices.is_empty() || !indices.len().is_multiple_of(3) {
            return Err(Error::InvalidInput);
        }

        let vertex_count = i32::try_from(vertices.len()).map_err(|_| Error::InvalidInput)?;
        let triangle_count = i32::try_from(indices.len() / 3).map_err(|_| Error::InvalidInput)?;
        if material_indices.is_some_and(|materials| materials.len() != triangle_count as usize)
            || options.weld_tolerance < 0.0
            || !options.weld_tolerance.is_finite()
            || vertices.iter().any(|vertex| !is_valid_vec3(*vertex))
        {
            return Err(Error::InvalidInput);
        }

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
        let mut material_indices = material_indices.map_or_else(Vec::new, ToOwned::to_owned);

        let def = sys::b3MeshDef {
            vertices: raw_vertices.as_mut_ptr(),
            indices: raw_indices.as_mut_ptr(),
            materialIndices: as_mut_ptr(&mut material_indices),
            weldTolerance: options.weld_tolerance,
            vertexCount: vertex_count,
            triangleCount: triangle_count,
            weldVertices: options.weld_vertices,
            useMedianSplit: options.use_median_split,
            identifyEdges: options.identify_edges,
        };
        Self::from_raw(unsafe { sys::b3CreateMesh(&def, ptr::null_mut(), 0) })
    }

    pub fn grid(x_count: i32, z_count: i32, cell_width: f32, material_count: i32) -> Result<Self> {
        Self::grid_with_edges(x_count, z_count, cell_width, material_count, true)
    }

    pub fn grid_with_edges(
        x_count: i32,
        z_count: i32,
        cell_width: f32,
        material_count: i32,
        identify_edges: bool,
    ) -> Result<Self> {
        if x_count < 2
            || z_count < 2
            || cell_width <= 0.0
            || !(0..=u8::MAX as i32).contains(&material_count)
        {
            return Err(Error::InvalidInput);
        }
        Self::from_raw(unsafe {
            sys::b3CreateGridMesh(x_count, z_count, cell_width, material_count, identify_edges)
        })
    }

    pub fn wave(
        x_count: i32,
        z_count: i32,
        cell_width: f32,
        amplitude: f32,
        row_frequency: f32,
        column_frequency: f32,
    ) -> Result<Self> {
        if x_count < 2
            || z_count < 2
            || cell_width <= 0.0
            || !amplitude.is_finite()
            || !row_frequency.is_finite()
            || !column_frequency.is_finite()
        {
            return Err(Error::InvalidInput);
        }
        Self::from_raw(unsafe {
            sys::b3CreateWaveMesh(
                x_count,
                z_count,
                cell_width,
                amplitude,
                row_frequency,
                column_frequency,
            )
        })
    }

    pub fn torus(
        radial_resolution: i32,
        tubular_resolution: i32,
        radius: f32,
        thickness: f32,
    ) -> Result<Self> {
        if radial_resolution < 3 || tubular_resolution < 3 || radius <= 0.0 || thickness <= 0.0 {
            return Err(Error::InvalidInput);
        }
        Self::from_raw(unsafe {
            sys::b3CreateTorusMesh(radial_resolution, tubular_resolution, radius, thickness)
        })
    }

    pub fn box_mesh(center: Vec3, half_extents: Vec3, weld: bool) -> Self {
        assert!(is_valid_vec3(center));
        assert!(half_extents.x > 0.0 && half_extents.y > 0.0 && half_extents.z > 0.0);
        let raw = unsafe { sys::b3CreateBoxMesh(center.into(), half_extents.into(), weld) };
        Self::from_raw(raw).expect("box3d returned a null mesh")
    }

    pub fn hollow_box(center: Vec3, half_extents: Vec3) -> Result<Self> {
        if !is_valid_vec3(center)
            || half_extents.x <= 0.0
            || half_extents.y <= 0.0
            || half_extents.z <= 0.0
        {
            return Err(Error::InvalidInput);
        }
        Self::from_raw(unsafe { sys::b3CreateHollowBoxMesh(center.into(), half_extents.into()) })
    }

    pub fn platform(center: Vec3, height: f32, top_width: f32, bottom_width: f32) -> Result<Self> {
        if !is_valid_vec3(center) || height <= 0.0 || top_width <= 0.0 || bottom_width <= 0.0 {
            return Err(Error::InvalidInput);
        }
        Self::from_raw(unsafe {
            sys::b3CreatePlatformMesh(center.into(), height, top_width, bottom_width)
        })
    }

    pub fn height(&self) -> i32 {
        unsafe { sys::b3GetHeight(self.raw.as_ptr()) }
    }

    pub fn bounds(&self) -> Aabb {
        unsafe { self.raw.as_ref().bounds }.into()
    }

    pub fn compute_aabb(&self, transform: Transform, scale: Vec3) -> Aabb {
        compute_mesh_aabb(self, transform, scale)
    }

    pub fn vertex_count(&self) -> i32 {
        unsafe { self.raw.as_ref().vertexCount }
    }

    pub fn triangle_count(&self) -> i32 {
        unsafe { self.raw.as_ref().triangleCount }
    }

    pub fn vertices(&self) -> Vec<Vec3> {
        let raw = unsafe { self.raw.as_ref() };
        unsafe { offset_slice::<sys::b3Vec3, _>(raw, raw.vertexOffset, raw.vertexCount) }
            .iter()
            .copied()
            .map(Into::into)
            .collect()
    }

    pub fn triangles(&self) -> Vec<MeshTriangle> {
        let raw = unsafe { self.raw.as_ref() };
        unsafe {
            offset_slice::<sys::b3MeshTriangle, _>(raw, raw.triangleOffset, raw.triangleCount)
        }
        .iter()
        .map(|triangle| MeshTriangle {
            indices: [
                triangle.index1 as u32,
                triangle.index2 as u32,
                triangle.index3 as u32,
            ],
        })
        .collect()
    }

    pub fn material_indices(&self) -> Vec<u8> {
        let raw = unsafe { self.raw.as_ref() };
        unsafe { offset_slice::<u8, _>(raw, raw.materialOffset, raw.triangleCount) }.to_vec()
    }

    pub fn flags(&self) -> Vec<u8> {
        let raw = unsafe { self.raw.as_ref() };
        unsafe { offset_slice::<u8, _>(raw, raw.flagsOffset, raw.triangleCount) }.to_vec()
    }

    pub fn query<F>(&self, scale: Vec3, bounds: Aabb, mut f: F)
    where
        F: FnMut(MeshQueryTriangle) -> bool,
    {
        assert_valid_vec3(scale);
        assert_valid_aabb(bounds);
        let raw_mesh = sys::b3Mesh {
            data: self.raw(),
            scale: scale.into(),
        };
        let mut context = MeshQueryContext {
            f: &mut f,
            panic: None,
        };
        unsafe {
            sys::b3QueryMesh(
                &raw_mesh,
                bounds.into(),
                Some(mesh_query_callback::<F>),
                (&mut context as *mut MeshQueryContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
    }

    pub(crate) fn raw(&self) -> *const sys::b3MeshData {
        self.raw.as_ptr()
    }

    fn from_raw(raw: *mut sys::b3MeshData) -> Result<Self> {
        let raw = NonNull::new(raw).ok_or(Error::Null)?;
        Ok(Self { raw })
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
            || !is_positive_vec3(scale)
            || heights.len() != (count_x as usize) * (count_z as usize)
            || heights.iter().any(|height| !height.is_finite())
        {
            return Err(Error::InvalidInput);
        }

        let (minimum, maximum) = min_max(heights);
        let mut raw_heights = heights.to_vec();
        let mut material_indices = vec![0; ((count_x - 1) * (count_z - 1)) as usize];
        let def = sys::b3HeightFieldDef {
            heights: raw_heights.as_mut_ptr(),
            materialIndices: material_indices.as_mut_ptr(),
            scale: scale.into(),
            countX: count_x,
            countZ: count_z,
            globalMinimumHeight: minimum,
            globalMaximumHeight: maximum,
            clockwiseWinding: false,
        };
        Self::from_raw(unsafe { sys::b3CreateHeightField(&def) })
    }

    pub fn grid(row_count: i32, column_count: i32, scale: Vec3, make_holes: bool) -> Result<Self> {
        if row_count < 2 || column_count < 2 || !is_positive_vec3(scale) {
            return Err(Error::InvalidInput);
        }
        Self::from_raw(unsafe {
            sys::b3CreateGrid(row_count, column_count, scale.into(), make_holes)
        })
    }

    pub fn wave(
        row_count: i32,
        column_count: i32,
        scale: Vec3,
        row_frequency: f32,
        column_frequency: f32,
        make_holes: bool,
    ) -> Result<Self> {
        if row_count < 2
            || column_count < 2
            || !is_positive_vec3(scale)
            || !row_frequency.is_finite()
            || !column_frequency.is_finite()
        {
            return Err(Error::InvalidInput);
        }
        Self::from_raw(unsafe {
            sys::b3CreateWave(
                row_count,
                column_count,
                scale.into(),
                row_frequency,
                column_frequency,
                make_holes,
            )
        })
    }

    pub fn load(path: impl AsRef<Path>) -> Result<Self> {
        let path = path_to_cstring(path)?;
        Self::from_raw(unsafe { sys::b3LoadHeightField(path.as_ptr()) })
    }

    pub fn dump_data(
        path: impl AsRef<Path>,
        heights: &[f32],
        count_x: i32,
        count_z: i32,
        scale: Vec3,
    ) -> Result<()> {
        if count_x < 2
            || count_z < 2
            || !is_positive_vec3(scale)
            || heights.len() != (count_x as usize) * (count_z as usize)
            || heights.iter().any(|height| !height.is_finite())
        {
            return Err(Error::InvalidInput);
        }
        let raw_path = path_to_cstring(&path)?;
        let (minimum, maximum) = min_max(heights);
        let mut raw_heights = heights.to_vec();
        let mut material_indices = vec![0; ((count_x - 1) * (count_z - 1)) as usize];
        let def = sys::b3HeightFieldDef {
            heights: raw_heights.as_mut_ptr(),
            materialIndices: material_indices.as_mut_ptr(),
            scale: scale.into(),
            countX: count_x,
            countZ: count_z,
            globalMinimumHeight: minimum,
            globalMaximumHeight: maximum,
            clockwiseWinding: false,
        };
        unsafe { sys::b3DumpHeightData(&def, raw_path.as_ptr()) };
        let metadata = std::fs::metadata(path.as_ref()).map_err(|_| Error::InvalidInput)?;
        if metadata.is_file() && metadata.len() > 0 {
            Ok(())
        } else {
            Err(Error::InvalidInput)
        }
    }

    pub fn bounds(&self) -> Aabb {
        unsafe { self.raw.as_ref().aabb }.into()
    }

    pub fn compute_aabb(&self, transform: Transform) -> Aabb {
        compute_height_field_aabb(self, transform)
    }

    pub fn row_count(&self) -> i32 {
        unsafe { self.raw.as_ref().rowCount }
    }

    pub fn column_count(&self) -> i32 {
        unsafe { self.raw.as_ref().columnCount }
    }

    pub fn scale(&self) -> Vec3 {
        unsafe { self.raw.as_ref().scale }.into()
    }

    pub fn min_height(&self) -> f32 {
        unsafe { self.raw.as_ref().minHeight }
    }

    pub fn max_height(&self) -> f32 {
        unsafe { self.raw.as_ref().maxHeight }
    }

    pub fn compressed_heights(&self) -> Vec<u16> {
        let raw = unsafe { self.raw.as_ref() };
        let count = raw.rowCount.saturating_mul(raw.columnCount);
        unsafe { offset_slice::<u16, _>(raw, raw.heightsOffset, count) }.to_vec()
    }

    pub fn material_indices(&self) -> Vec<u8> {
        let raw = unsafe { self.raw.as_ref() };
        let count = (raw.rowCount - 1).saturating_mul(raw.columnCount - 1);
        unsafe { offset_slice::<u8, _>(raw, raw.materialOffset, count) }.to_vec()
    }

    pub fn flags(&self) -> Vec<u8> {
        let raw = unsafe { self.raw.as_ref() };
        let count = 2 * (raw.rowCount - 1).saturating_mul(raw.columnCount - 1);
        unsafe { offset_slice::<u8, _>(raw, raw.flagsOffset, count) }.to_vec()
    }

    pub fn query<F>(&self, bounds: Aabb, mut f: F)
    where
        F: FnMut(MeshQueryTriangle) -> bool,
    {
        assert_valid_aabb(bounds);
        let mut context = MeshQueryContext {
            f: &mut f,
            panic: None,
        };
        unsafe {
            sys::b3QueryHeightField(
                self.raw(),
                bounds.into(),
                Some(mesh_query_callback::<F>),
                (&mut context as *mut MeshQueryContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
    }

    pub(crate) fn raw(&self) -> *const sys::b3HeightFieldData {
        self.raw.as_ptr()
    }

    fn from_raw(raw: *mut sys::b3HeightFieldData) -> Result<Self> {
        let raw = NonNull::new(raw).ok_or(Error::Null)?;
        Ok(Self { raw })
    }
}

impl Drop for HeightField {
    fn drop(&mut self) {
        unsafe { sys::b3DestroyHeightField(self.raw.as_ptr()) };
    }
}

impl Body<'_> {
    pub fn create_mesh<'a>(&'a self, mesh: &'a Mesh, scale: Vec3, def: ShapeDef) -> Shape<'a> {
        let raw_def = raw_shape_def(def);
        let raw = handle::shape(unsafe {
            sys::b3CreateMeshShape(self.raw(), &raw_def, mesh.raw(), scale.into())
        })
        .expect("box3d returned an invalid shape");

        Shape::from_raw(raw)
    }

    pub fn create_height_field<'a>(
        &'a self,
        height_field: &'a HeightField,
        def: ShapeDef,
    ) -> Shape<'a> {
        let raw_def = raw_shape_def(def);
        let raw = handle::shape(unsafe {
            sys::b3CreateHeightFieldShape(self.raw(), &raw_def, height_field.raw())
        })
        .expect("box3d returned an invalid shape");

        Shape::from_raw(raw)
    }
}

struct MeshQueryContext<'a, F> {
    f: &'a mut F,
    panic: Option<CallbackPanic>,
}

unsafe extern "C" fn mesh_query_callback<F>(
    a: sys::b3Vec3,
    b: sys::b3Vec3,
    c: sys::b3Vec3,
    triangle_index: i32,
    context: *mut c_void,
) -> bool
where
    F: FnMut(MeshQueryTriangle) -> bool,
{
    let context = unsafe { &mut *(context as *mut MeshQueryContext<'_, F>) };
    if context.panic.is_some() {
        return false;
    }

    match catch_unwind(AssertUnwindSafe(|| {
        (context.f)(MeshQueryTriangle {
            a: a.into(),
            b: b.into(),
            c: c.into(),
            triangle_index,
        })
    })) {
        Ok(keep_going) => keep_going,
        Err(panic) => {
            context.panic = Some(panic);
            false
        }
    }
}

fn resume_callback_panic(panic: Option<CallbackPanic>) {
    if let Some(panic) = panic {
        resume_unwind(panic);
    }
}

fn as_mut_ptr<T>(values: &mut Vec<T>) -> *mut T {
    if values.is_empty() {
        ptr::null_mut()
    } else {
        values.as_mut_ptr()
    }
}

unsafe fn offset_slice<T, U>(base: &U, offset: i32, count: i32) -> &[T] {
    if offset <= 0 || count <= 0 {
        &[]
    } else {
        unsafe {
            slice::from_raw_parts(
                (std::ptr::from_ref(base).cast::<u8>())
                    .add(offset as usize)
                    .cast::<T>(),
                count as usize,
            )
        }
    }
}

fn min_max(values: &[f32]) -> (f32, f32) {
    values
        .iter()
        .copied()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(min, max), value| {
            (min.min(value), max.max(value))
        })
}

fn assert_valid_aabb(aabb: Aabb) {
    assert_valid_vec3(aabb.lower_bound);
    assert_valid_vec3(aabb.upper_bound);
    assert!(aabb.lower_bound.x <= aabb.upper_bound.x);
    assert!(aabb.lower_bound.y <= aabb.upper_bound.y);
    assert!(aabb.lower_bound.z <= aabb.upper_bound.z);
}

fn assert_valid_vec3(value: Vec3) {
    assert!(is_valid_vec3(value));
}

fn is_valid_vec3(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

fn is_positive_vec3(value: Vec3) -> bool {
    value.x > 0.0 && value.y > 0.0 && value.z > 0.0
}

fn path_to_cstring(path: impl AsRef<Path>) -> Result<CString> {
    let path = path.as_ref().to_str().ok_or(Error::InvalidInput)?;
    CString::new(path).map_err(|_| Error::InvalidInput)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        overlap_height_field, overlap_mesh, ray_cast_height_field, ray_cast_mesh,
        shape_cast_height_field, shape_cast_mesh, BodyDef, RayCastInput, ShapeCastInput,
        ShapeProxy, Transform, World,
    };

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
                ..ShapeDef::default()
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

    #[test]
    fn mesh_generators_aabb_query_and_casts() {
        let grid = Mesh::grid(3, 3, 1.0, 1).unwrap();
        assert!(grid.vertex_count() > 0);
        assert!(grid.triangle_count() > 0);
        assert_eq!(grid.triangles().len(), grid.triangle_count() as usize);
        assert_eq!(
            grid.material_indices().len(),
            grid.triangle_count() as usize
        );
        assert!(grid.height() >= 0);

        assert!(
            Mesh::wave(3, 3, 1.0, 0.25, 0.1, 0.2)
                .unwrap()
                .triangle_count()
                > 0
        );
        assert!(Mesh::torus(4, 4, 1.0, 0.25).unwrap().triangle_count() > 0);
        assert!(
            Mesh::hollow_box(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0))
                .unwrap()
                .triangle_count()
                > 0
        );
        assert!(
            Mesh::platform(Vec3::ZERO, 1.0, 1.0, 2.0)
                .unwrap()
                .triangle_count()
                > 0
        );

        let mut visited = 0;
        grid.query(Vec3::new(1.0, 1.0, 1.0), grid.bounds(), |triangle| {
            visited += 1;
            triangle.triangle_index >= 0
        });
        assert!(visited > 0);

        let mesh = Mesh::box_mesh(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0), true);
        let scale = Vec3::new(1.0, 1.0, 1.0);
        let aabb = mesh.compute_aabb(Transform::IDENTITY, scale);
        assert_eq!(aabb.lower_bound, Vec3::new(-1.0, -1.0, -1.0));
        assert_eq!(aabb.upper_bound, Vec3::new(1.0, 1.0, 1.0));

        let overlap_point = [Vec3::new(1.0, 0.0, 0.0)];
        let overlap_proxy = ShapeProxy::new(&overlap_point, 0.1).unwrap();
        assert!(overlap_mesh(
            &mesh,
            scale,
            Transform::IDENTITY,
            overlap_proxy
        ));
        assert!(ray_cast_mesh(
            &mesh,
            scale,
            RayCastInput::new(Vec3::new(-3.0, 0.0, 0.0), Vec3::new(6.0, 0.0, 0.0), 1.0),
        )
        .is_some());

        let cast_point = [Vec3::new(-3.0, 0.0, 0.0)];
        let cast_proxy = ShapeProxy::new(&cast_point, 0.1).unwrap();
        assert!(shape_cast_mesh(
            &mesh,
            scale,
            ShapeCastInput::new(cast_proxy, Vec3::new(6.0, 0.0, 0.0), 1.0, false),
        )
        .is_some());
    }

    #[test]
    fn height_field_grid_wave_dump_load_query_and_casts() {
        let scale = Vec3::new(1.0, 1.0, 1.0);
        let grid = HeightField::grid(4, 4, scale, false).unwrap();
        assert_eq!(grid.row_count(), 4);
        assert_eq!(grid.column_count(), 4);
        assert_eq!(grid.compressed_heights().len(), 16);
        assert_eq!(grid.material_indices().len(), 9);
        assert_eq!(grid.flags().len(), 18);

        assert!(HeightField::wave(4, 4, scale, 0.1, 0.2, false)
            .unwrap()
            .max_height()
            .is_finite());

        let mut visited = 0;
        grid.query(
            Aabb {
                lower_bound: Vec3::new(-10.0, -10.0, -10.0),
                upper_bound: Vec3::new(10.0, 10.0, 10.0),
            },
            |triangle| {
                visited += 1;
                triangle.triangle_index >= 0
            },
        );
        assert!(visited > 0);

        let path = std::env::temp_dir().join(format!(
            "box3d-height-field-{}-{}.bin",
            std::process::id(),
            grid.row_count()
        ));
        HeightField::dump_data(&path, &[0.0, 0.0, 0.0, 0.0], 2, 2, scale).unwrap();
        let loaded = HeightField::load(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(loaded.row_count(), 2);
        assert_eq!(loaded.column_count(), 2);

        let overlap_point = [Vec3::ZERO];
        let overlap_proxy = ShapeProxy::new(&overlap_point, 0.1).unwrap();
        assert!(overlap_height_field(
            &grid,
            Transform::IDENTITY,
            overlap_proxy
        ));
        assert!(ray_cast_height_field(
            &grid,
            RayCastInput::new(Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, -2.0, 0.0), 1.0),
        )
        .is_some());

        let cast_point = [Vec3::new(0.0, 1.0, 0.0)];
        let cast_proxy = ShapeProxy::new(&cast_point, 0.1).unwrap();
        assert!(shape_cast_height_field(
            &grid,
            ShapeCastInput::new(cast_proxy, Vec3::new(0.0, -2.0, 0.0), 1.0, false),
        )
        .is_some());
    }
}
