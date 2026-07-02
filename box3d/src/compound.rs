use std::{
    any::Any,
    ffi::c_void,
    mem::ManuallyDrop,
    panic::{catch_unwind, resume_unwind, AssertUnwindSafe},
    ptr,
    ptr::NonNull,
    slice,
};

use box3d_sys as sys;

use crate::{
    body::Body,
    collision::{compute_compound_aabb, Capsule, Sphere},
    handle,
    hull::HullRef,
    math::{Aabb, SurfaceMaterial, Transform, Vec3},
    mesh::Mesh,
    shape::{raw_shape_def, Shape, ShapeDef, ShapeType},
    Error, Result,
};

type CallbackPanic = Box<dyn Any + Send + 'static>;

pub struct Compound {
    raw: NonNull<sys::b3CompoundData>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompoundCapsule {
    pub capsule: Capsule,
    pub material_index: i32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompoundHull {
    pub transform: Transform,
    pub material_index: i32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompoundMesh {
    pub transform: Transform,
    pub scale: Vec3,
    pub material_indices: [i32; 4],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompoundSphere {
    pub sphere: Sphere,
    pub material_index: i32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CompoundChild {
    Capsule {
        capsule: Capsule,
        material_indices: [i32; 4],
    },
    Hull {
        transform: Transform,
        material_indices: [i32; 4],
    },
    Mesh {
        transform: Transform,
        scale: Vec3,
        material_indices: [i32; 4],
    },
    Sphere {
        sphere: Sphere,
        material_indices: [i32; 4],
    },
}

impl CompoundChild {
    pub fn shape_type(self) -> ShapeType {
        match self {
            Self::Capsule { .. } => ShapeType::Capsule,
            Self::Hull { .. } => ShapeType::Hull,
            Self::Mesh { .. } => ShapeType::Mesh,
            Self::Sphere { .. } => ShapeType::Sphere,
        }
    }
}

#[derive(Clone, Copy)]
pub enum CompoundPart<'a> {
    Sphere {
        center: Vec3,
        radius: f32,
        material: SurfaceMaterial,
    },
    Capsule {
        point1: Vec3,
        point2: Vec3,
        radius: f32,
        material: SurfaceMaterial,
    },
    Hull {
        hull: HullRef<'a>,
        transform: Transform,
        material: SurfaceMaterial,
    },
    Mesh {
        mesh: &'a Mesh,
        transform: Transform,
        scale: Vec3,
        material: SurfaceMaterial,
    },
}

impl Compound {
    pub fn new(parts: &[CompoundPart<'_>]) -> Result<Self> {
        if parts.is_empty() || parts.len() > i32::MAX as usize {
            return Err(Error::InvalidInput);
        }

        let mut capsules = Vec::new();
        let mut hulls = Vec::new();
        let mut meshes = Vec::new();
        let mut mesh_materials = Vec::with_capacity(parts.len());
        let mut spheres = Vec::new();

        for part in parts {
            match *part {
                CompoundPart::Sphere {
                    center,
                    radius,
                    material,
                } => {
                    if radius <= 0.0 {
                        return Err(Error::InvalidInput);
                    }
                    spheres.push(sys::b3CompoundSphereDef {
                        sphere: sys::b3Sphere {
                            center: center.into(),
                            radius,
                        },
                        material: material.into(),
                    });
                }
                CompoundPart::Capsule {
                    point1,
                    point2,
                    radius,
                    material,
                } => {
                    if radius <= 0.0 {
                        return Err(Error::InvalidInput);
                    }
                    capsules.push(sys::b3CompoundCapsuleDef {
                        capsule: sys::b3Capsule {
                            center1: point1.into(),
                            center2: point2.into(),
                            radius,
                        },
                        material: material.into(),
                    });
                }
                CompoundPart::Hull {
                    hull,
                    transform,
                    material,
                } => hulls.push(sys::b3CompoundHullDef {
                    hull: hull.raw(),
                    transform: transform.into(),
                    material: material.into(),
                }),
                CompoundPart::Mesh {
                    mesh,
                    transform,
                    scale,
                    material,
                } => {
                    if scale.x == 0.0 || scale.y == 0.0 || scale.z == 0.0 {
                        return Err(Error::InvalidInput);
                    }
                    mesh_materials.push(material.into());
                    let material = mesh_materials
                        .last()
                        .map(|material| material as *const sys::b3SurfaceMaterial)
                        .unwrap();
                    meshes.push(sys::b3CompoundMeshDef {
                        meshData: mesh.raw(),
                        transform: transform.into(),
                        scale: scale.into(),
                        materials: material,
                        materialCount: 1,
                    });
                }
            }
        }

        let def = sys::b3CompoundDef {
            capsules: as_mut_ptr(&mut capsules),
            capsuleCount: count(capsules.len())?,
            hulls: as_mut_ptr(&mut hulls),
            hullCount: count(hulls.len())?,
            meshes: as_mut_ptr(&mut meshes),
            meshCount: count(meshes.len())?,
            spheres: as_mut_ptr(&mut spheres),
            sphereCount: count(spheres.len())?,
        };
        let raw = NonNull::new(unsafe { sys::b3CreateCompound(&def) }).ok_or(Error::Null)?;

        Ok(Self { raw })
    }

    pub fn into_bytes(self) -> Vec<u8> {
        let this = ManuallyDrop::new(self);
        let raw = this.raw.as_ptr();
        let byte_count = unsafe { (*raw).byteCount as usize };
        let bytes = unsafe {
            let bytes = sys::b3ConvertCompoundToBytes(raw);
            slice::from_raw_parts(bytes, byte_count).to_vec()
        };
        unsafe { sys::b3DestroyCompound(raw) };
        bytes
    }

    pub fn byte_count(&self) -> i32 {
        unsafe { self.raw.as_ref().byteCount }
    }

    pub fn child_count(&self) -> i32 {
        self.capsule_count() + self.hull_count() + self.mesh_count() + self.sphere_count()
    }

    pub fn material_count(&self) -> i32 {
        unsafe { self.raw.as_ref().materialCount }
    }

    pub fn capsule_count(&self) -> i32 {
        unsafe { self.raw.as_ref().capsuleCount }
    }

    pub fn hull_count(&self) -> i32 {
        unsafe { self.raw.as_ref().hullCount }
    }

    pub fn mesh_count(&self) -> i32 {
        unsafe { self.raw.as_ref().meshCount }
    }

    pub fn sphere_count(&self) -> i32 {
        unsafe { self.raw.as_ref().sphereCount }
    }

    pub fn shared_hull_count(&self) -> i32 {
        unsafe { self.raw.as_ref().sharedHullCount }
    }

    pub fn shared_mesh_count(&self) -> i32 {
        unsafe { self.raw.as_ref().sharedMeshCount }
    }

    pub fn materials(&self) -> Vec<SurfaceMaterial> {
        let count = self.material_count();
        if count <= 0 {
            return Vec::new();
        }

        let materials = unsafe { sys::b3GetCompoundMaterials(self.raw()) };
        if materials.is_null() {
            Vec::new()
        } else {
            unsafe { slice::from_raw_parts(materials, count as usize) }
                .iter()
                .copied()
                .map(Into::into)
                .collect()
        }
    }

    pub fn child(&self, child_index: i32) -> Result<CompoundChild> {
        check_index(child_index, self.child_count())?;
        let child = unsafe { sys::b3GetCompoundChild(self.raw(), child_index) };
        Ok(match child.type_ {
            sys::b3ShapeType_b3_capsuleShape => CompoundChild::Capsule {
                capsule: unsafe { child.__bindgen_anon_1.capsule }.into(),
                material_indices: child.materialIndices,
            },
            sys::b3ShapeType_b3_hullShape => CompoundChild::Hull {
                transform: child.transform.into(),
                material_indices: child.materialIndices,
            },
            sys::b3ShapeType_b3_meshShape => {
                let mesh = unsafe { child.__bindgen_anon_1.mesh };
                CompoundChild::Mesh {
                    transform: child.transform.into(),
                    scale: mesh.scale.into(),
                    material_indices: child.materialIndices,
                }
            }
            sys::b3ShapeType_b3_sphereShape => CompoundChild::Sphere {
                sphere: unsafe { child.__bindgen_anon_1.sphere }.into(),
                material_indices: child.materialIndices,
            },
            _ => return Err(Error::InvalidInput),
        })
    }

    pub fn capsule(&self, index: i32) -> Result<CompoundCapsule> {
        check_index(index, self.capsule_count())?;
        let capsule = unsafe { sys::b3GetCompoundCapsule(self.raw(), index) };
        Ok(CompoundCapsule {
            capsule: capsule.capsule.into(),
            material_index: capsule.materialIndex,
        })
    }

    pub fn hull(&self, index: i32) -> Result<CompoundHull> {
        check_index(index, self.hull_count())?;
        let hull = unsafe { sys::b3GetCompoundHull(self.raw(), index) };
        Ok(CompoundHull {
            transform: hull.transform.into(),
            material_index: hull.materialIndex,
        })
    }

    pub fn mesh(&self, index: i32) -> Result<CompoundMesh> {
        check_index(index, self.mesh_count())?;
        let mesh = unsafe { sys::b3GetCompoundMesh(self.raw(), index) };
        Ok(CompoundMesh {
            transform: mesh.transform.into(),
            scale: mesh.scale.into(),
            material_indices: mesh.materialIndices,
        })
    }

    pub fn sphere(&self, index: i32) -> Result<CompoundSphere> {
        check_index(index, self.sphere_count())?;
        let sphere = unsafe { sys::b3GetCompoundSphere(self.raw(), index) };
        Ok(CompoundSphere {
            sphere: sphere.sphere.into(),
            material_index: sphere.materialIndex,
        })
    }

    pub fn query<F>(&self, aabb: Aabb, mut f: F)
    where
        F: FnMut(i32) -> bool,
    {
        assert_valid_aabb(aabb);
        let mut context = CompoundQueryContext {
            f: &mut f,
            panic: None,
        };
        unsafe {
            sys::b3QueryCompound(
                self.raw(),
                aabb.into(),
                Some(compound_query_callback::<F>),
                (&mut context as *mut CompoundQueryContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
    }

    pub fn compute_aabb(&self, transform: Transform) -> Aabb {
        compute_compound_aabb(self, transform)
    }

    pub(crate) fn raw(&self) -> *const sys::b3CompoundData {
        self.raw.as_ptr()
    }
}

impl Drop for Compound {
    fn drop(&mut self) {
        unsafe { sys::b3DestroyCompound(self.raw.as_ptr()) };
    }
}

impl Body<'_> {
    pub fn create_compound<'a>(&'a self, compound: &'a Compound, def: ShapeDef) -> Shape<'a> {
        let mut raw_def = raw_shape_def(def);
        let raw = handle::shape(unsafe {
            sys::b3CreateCompoundShape(self.raw(), &mut raw_def, compound.raw())
        })
        .expect("box3d returned an invalid shape");

        Shape::from_raw(raw)
    }
}

fn as_mut_ptr<T>(values: &mut Vec<T>) -> *mut T {
    if values.is_empty() {
        ptr::null_mut()
    } else {
        values.as_mut_ptr()
    }
}

fn count(count: usize) -> Result<i32> {
    i32::try_from(count).map_err(|_| Error::InvalidInput)
}

fn check_index(index: i32, count: i32) -> Result<()> {
    if 0 <= index && index < count {
        Ok(())
    } else {
        Err(Error::InvalidInput)
    }
}

struct CompoundQueryContext<'a, F> {
    f: &'a mut F,
    panic: Option<CallbackPanic>,
}

unsafe extern "C" fn compound_query_callback<F>(
    _compound: *const sys::b3CompoundData,
    child_index: i32,
    context: *mut c_void,
) -> bool
where
    F: FnMut(i32) -> bool,
{
    let context = unsafe { &mut *(context as *mut CompoundQueryContext<'_, F>) };
    if context.panic.is_some() {
        return false;
    }

    match catch_unwind(AssertUnwindSafe(|| (context.f)(child_index))) {
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

fn assert_valid_aabb(aabb: Aabb) {
    assert_valid_vec3(aabb.lower_bound);
    assert_valid_vec3(aabb.upper_bound);
    assert!(aabb.lower_bound.x <= aabb.upper_bound.x);
    assert!(aabb.lower_bound.y <= aabb.upper_bound.y);
    assert!(aabb.lower_bound.z <= aabb.upper_bound.z);
}

fn assert_valid_vec3(value: Vec3) {
    assert!(value.x.is_finite() && value.y.is_finite() && value.z.is_finite());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        overlap_compound, ray_cast_compound, shape_cast_compound, BodyDef, BoxHull, Mesh, Quat,
        RayCastInput, ShapeCastInput, ShapeProxy, ShapeType, World,
    };

    #[test]
    fn compound_two_boxes_supports_dynamic_sphere() {
        let world = World::default();
        let compound_body = world.create_body(BodyDef::static_at(Vec3::ZERO));
        let box_hull = BoxHull::new(Vec3::new(1.0, 0.5, 1.0));
        let material = SurfaceMaterial::default();
        let compound = Compound::new(&[
            CompoundPart::Hull {
                hull: (&box_hull).into(),
                transform: Transform::new(Vec3::new(-1.0, 0.0, 0.0), crate::Quat::IDENTITY),
                material,
            },
            CompoundPart::Hull {
                hull: (&box_hull).into(),
                transform: Transform::new(Vec3::new(1.0, 0.0, 0.0), crate::Quat::IDENTITY),
                material,
            },
        ])
        .unwrap();
        let compound_shape = compound_body.create_compound(&compound, ShapeDef::default());

        let sphere_body = world.create_body(BodyDef::dynamic_at(Vec3::new(-1.0, 4.0, 0.0)));
        let sphere_shape = sphere_body.create_sphere(
            Vec3::ZERO,
            0.5,
            ShapeDef {
                density: 1.0,
                friction: 0.3,
                ..ShapeDef::default()
            },
        );

        for _ in 0..120 {
            world.step(1.0 / 60.0, 4);
        }

        assert!(compound_shape.is_valid());
        assert!(sphere_shape.is_valid());
        assert!(
            (sphere_body.position().y - 1.0).abs() < 0.05,
            "{:?}",
            sphere_body.position()
        );
    }

    #[test]
    fn compound_inspection_query_aabb_and_casts() {
        let box_hull = BoxHull::cube(0.25);
        let mesh = Mesh::box_mesh(Vec3::ZERO, Vec3::new(0.25, 0.25, 0.25), true);
        let material = SurfaceMaterial::default();
        let compound = Compound::new(&[
            CompoundPart::Capsule {
                point1: Vec3::new(-0.5, 0.0, 0.0),
                point2: Vec3::new(0.5, 0.0, 0.0),
                radius: 0.25,
                material,
            },
            CompoundPart::Hull {
                hull: (&box_hull).into(),
                transform: Transform::new(Vec3::new(2.0, 0.0, 0.0), Quat::IDENTITY),
                material,
            },
            CompoundPart::Mesh {
                mesh: &mesh,
                transform: Transform::new(Vec3::new(-2.0, 0.0, 0.0), Quat::IDENTITY),
                scale: Vec3::new(1.0, 1.0, 1.0),
                material,
            },
            CompoundPart::Sphere {
                center: Vec3::new(0.0, 1.0, 0.0),
                radius: 0.25,
                material,
            },
        ])
        .unwrap();

        assert_eq!(compound.child_count(), 4);
        assert_eq!(compound.capsule_count(), 1);
        assert_eq!(compound.hull_count(), 1);
        assert_eq!(compound.mesh_count(), 1);
        assert_eq!(compound.sphere_count(), 1);
        assert!(!compound.materials().is_empty());
        assert_eq!(compound.capsule(0).unwrap().capsule.radius, 0.25);
        assert_eq!(
            compound.hull(0).unwrap().transform.p,
            Vec3::new(2.0, 0.0, 0.0)
        );
        assert_eq!(compound.mesh(0).unwrap().scale, Vec3::new(1.0, 1.0, 1.0));
        assert_eq!(compound.sphere(0).unwrap().sphere.radius, 0.25);
        assert_eq!(compound.child(0).unwrap().shape_type(), ShapeType::Capsule);
        assert_eq!(compound.child(1).unwrap().shape_type(), ShapeType::Hull);
        assert_eq!(compound.child(2).unwrap().shape_type(), ShapeType::Mesh);
        assert_eq!(compound.child(3).unwrap().shape_type(), ShapeType::Sphere);

        let mut children = Vec::new();
        compound.query(
            Aabb {
                lower_bound: Vec3::new(-3.0, -1.0, -1.0),
                upper_bound: Vec3::new(3.0, 2.0, 1.0),
            },
            |child_index| {
                children.push(child_index);
                true
            },
        );
        children.sort_unstable();
        assert_eq!(children, vec![0, 1, 2, 3]);

        let aabb = compound.compute_aabb(Transform::IDENTITY);
        assert!(aabb.lower_bound.x < -2.0);
        assert!(aabb.upper_bound.x > 2.0);

        let overlap_point = [Vec3::ZERO];
        let overlap_proxy = ShapeProxy::new(&overlap_point, 0.1).unwrap();
        assert!(overlap_compound(
            &compound,
            Transform::IDENTITY,
            overlap_proxy
        ));
        assert!(ray_cast_compound(
            &compound,
            RayCastInput::new(Vec3::new(-4.0, 0.0, 0.0), Vec3::new(8.0, 0.0, 0.0), 1.0),
        )
        .is_some());

        let cast_point = [Vec3::new(-4.0, 0.0, 0.0)];
        let cast_proxy = ShapeProxy::new(&cast_point, 0.1).unwrap();
        assert!(shape_cast_compound(
            &compound,
            ShapeCastInput::new(cast_proxy, Vec3::new(8.0, 0.0, 0.0), 1.0, false),
        )
        .is_some());
    }

    #[test]
    fn compound_into_bytes_consumes_native_compound() {
        let compound = Compound::new(&[CompoundPart::Sphere {
            center: Vec3::ZERO,
            radius: 0.5,
            material: SurfaceMaterial::default(),
        }])
        .unwrap();
        let byte_count = compound.byte_count() as usize;
        let bytes = compound.into_bytes();

        assert_eq!(bytes.len(), byte_count);
        assert!(bytes.len() >= std::mem::size_of::<usize>());
    }
}
