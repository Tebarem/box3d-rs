use std::{ptr, ptr::NonNull};

use box3d_sys as sys;

use crate::{
    body::Body,
    handle,
    hull::HullRef,
    math::{SurfaceMaterial, Transform, Vec3},
    mesh::Mesh,
    shape::{raw_shape_def, Shape, ShapeDef},
    Error, Result,
};

pub struct Compound {
    raw: NonNull<sys::b3CompoundData>,
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

        let mut def = sys::b3CompoundDef {
            capsules: as_mut_ptr(&mut capsules),
            capsuleCount: count(capsules.len())?,
            hulls: as_mut_ptr(&mut hulls),
            hullCount: count(hulls.len())?,
            meshes: as_mut_ptr(&mut meshes),
            meshCount: count(meshes.len())?,
            spheres: as_mut_ptr(&mut spheres),
            sphereCount: count(spheres.len())?,
        };
        let raw = NonNull::new(unsafe { sys::b3CreateCompound(&mut def) }).ok_or(Error::Null)?;

        Ok(Self { raw })
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyDef, BoxHull, World};

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
}
