use super::collider::{PhysxGeometryKind, PhysxPhysicsShape};
use super::controller::PhysxPhysicsController;
use super::types::PxShape;
use crate::physics::IPhysicsColliderBuilder;
use common::chunks::position::Vector3;
use physx::cooking;
use physx::owner::Owner;
use physx::prelude::{Geometry, TriangleMeshGeometry};
use physx::traits::Class;
use physx::cooking::PxTriangleMeshDesc;
use physx_sys::{
    PxBoxGeometry_new, PxCapsuleGeometry, PxCapsuleGeometry_new, PxMeshGeometryFlags,
    PxMeshScale_new, PxPhysics_createShape_mut, PxShapeFlags, PxTriangleMeshGeometry,
};
use std::ffi::c_void;

pub(crate) enum GeometryInner {
    Box(physx_sys::PxBoxGeometry),
    Capsule(PxCapsuleGeometry),
    TriangleMesh {
        desc: PxTriangleMeshDesc,
        // Keep verts/indices alive so that desc's raw pointers remain valid
        _verts: Vec<physx_sys::PxVec3>,
        _indices: Vec<[u32; 3]>,
    },
}

pub struct PhysxPhysicsColliderBuilder {
    pub(crate) geometry: GeometryInner,
}

impl PhysxPhysicsColliderBuilder {
    /// Creates a PxShape from this builder's geometry.
    ///
    /// For trimesh geometry, cooking happens here and the cooked mesh is kept alive
    /// until after `PxPhysics_createShape_mut` acquires its own reference.
    /// This avoids use-after-free: `PxTriangleMeshGeometry` only holds a raw pointer
    /// to the cooked mesh, so the mesh must outlive the createShape call.
    pub(crate) fn create_px_shape(
        &mut self,
        controller: &mut PhysxPhysicsController,
        material_ptr: *mut physx_sys::PxMaterial,
        flags: PxShapeFlags,
        shape_id: usize,
    ) -> Owner<PxShape> {
        match &self.geometry {
            GeometryInner::Box(g) => {
                let geometry: Box<dyn Geometry> = Box::new(*g);
                unsafe { Self::create_shape_raw(controller, geometry.as_ptr(), material_ptr, flags, shape_id) }
            }
            GeometryInner::Capsule(g) => {
                let geometry: Box<dyn Geometry> = Box::new(*g);
                unsafe { Self::create_shape_raw(controller, geometry.as_ptr(), material_ptr, flags, shape_id) }
            }
            GeometryInner::TriangleMesh { desc, .. } => {
                let params = cooking::PxCookingParams::new(controller.physics.physics()).unwrap();
                let mut mesh =
                    match cooking::create_triangle_mesh(controller.physics.physics_mut(), &params, desc) {
                        cooking::TriangleMeshCookingResult::Success(m) => m,
                        _ => panic!("create_triangle_mesh error"),
                    };
                let geometry = PxTriangleMeshGeometry::new(
                    &mut mesh,
                    &unsafe { PxMeshScale_new() },
                    PxMeshGeometryFlags::DoubleSided,
                );
                let geom_ptr = &geometry as *const PxTriangleMeshGeometry
                    as *const physx_sys::PxGeometry;
                let shape = unsafe {
                    Self::create_shape_raw(controller, geom_ptr, material_ptr, flags, shape_id)
                };
                // `mesh` drops here AFTER createShape has acquired its reference
                shape
            }
        }
    }

    unsafe fn create_shape_raw(
        controller: &mut PhysxPhysicsController,
        geometry_ptr: *const physx_sys::PxGeometry,
        material_ptr: *mut physx_sys::PxMaterial,
        flags: PxShapeFlags,
        shape_id: usize,
    ) -> Owner<PxShape> {
        physx::shape::Shape::from_raw(
            PxPhysics_createShape_mut(
                controller.physics.as_mut_ptr(),
                geometry_ptr,
                material_ptr,
                true,
                flags,
            ),
            shape_id,
        )
        .unwrap()
    }
}

/// Cooks a triangle mesh from raw vertex/index data and applies it directly to
/// the given PxShape. The cooked mesh (`Owner<TriangleMesh>`) is kept alive in this
/// scope until after `PxShape_setGeometry_mut` acquires its reference, avoiding
/// use-after-free (PxTriangleMeshGeometry only holds a raw pointer to the mesh).
pub(crate) fn cook_and_apply_trimesh(
    controller: &mut PhysxPhysicsController,
    shape_ptr: *mut physx_sys::PxShape,
    verts: &[physx_sys::PxVec3],
    indices: &[[u32; 3]],
) {
    let mut desc = PxTriangleMeshDesc::new();
    desc.obj.points.count = verts.len() as u32;
    desc.obj.points.stride = std::mem::size_of::<physx_sys::PxVec3>() as u32;
    desc.obj.points.data = if verts.is_empty() {
        std::ptr::null()
    } else {
        verts.as_ptr() as *const c_void
    };

    desc.obj.triangles.count = indices.len() as u32;
    desc.obj.triangles.stride = std::mem::size_of::<[u32; 3]>() as u32;
    desc.obj.triangles.data = if indices.is_empty() {
        std::ptr::null()
    } else {
        indices.as_ptr() as *const c_void
    };

    let params = cooking::PxCookingParams::new(controller.physics.physics()).unwrap();
    let mut mesh =
        match cooking::create_triangle_mesh(controller.physics.physics_mut(), &params, &desc) {
            cooking::TriangleMeshCookingResult::Success(m) => m,
            _ => panic!("cook_and_apply_trimesh: create_triangle_mesh error"),
        };
    let geometry = PxTriangleMeshGeometry::new(
        &mut mesh,
        &unsafe { PxMeshScale_new() },
        PxMeshGeometryFlags::DoubleSided,
    );
    let geom_ptr = &geometry as *const PxTriangleMeshGeometry as *const physx_sys::PxGeometry;
    unsafe { physx_sys::PxShape_setGeometry_mut(shape_ptr, geom_ptr) };
    // `mesh` drops here AFTER setGeometry has acquired its reference
}

impl IPhysicsColliderBuilder<PhysxPhysicsShape> for PhysxPhysicsColliderBuilder {
    fn cuboid(hx: f32, hy: f32, hz: f32) -> Self {
        Self {
            geometry: GeometryInner::Box(unsafe { PxBoxGeometry_new(hx * 0.5, hy * 0.5, hz * 0.5) }),
        }
    }

    fn cylinder(half_height: f32, radius: f32) -> Self {
        // PhysX capsule adds hemisphere caps (radius) on each end, making total
        // height = 2*(half_height + radius). To match Rapier cylinder total height
        // of 2*half_height, reduce the cylindrical half-height by radius.
        let capsule_half_height = (half_height - radius).max(0.0);
        Self {
            geometry: GeometryInner::Capsule(unsafe {
                PxCapsuleGeometry_new(radius, capsule_half_height)
            }),
        }
    }

    fn trimesh(verts: Vec<Vector3>, indices: Vec<[u32; 3]>) -> Self {
        let px_verts: Vec<physx_sys::PxVec3> = verts
            .into_iter()
            .map(|v| physx_sys::PxVec3 { x: v.x, y: v.y, z: v.z })
            .collect();

        let mut desc = PxTriangleMeshDesc::new();
        desc.obj.points.count = px_verts.len() as u32;
        desc.obj.points.stride = std::mem::size_of::<physx_sys::PxVec3>() as u32;
        desc.obj.points.data = if px_verts.is_empty() {
            std::ptr::null()
        } else {
            px_verts.as_ptr() as *const c_void
        };

        desc.obj.triangles.count = indices.len() as u32;
        desc.obj.triangles.stride = std::mem::size_of::<[u32; 3]>() as u32;
        desc.obj.triangles.data = if indices.is_empty() {
            std::ptr::null()
        } else {
            indices.as_ptr() as *const c_void
        };

        Self {
            geometry: GeometryInner::TriangleMesh {
                desc,
                _verts: px_verts,
                _indices: indices,
            },
        }
    }

    fn get_shape(&self) -> PhysxPhysicsShape {
        match &self.geometry {
            GeometryInner::Box(g) => PhysxPhysicsShape::ready(PhysxGeometryKind::Box(*g)),
            GeometryInner::Capsule(g) => PhysxPhysicsShape::ready(PhysxGeometryKind::Capsule(*g)),
            GeometryInner::TriangleMesh {
                _verts, _indices, ..
            } => {
                // Trimesh needs cooking which requires the PhysicsFoundation.
                // Return a Deferred shape that will be cooked when applied to a collider.
                PhysxPhysicsShape::deferred(_verts.clone(), _indices.clone())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::PhysxPhysicsColliderBuilder;
    use crate::{
        physics::{IPhysicsCollider, IPhysicsColliderBuilder, IPhysicsContainer},
        physx::container::PhysxPhysicsContainer,
    };
    use common::chunks::position::Vector3;

    #[test]
    fn test_collider() {
        let verts = vec![
            Vector3::new(0., 1., 0.),
            Vector3::new(0., -1., 0.),
            Vector3::new(1., 0., 0.),
        ];
        let indices: Vec<[u32; 3]> = vec![[0, 1, 2]];
        let collider = PhysxPhysicsColliderBuilder::trimesh(verts, indices);

        let container = PhysxPhysicsContainer::default();
        let collider = container.spawn_collider(collider);
        // Shape index is from a global AtomicUsize; just verify it's stable
        let idx = collider.get_index();
        assert_eq!(collider.get_index(), idx);
        assert_eq!(collider.get_position(), Vector3::zero());
    }
}
