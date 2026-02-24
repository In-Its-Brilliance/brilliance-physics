use super::bridge::{network_to_physx, physx_to_network};
use super::controller::PhysxPhysicsController;
use crate::physics::{IPhysicsCollider, IPhysicsShape};
use common::chunks::position::Vector3;
use parking_lot::RwLock;
use physx::actor::Actor;
use physx::math::PxTransform;
use physx::shape::Shape;
use physx::traits::Class;
use physx::{owner::Owner, prelude::RigidActor};
use physx_sys::{
    PxBoxGeometry, PxCapsuleGeometry, PxGeometry_getType, PxGeometryType,
    PxScene_removeActor_mut, PxShape_getGeometry, PxShape_getQueryFilterData,
    PxShape_setFlag_mut, PxShape_setGeometry_mut, PxShape_setQueryFilterData_mut,
    PxShape_setSimulationFilterData_mut, PxShapeFlag, PxTriangleMeshGeometry,
};
use std::sync::Arc;

use super::types::{PxRigidStatic, PxShape};

// ---- PhysxPhysicsShape ----

/// Represents PhysX geometry in a transferable form.
/// `Ready` variants contain concrete geometry structs that can be passed to PhysX APIs.
/// `Deferred` variant holds raw trimesh data that needs cooking before use.
pub enum PhysxPhysicsShapeInner {
    Ready(PhysxGeometryKind),
    Deferred {
        verts: Vec<physx_sys::PxVec3>,
        indices: Vec<[u32; 3]>,
    },
}

/// Concrete PhysX geometry types (all are Copy/POD structs from physx-sys).
#[derive(Clone, Copy)]
pub enum PhysxGeometryKind {
    Box(PxBoxGeometry),
    Capsule(PxCapsuleGeometry),
    TriangleMesh(PxTriangleMeshGeometry),
}

impl PhysxGeometryKind {
    /// Returns a raw pointer to the geometry as `*const PxGeometry`,
    /// suitable for passing to PhysX C API functions (sweep, setGeometry, etc.).
    pub(crate) fn as_geometry_ptr(&self) -> *const physx_sys::PxGeometry {
        match self {
            PhysxGeometryKind::Box(g) => g as *const PxBoxGeometry as *const physx_sys::PxGeometry,
            PhysxGeometryKind::Capsule(g) => {
                g as *const PxCapsuleGeometry as *const physx_sys::PxGeometry
            }
            PhysxGeometryKind::TriangleMesh(g) => {
                g as *const PxTriangleMeshGeometry as *const physx_sys::PxGeometry
            }
        }
    }
}

pub struct PhysxPhysicsShape {
    pub(crate) inner: PhysxPhysicsShapeInner,
}

impl PhysxPhysicsShape {
    pub(crate) fn ready(kind: PhysxGeometryKind) -> Self {
        Self {
            inner: PhysxPhysicsShapeInner::Ready(kind),
        }
    }

    pub(crate) fn deferred(verts: Vec<physx_sys::PxVec3>, indices: Vec<[u32; 3]>) -> Self {
        Self {
            inner: PhysxPhysicsShapeInner::Deferred { verts, indices },
        }
    }

}

impl IPhysicsShape for PhysxPhysicsShape {}

/// Extracts the geometry kind from a PxShape by reading the geometry type
/// and copying the appropriate concrete geometry struct.
pub(crate) unsafe fn extract_geometry_kind(
    shape_ptr: *const physx_sys::PxShape,
) -> PhysxGeometryKind {
    let geom_ptr = PxShape_getGeometry(shape_ptr);
    let geom_type = PxGeometry_getType(geom_ptr);
    match geom_type {
        PxGeometryType::Box => {
            let box_geom = *(geom_ptr as *const PxBoxGeometry);
            PhysxGeometryKind::Box(box_geom)
        }
        PxGeometryType::Capsule => {
            let cap_geom = *(geom_ptr as *const PxCapsuleGeometry);
            PhysxGeometryKind::Capsule(cap_geom)
        }
        PxGeometryType::Trianglemesh => {
            let tri_geom = *(geom_ptr as *const PxTriangleMeshGeometry);
            PhysxGeometryKind::TriangleMesh(tri_geom)
        }
        _ => panic!(
            "PhysxPhysicsShape: unsupported geometry type: {:?}",
            geom_type
        ),
    }
}

// ---- PhysxPhysicsCollider ----

pub struct PhysxPhysicsCollider {
    pub(crate) controller: Arc<RwLock<PhysxPhysicsController>>,
    pub(crate) actor: RwLock<Owner<PxRigidStatic>>,
    shape: RwLock<Owner<PxShape>>,
}

impl PhysxPhysicsCollider {
    pub(crate) fn create(
        controller: Arc<RwLock<PhysxPhysicsController>>,
        actor: Owner<PxRigidStatic>,
        shape: Owner<PxShape>,
    ) -> Self {
        Self {
            controller,
            actor: RwLock::new(actor),
            shape: RwLock::new(shape),
        }
    }
}

impl IPhysicsCollider<PhysxPhysicsShape> for PhysxPhysicsCollider {
    fn get_position(&self) -> Vector3 {
        let actor = self.actor.read();
        physx_to_network(&actor.get_global_position())
    }

    fn set_position(&mut self, position: Vector3) {
        let mut actor = self.actor.write();
        actor.set_global_pose(
            &PxTransform::from_translation(&network_to_physx(&position)),
            true,
        );
    }

    fn set_enabled(&mut self, active: bool) {
        let mut actor = self.actor.write();
        actor.enable_gravity(active);
    }

    fn get_index(&self) -> usize {
        let shape = self.shape.read();
        *shape.get_user_data()
    }

    fn remove(&self) {
        let mut controller = self.controller.write();
        let mut actor = self.actor.write();
        unsafe {
            PxScene_removeActor_mut(controller.scene.as_mut_ptr(), actor.as_mut_ptr(), false);
        }
    }

    fn get_shape(&self) -> PhysxPhysicsShape {
        let shape = self.shape.read();
        let kind = unsafe { extract_geometry_kind(shape.as_ptr()) };
        PhysxPhysicsShape::ready(kind)
    }

    fn set_shape(&mut self, new_shape: PhysxPhysicsShape) {
        match new_shape.inner {
            PhysxPhysicsShapeInner::Ready(kind) => {
                let mut shape = self.shape.write();
                unsafe {
                    PxShape_setGeometry_mut(shape.as_mut_ptr(), kind.as_geometry_ptr());
                }
            }
            PhysxPhysicsShapeInner::Deferred { verts, indices } => {
                let mut controller = self.controller.write();
                let mut shape = self.shape.write();
                super::collider_builder::cook_and_apply_trimesh(
                    &mut controller,
                    shape.as_mut_ptr(),
                    &verts,
                    &indices,
                );
            }
        }
    }

    fn set_sensor(&self, is_sensor: bool) {
        let mut shape = self.shape.write();
        unsafe {
            // PhysX does not support eTRIGGER_SHAPE on triangle meshes.
            // We disable simulation and mark in filter data as a universal approach.
            PxShape_setFlag_mut(
                shape.as_mut_ptr(),
                PxShapeFlag::SimulationShape,
                !is_sensor,
            );
            // Also remove sensors from scene queries (sweep/raycast) entirely,
            // so they never appear as closest-hit blocking valid solid geometry behind them.
            PxShape_setFlag_mut(
                shape.as_mut_ptr(),
                PxShapeFlag::SceneQueryShape,
                !is_sensor,
            );
            let mut filter_data = PxShape_getQueryFilterData(shape.as_ptr());
            filter_data.word2 = if is_sensor { 1 } else { 0 };
            PxShape_setQueryFilterData_mut(shape.as_mut_ptr(), &filter_data);
            PxShape_setSimulationFilterData_mut(shape.as_mut_ptr(), &filter_data);
        }
    }

    fn set_collision_mask(&self, groups: u32, mask: u32) {
        let mut shape = self.shape.write();
        unsafe {
            let mut filter_data = PxShape_getQueryFilterData(shape.as_ptr());
            filter_data.word0 = groups;
            filter_data.word1 = mask;
            // Preserve word2 (sensor flag)
            PxShape_setSimulationFilterData_mut(shape.as_mut_ptr(), &filter_data);
            PxShape_setQueryFilterData_mut(shape.as_mut_ptr(), &filter_data);
        }
    }
}
