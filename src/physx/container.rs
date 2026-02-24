use super::{
    bridge::{network_to_physx_sys, physx_sys_to_network, CAPSULE_ROTATION},
    collider::{PhysxPhysicsCollider, PhysxPhysicsShape, PhysxPhysicsShapeInner, PhysxGeometryKind},
    controller::PhysxPhysicsController,
    query_filter::PhysxQueryFilter,
    types::PxShape,
};
use crate::physics::{IPhysicsContainer, RayCastResultNormal, ShapeCastResult};
use common::{chunks::position::Vector3, utils::debug::info::DebugInfo};
use parking_lot::RwLock;
use physx::{
    math::{PxTransform, PxVec3},
    prelude::{Physics, RigidActor},
    traits::Class,
};
use physx::shape::Shape;
use physx_sys::{
    PxActorTypeFlags, PxHitFlags, PxScene_addActor_mut,
    PxScene_getNbActors, PxSceneQueryExt_raycastMultiple, PxSceneQueryExt_sweepMultiple,
    PxShape_setLocalPose_mut, PxShapeFlags,
};
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::{ptr::null_mut, time::Instant};

use super::collider_builder::{GeometryInner, PhysxPhysicsColliderBuilder};

static NEXT_SHAPE_ID: AtomicUsize = AtomicUsize::new(0);

#[derive(Clone)]
pub struct PhysxPhysicsContainer {
    controller: Arc<RwLock<PhysxPhysicsController>>,
    last_step_duration: Arc<RwLock<std::time::Duration>>,
}

impl Default for PhysxPhysicsContainer {
    fn default() -> Self {
        let controller = Arc::new(RwLock::new(PhysxPhysicsController::create()));
        Self {
            controller,
            last_step_duration: Arc::new(RwLock::new(std::time::Duration::ZERO)),
        }
    }
}

impl
    IPhysicsContainer<
        PhysxPhysicsShape,
        PhysxPhysicsCollider,
        PhysxPhysicsColliderBuilder,
        PhysxQueryFilter,
    > for PhysxPhysicsContainer
{
    fn step(&self, delta: f32) {
        let start = Instant::now();
        self.controller.write().step(delta, self);
        *self.last_step_duration.write() = start.elapsed();
    }

    fn spawn_collider(
        &self,
        mut collider_builder: PhysxPhysicsColliderBuilder,
    ) -> PhysxPhysicsCollider {
        let mut controller = self.controller.write();

        let mut actor = controller
            .physics
            .create_static(
                PxTransform::from_translation(&PxVec3::new(0.0, 0.0, 0.0)),
                (),
            )
            .unwrap();
        unsafe {
            PxScene_addActor_mut(
                controller.scene.as_mut_ptr(),
                actor.as_mut_ptr(),
                std::ptr::null(),
            );
        }

        let mut material = controller
            .physics
            .create_material(0.0, 0.0, 0.0, ())
            .unwrap();
        let flags = PxShapeFlags::SceneQueryShape
            | PxShapeFlags::SimulationShape
            | PxShapeFlags::Visualization;
        let shape_id = NEXT_SHAPE_ID.fetch_add(1, Ordering::Relaxed);
        let mut shape = collider_builder.create_px_shape(
            &mut *controller,
            material.as_mut_ptr(),
            flags,
            shape_id,
        );
        // PhysX capsules are X-axis aligned; rotate 90° around Z for vertical (Y-axis)
        if matches!(collider_builder.geometry, GeometryInner::Capsule(_)) {
            let capsule_pose = physx_sys::PxTransform {
                p: physx_sys::PxVec3 { x: 0.0, y: 0.0, z: 0.0 },
                q: CAPSULE_ROTATION,
            };
            unsafe { PxShape_setLocalPose_mut(shape.as_mut_ptr(), &capsule_pose) };
        }
        actor.attach_shape(&mut shape);
        PhysxPhysicsCollider::create(self.controller.clone(), actor, shape)
    }

    fn cast_ray(
        &self,
        origin: Vector3,
        dir: Vector3,
        max_toi: f32,
        filter: PhysxQueryFilter,
    ) -> Option<RayCastResultNormal> {
        const MAX_HITS: usize = 16;
        let controller = self.controller.read();

        let mut hit_buffer: [MaybeUninit<physx_sys::PxRaycastHit>; MAX_HITS] =
            unsafe { MaybeUninit::uninit().assume_init() };
        let mut blocking_hit = false;

        let callback_ptr = filter.pre_filter_callback.unwrap_or(null_mut());

        let num_hits = unsafe {
            PxSceneQueryExt_raycastMultiple(
                controller.scene.as_ptr(),
                &network_to_physx_sys(&origin),
                &network_to_physx_sys(&dir),
                max_toi,
                PxHitFlags::Default | PxHitFlags::Position | PxHitFlags::Normal,
                hit_buffer[0].as_mut_ptr(),
                MAX_HITS as u32,
                &mut blocking_hit,
                &filter.filter as *const _,
                callback_ptr,
                std::ptr::null(),
            )
        };

        if num_hits <= 0 {
            return None;
        }

        // Collect hits, sort by distance, return first that passes post-filter
        let hits: &[physx_sys::PxRaycastHit] =
            unsafe { std::slice::from_raw_parts(hit_buffer[0].as_ptr(), num_hits as usize) };

        let mut sorted_indices: Vec<usize> = (0..num_hits as usize).collect();
        sorted_indices.sort_by(|&a, &b| hits[a].distance.partial_cmp(&hits[b].distance).unwrap());

        for &i in &sorted_indices {
            let h = &hits[i];
            let collider_id = unsafe { get_shape_id_from_ptr(h.shape) };
            if filter.post_filter_hit(h.shape as *const _, collider_id) {
                return Some(RayCastResultNormal {
                    collider_id,
                    point: physx_sys_to_network(&h.position),
                    normal: physx_sys_to_network(&h.normal),
                });
            }
        }

        None
    }

    fn cast_shape(
        &self,
        shape: PhysxPhysicsShape,
        origin: Vector3,
        dir: Vector3,
        max_toi: f32,
        filter: PhysxQueryFilter,
    ) -> Option<ShapeCastResult> {
        const MAX_HITS: usize = 16;
        let geometry_kind = match &shape.inner {
            PhysxPhysicsShapeInner::Ready(kind) => kind,
            PhysxPhysicsShapeInner::Deferred { .. } => {
                panic!("cast_shape: cannot use uncooked Deferred shape for sweep query")
            }
        };

        let controller = self.controller.read();

        // Use capsule rotation for capsule geometry, identity for others
        let rotation = if matches!(geometry_kind, PhysxGeometryKind::Capsule(_)) {
            CAPSULE_ROTATION
        } else {
            physx_sys::PxQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        };
        let pose = physx_sys::PxTransform {
            q: rotation,
            p: network_to_physx_sys(&origin),
        };

        let dir_sys = network_to_physx_sys(&dir);

        let mut hit_buffer: [MaybeUninit<physx_sys::PxSweepHit>; MAX_HITS] =
            unsafe { MaybeUninit::uninit().assume_init() };
        let mut blocking_hit = false;

        let callback_ptr = filter.pre_filter_callback.unwrap_or(null_mut());

        let num_hits = unsafe {
            PxSceneQueryExt_sweepMultiple(
                controller.scene.as_ptr(),
                geometry_kind.as_geometry_ptr(),
                &pose,
                &dir_sys,
                max_toi,
                PxHitFlags::Default | PxHitFlags::Position | PxHitFlags::Normal,
                hit_buffer[0].as_mut_ptr(),
                MAX_HITS as u32,
                &mut blocking_hit,
                &filter.filter as *const _,
                callback_ptr,
                std::ptr::null(),
                0.0,
            )
        };

        if num_hits <= 0 {
            return None;
        }

        let hits: &[physx_sys::PxSweepHit] =
            unsafe { std::slice::from_raw_parts(hit_buffer[0].as_ptr(), num_hits as usize) };

        let mut sorted_indices: Vec<usize> = (0..num_hits as usize).collect();
        sorted_indices.sort_by(|&a, &b| hits[a].distance.partial_cmp(&hits[b].distance).unwrap());

        for &i in &sorted_indices {
            let h = &hits[i];
            let collider_id = unsafe { get_shape_id_from_ptr(h.shape) };
            if filter.post_filter_hit(h.shape as *const _, collider_id) {
                let point = Vector3::new(
                    origin.x + dir.x * h.distance,
                    origin.y + dir.y * h.distance,
                    origin.z + dir.z * h.distance,
                );
                return Some(ShapeCastResult {
                    collider_id,
                    point,
                });
            }
        }

        None
    }

    fn get_debug_info(&self) -> DebugInfo {
        let controller = self.controller.read();
        let step_dur = *self.last_step_duration.read();

        let nb_static = unsafe {
            PxScene_getNbActors(controller.scene.as_ptr(), PxActorTypeFlags::RigidStatic)
        };

        DebugInfo::new()
            .insert("last_step_dur", step_dur)
            .insert("static_actors", nb_static as usize)
    }
}

unsafe fn get_shape_id_from_ptr(shape: *const physx_sys::PxShape) -> usize {
    let shape = &*(shape as *const PxShape);
    *shape.get_user_data()
}

#[cfg(test)]
mod tests {
    use crate::{physics::IPhysicsContainer, physx::container::PhysxPhysicsContainer};

    #[test]
    fn test_create_and_step() {
        let container = PhysxPhysicsContainer::default();
        container.step(1.0);
    }
}
