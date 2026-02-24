use crate::physics::IPhysicsCharacterController;
use common::chunks::position::Vector3;
use physx::prelude::RigidActor;
use physx::shape::Shape;
use physx::traits::Class;
use physx_sys::{
    PxGeometry_getType, PxGeometryType, PxHitFlags, PxSceneQueryExt_sweepSingle,
    PxShape_getGeometry,
};
use std::mem::MaybeUninit;
use std::ptr::{null, null_mut};

fn vec3_length(v: &Vector3) -> f32 {
    (v.x * v.x + v.y * v.y + v.z * v.z).sqrt()
}

use super::bridge::{network_to_physx_sys, physx_sys_to_network, CAPSULE_ROTATION};
use super::collider::{PhysxPhysicsCollider, PhysxPhysicsShape};
use super::query_filter::PhysxQueryFilter;
use super::types::PxShape as PxShapeType;

const SKIN_WIDTH: f32 = 0.01;
const MAX_SLIDE_ITERATIONS: usize = 4;

pub struct PhysxPhysicsCharacterController {
    custom_mass: Option<f32>,
    snap_to_ground: Option<f32>,
}

impl IPhysicsCharacterController<PhysxPhysicsShape, PhysxPhysicsCollider, PhysxQueryFilter>
    for PhysxPhysicsCharacterController
{
    fn create(custom_mass: Option<f32>, snap_to_ground: Option<f32>) -> Self {
        Self {
            custom_mass,
            snap_to_ground,
        }
    }

    fn move_shape(
        &mut self,
        collider: &PhysxPhysicsCollider,
        filter: PhysxQueryFilter,
        _delta: f64,
        movement: Vector3,
    ) -> Vector3 {
        let controller = collider.controller.read();
        let actor_guard = collider.actor.read();

        // Get the geometry from the collider's first shape
        let shapes = actor_guard.get_shapes();
        if shapes.is_empty() {
            return movement;
        }
        let geom_ptr = unsafe { PxShape_getGeometry(shapes[0].as_ptr()) };

        // Determine capsule rotation for sweep pose (PhysX capsules are X-aligned)
        let geom_type = unsafe { PxGeometry_getType(geom_ptr) };
        let rotation = if geom_type == PxGeometryType::Capsule {
            CAPSULE_ROTATION
        } else {
            physx_sys::PxQuat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        };

        // Get current position from the actor
        let global_pos = actor_guard.get_global_position();
        let mut current_pos = physx_sys::PxVec3 {
            x: global_pos.x(),
            y: global_pos.y(),
            z: global_pos.z(),
        };

        let callback_ptr = filter.pre_filter_callback.unwrap_or(null_mut());

        let mut remaining = movement;
        let mut result = Vector3::zero();

        // Iterative sweep-and-slide
        for _ in 0..MAX_SLIDE_ITERATIONS {
            let length = vec3_length(&remaining);
            if length < 1e-6 {
                break;
            }

            let dir = Vector3::new(
                remaining.x / length,
                remaining.y / length,
                remaining.z / length,
            );

            let pose = physx_sys::PxTransform {
                q: rotation,
                p: current_pos,
            };

            let dir_sys = network_to_physx_sys(&dir);
            let mut sweep_hit = MaybeUninit::uninit();

            let hit = unsafe {
                PxSceneQueryExt_sweepSingle(
                    controller.scene.as_ptr(),
                    geom_ptr,
                    &pose,
                    &dir_sys,
                    length + SKIN_WIDTH,
                    PxHitFlags::Default | PxHitFlags::Position | PxHitFlags::Normal,
                    sweep_hit.as_mut_ptr(),
                    &filter.filter as *const _,
                    callback_ptr,
                    null(),
                    0.0,
                )
            };

            if hit {
                let sweep_hit = unsafe { sweep_hit.assume_init() };

                // Post-filter: check sensors, collision mask, predicate
                let collider_id = unsafe {
                    let shape = &*(sweep_hit.shape as *const PxShapeType);
                    *shape.get_user_data()
                };
                if !filter.post_filter_hit(sweep_hit.shape as *const _, collider_id) {
                    // Hit was filtered out — advance past it and continue sweeping
                    // so we can find solid geometry behind the filtered shape.
                    let advance_dist = (sweep_hit.distance + SKIN_WIDTH).min(length);
                    let step = Vector3::new(
                        dir.x * advance_dist,
                        dir.y * advance_dist,
                        dir.z * advance_dist,
                    );
                    result = result + step;
                    current_pos.x += step.x;
                    current_pos.y += step.y;
                    current_pos.z += step.z;
                    let leftover = length - advance_dist;
                    if leftover < 1e-6 {
                        break;
                    }
                    remaining = Vector3::new(
                        dir.x * leftover,
                        dir.y * leftover,
                        dir.z * leftover,
                    );
                    continue;
                }

                let safe_dist = (sweep_hit.distance - SKIN_WIDTH).max(0.0);

                let step = Vector3::new(
                    dir.x * safe_dist,
                    dir.y * safe_dist,
                    dir.z * safe_dist,
                );
                result = result + step;
                current_pos.x += step.x;
                current_pos.y += step.y;
                current_pos.z += step.z;

                // Slide: project remaining movement onto the collision plane
                let normal = physx_sys_to_network(&sweep_hit.normal);
                let penetrated_dist = length - safe_dist;

                // Remove the normal component from remaining movement
                let dot =
                    remaining.x * normal.x + remaining.y * normal.y + remaining.z * normal.z;
                remaining = Vector3::new(
                    remaining.x - normal.x * dot,
                    remaining.y - normal.y * dot,
                    remaining.z - normal.z * dot,
                );

                // Scale to remaining travel distance
                let remaining_len = vec3_length(&remaining);
                if remaining_len > 1e-6 {
                    let scale = (penetrated_dist / remaining_len).min(1.0);
                    remaining = Vector3::new(
                        remaining.x * scale,
                        remaining.y * scale,
                        remaining.z * scale,
                    );
                } else {
                    break;
                }
            } else {
                // No obstacle — apply full remaining movement
                result = result + remaining;
                current_pos.x += remaining.x;
                current_pos.y += remaining.y;
                current_pos.z += remaining.z;
                break;
            }
        }

        // Snap-to-ground: sweep downward to find ground surface
        if let Some(snap_distance) = self.snap_to_ground {
            let down_dir = physx_sys::PxVec3 {
                x: 0.0,
                y: -1.0,
                z: 0.0,
            };
            let pose = physx_sys::PxTransform {
                q: rotation,
                p: current_pos,
            };

            let mut sweep_hit = MaybeUninit::uninit();
            let hit = unsafe {
                PxSceneQueryExt_sweepSingle(
                    controller.scene.as_ptr(),
                    geom_ptr,
                    &pose,
                    &down_dir,
                    snap_distance,
                    PxHitFlags::Default | PxHitFlags::Position | PxHitFlags::Normal,
                    sweep_hit.as_mut_ptr(),
                    &filter.filter as *const _,
                    callback_ptr,
                    null(),
                    0.0,
                )
            };

            if hit {
                let sweep_hit = unsafe { sweep_hit.assume_init() };
                // Post-filter: only snap to ground that passes the filter
                let collider_id = unsafe {
                    let shape = &*(sweep_hit.shape as *const PxShapeType);
                    *shape.get_user_data()
                };
                if filter.post_filter_hit(sweep_hit.shape as *const _, collider_id) {
                    let snap_dist = (sweep_hit.distance - SKIN_WIDTH).max(0.0);
                    result.y -= snap_dist;
                }
            }
        }

        result
    }

    fn get_custom_mass(&mut self) -> &Option<f32> {
        &self.custom_mass
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        physics::{
            IPhysicsCharacterController, IPhysicsCollider, IPhysicsColliderBuilder,
            IPhysicsContainer, IQueryFilter,
        },
        physx::{
            character_controller::PhysxPhysicsCharacterController,
            collider_builder::PhysxPhysicsColliderBuilder,
            container::PhysxPhysicsContainer,
            query_filter::PhysxQueryFilter,
        },
    };
    use common::chunks::position::Vector3;

    use crate::PhysicsCollider;

    /// Flat trimesh ground plane at Y=0, spanning -50..+50 on X and Z.
    /// Returns the collider so it stays alive (dropping removes the actor from the scene).
    fn create_ground_plane(container: &PhysxPhysicsContainer) -> PhysicsCollider {
        let verts = vec![
            Vector3::new(-50.0, 0.0, -50.0),
            Vector3::new(50.0, 0.0, -50.0),
            Vector3::new(50.0, 0.0, 50.0),
            Vector3::new(-50.0, 0.0, 50.0),
        ];
        // Winding with normals facing DOWN — DoubleSided flag should handle this
        let indices: Vec<[u32; 3]> = vec![[0, 1, 2], [0, 2, 3]];
        let ground_builder = PhysxPhysicsColliderBuilder::trimesh(verts, indices);
        container.spawn_collider(ground_builder)
    }

    #[test]
    fn test_move_shape_capsule_blocked_by_trimesh() {
        let container = PhysxPhysicsContainer::default();
        container.step(0.01);

        // Ground plane at Y=0 (must keep alive!)
        let _ground = create_ground_plane(&container);
        container.step(0.01);

        // Verify the ground is detectable by raycast
        {
            use crate::physics::IPhysicsContainer;
            let debug_info = container.get_debug_info();
            println!("debug_info: {:?}", debug_info);

            let filter_ray = PhysxQueryFilter::default();
            let ray_hit = container.cast_ray(
                Vector3::new(0.0, 10.0, 0.0),
                Vector3::new(0.0, -1.0, 0.0),
                20.0,
                filter_ray,
            );
            println!("cast_ray from (0,10,0) down: {:?}", ray_hit.as_ref().map(|h| (h.point, h.normal)));
            assert!(ray_hit.is_some(), "Raycast could not find the ground plane!");
        }

        // Capsule (player) at Y=10 (center of capsule)
        // cylinder(0.9, 0.4) → PhysX capsule with radius=0.4, halfHeight=0.5
        // Total half-extent = 0.9, so bottom of capsule at Y = 10.0 - 0.9 = 9.1
        let capsule_builder = PhysxPhysicsColliderBuilder::cylinder(0.9, 0.4);
        let mut capsule = container.spawn_collider(capsule_builder);
        capsule.set_position(Vector3::new(0.0, 10.0, 0.0));

        let mut char_ctrl =
            PhysxPhysicsCharacterController::create(None, Some(0.1));

        // Try to move down by 20 units — should be stopped by the ground
        let mut filter = PhysxQueryFilter::default();
        filter.exclude_collider(&capsule);

        let translation = char_ctrl.move_shape(
            &capsule,
            filter,
            1.0 / 60.0,
            Vector3::new(0.0, -20.0, 0.0),
        );

        // Capsule center at Y=10, half-extent=0.9, ground at Y=0
        // Capsule should stop with center at ~Y=0.9 (bottom touching ground)
        // So translation.y should be about -9.1, NOT -20.0
        println!(
            "translation = ({}, {}, {})",
            translation.x, translation.y, translation.z
        );

        assert!(
            translation.y > -20.0,
            "Capsule passed through the ground! translation.y = {} (expected ~-9.1)",
            translation.y
        );
        assert!(
            translation.y > -10.0,
            "Capsule fell too far! translation.y = {} (expected ~-9.1)",
            translation.y
        );
    }
}
