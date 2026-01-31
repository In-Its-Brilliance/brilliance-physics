use common::{
    chunks::position::Vector3,
    timed_lock,
    utils::debug::{info::DebugInfo, SmartRwLock},
};
use nalgebra::Point3;
use parking_lot::{MappedRwLockReadGuard, MappedRwLockWriteGuard, RwLockReadGuard, RwLockWriteGuard};
use rapier3d::{parry::query::ShapeCastOptions, prelude::*};
use std::{sync::Arc, time::Instant};

use crate::physics::{IPhysicsContainer, RayCastResultNormal, ShapeCastResult};

use super::{
    bridge::IntoNaVector3,
    collider::{RapierPhysicsCollider, RapierPhysicsShape},
    collider_builder::RapierPhysicsColliderBuilder,
    controller::RapierPhysicsController,
    query_filter::RapierQueryFilter,
};

#[derive(Clone)]
pub struct RapierPhysicsContainer {
    pub(crate) controller: Arc<SmartRwLock<RapierPhysicsController>>,
    pub(crate) rigid_body_set: Arc<SmartRwLock<RigidBodySet>>,
    pub(crate) collider_set: Arc<SmartRwLock<ColliderSet>>,
    pub(crate) query_pipeline: Arc<SmartRwLock<QueryPipeline>>,
    pub(crate) island_manager: Arc<SmartRwLock<IslandManager>>,

    pub(crate) last_step_duration: Arc<SmartRwLock<std::time::Duration>>,
    colliders_changed: Arc<SmartRwLock<bool>>,
}

#[rustfmt::skip]
impl Default for RapierPhysicsContainer {
    fn default() -> Self {
        let rapier_physics_container = Self {
            controller: Arc::new(timed_lock!(RapierPhysicsController::create(), "RapierPhysicsContainer::controller")),
            rigid_body_set: Arc::new(timed_lock!(RigidBodySet::new(), "RapierPhysicsContainer::rigid_body_set")),
            collider_set: Arc::new(timed_lock!(ColliderSet::new(), "RapierPhysicsContainer::collider_set")),
            query_pipeline: Arc::new(timed_lock!(QueryPipeline::new(), "RapierPhysicsContainer::query_pipeline")),
            island_manager: Arc::new(timed_lock!(IslandManager::new(), "RapierPhysicsContainer::island_manager")),

            last_step_duration: Arc::new(timed_lock!(Default::default(), "RapierPhysicsContainer::last_step_duration")),
            colliders_changed: Arc::new(timed_lock!(false, "RapierPhysicsContainer::colliders_changed")),
       };
        rapier_physics_container
    }
}

impl RapierPhysicsContainer {
    pub fn get_collider(&self, collider_handle: &ColliderHandle) -> Option<MappedRwLockReadGuard<'_, Collider>> {
        RwLockReadGuard::try_map(self.collider_set.read(), |p| match p.get(*collider_handle) {
            Some(c) => Some(c),
            None => None,
        })
        .ok()
    }

    pub fn get_collider_mut(&self, collider_handle: &ColliderHandle) -> Option<MappedRwLockWriteGuard<'_, Collider>> {
        RwLockWriteGuard::try_map(self.collider_set.write(), |p| match p.get_mut(*collider_handle) {
            Some(c) => Some(c),
            None => None,
        })
        .ok()
    }

    pub(crate) fn remove_collider(&self, collider_handle: ColliderHandle) {
        *self.colliders_changed.write() = true;
        self.collider_set.write().remove(
            collider_handle,
            &mut self.island_manager.write(),
            &mut self.rigid_body_set.write(),
            true,
        );
    }

    pub fn get_rigid_body(&self, rigid_handle: &RigidBodyHandle) -> Option<MappedRwLockReadGuard<'_, RigidBody>> {
        RwLockReadGuard::try_map(self.rigid_body_set.read(), |p| match p.get(*rigid_handle) {
            Some(c) => Some(c),
            None => None,
        })
        .ok()
    }

    pub fn get_rigid_body_mut(
        &mut self,
        rigid_handle: &RigidBodyHandle,
    ) -> Option<MappedRwLockWriteGuard<'_, RigidBody>> {
        RwLockWriteGuard::try_map(self.rigid_body_set.write(), |p| match p.get_mut(*rigid_handle) {
            Some(c) => Some(c),
            None => None,
        })
        .ok()
    }

    fn rebuild_query_pipeline(&self) {
        let mut pipeline = self.query_pipeline.write();
        *pipeline = QueryPipeline::new();
        pipeline.update(&self.collider_set.read());
    }
}

impl IPhysicsContainer<RapierPhysicsShape, RapierPhysicsCollider, RapierPhysicsColliderBuilder, RapierQueryFilter>
    for RapierPhysicsContainer
{
    /// Выполняет шаг физической симуляции.
    ///
    /// Периодически перестраивает `QueryPipeline` для обхода бага с деградацией
    /// производительности при инкрементальных обновлениях:
    /// - https://github.com/dimforge/rapier/issues/617
    fn step(&self, delta: f32) {
        let start = Instant::now();
        self.controller.write().step(delta, self);

        if *self.colliders_changed.read() {
            self.rebuild_query_pipeline();
            *self.colliders_changed.write() = false;
        }

        *self.last_step_duration.write() = start.elapsed();
    }

    fn spawn_collider(&self, mut collider_builder: RapierPhysicsColliderBuilder) -> RapierPhysicsCollider {
        *self.colliders_changed.write() = true;
        let collider = std::mem::take(&mut collider_builder.builder);
        let collider_handle = self.collider_set.write().insert(collider);
        RapierPhysicsCollider::create(&self, collider_handle)
    }

    // https://docs.godotengine.org/en/stable/classes/class_node3d.html#class-node3d-property-rotation
    fn cast_ray(
        &self,
        origin: Vector3,
        dir: Vector3,
        max_toi: f32,
        filter: RapierQueryFilter,
    ) -> Option<RayCastResultNormal> {
        let origin = Point3::new(origin.x, origin.y, origin.z);
        let ray = Ray::new(origin, dir.to_na());

        let mut rapier_filter = filter.get_filter();
        let adapter = |handle: ColliderHandle, _: &Collider| (filter.predicate)(handle.into_raw_parts().0 as usize);
        if filter.is_predicate {
            rapier_filter = rapier_filter.predicate(&adapter);
        }

        let pipeline = self.query_pipeline.read();
        if let Some((handle, ray_intersection)) = pipeline.cast_ray_and_get_normal(
            &self.rigid_body_set.read(),
            &self.collider_set.read(),
            &ray,
            max_toi,
            true,
            rapier_filter,
        ) {
            let point = ray.point_at(ray_intersection.time_of_impact);
            let result = RayCastResultNormal {
                collider_id: handle.into_raw_parts().0 as usize,
                point: Vector3::new(point.x, point.y, point.z),
                normal: Vector3::new(
                    ray_intersection.normal.x,
                    ray_intersection.normal.y,
                    ray_intersection.normal.z,
                ),
            };
            return Some(result);
        }
        return None;
    }

    /// # Parameters
    /// * `shape` - the shape being cast
    /// * `origin` - the initial position of the shape (this is analog to ray.origin)
    /// * `dir` - the linear velocity the shape is travelling at (this is analog to ray.dir)
    /// * `max_toi` - The maximum time-of-impact that can be reported by this cast. This effectively
    fn cast_shape(
        &self,
        shape: RapierPhysicsShape,
        origin: Vector3,
        dir: Vector3,
        max_toi: f32,
        filter: RapierQueryFilter,
    ) -> Option<ShapeCastResult> {
        let shape_pos = Isometry::new(origin.to_na(), vector![0.0, 0.0, 0.0]);

        let mut rapier_filter = filter.get_filter();
        let adapter = |handle: ColliderHandle, _: &Collider| (filter.predicate)(handle.into_raw_parts().0 as usize);
        if filter.is_predicate {
            rapier_filter = rapier_filter.predicate(&adapter);
        }

        let options = ShapeCastOptions::with_max_time_of_impact(max_toi);
        let pipeline = self.query_pipeline.read();
        if let Some((handle, shape_hit)) = pipeline.cast_shape(
            &self.rigid_body_set.read(),
            &self.collider_set.read(),
            &shape_pos,
            &dir.to_na(),
            shape.get_shape(),
            options,
            rapier_filter,
        ) {
            let point = origin.to_na() + dir.to_na() * shape_hit.time_of_impact;
            let result = ShapeCastResult {
                collider_id: handle.into_raw_parts().0 as usize,
                point: Vector3::new(point.x, point.y, point.z),
            };
            return Some(result);
        }
        return None;
    }

    fn get_debug_info(&self) -> DebugInfo {
        let colliders = self.collider_set.read();
        let bodies = self.rigid_body_set.read();
        let islands = self.island_manager.read();
        let step_dur = *self.last_step_duration.read();

        DebugInfo::new()
            .insert("last_step_dur", step_dur)
            .insert("colliders", colliders.len())
            .insert("rigid_bodies", bodies.len())
            .insert("active_dynamic", islands.active_dynamic_bodies().len())
            .insert("active_kinematic", islands.active_kinematic_bodies().len())
    }
}
