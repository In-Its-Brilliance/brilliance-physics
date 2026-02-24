use crate::physics::IQueryFilter;
use super::collider::{PhysxPhysicsCollider, PhysxPhysicsShape};
use physx::traits::Class;
use physx_sys::{
    create_raycast_filter_callback, PxQueryFilterCallback, PxQueryFilterCallback_delete,
    PxQueryFilterData, PxQueryFilterData_new, PxQueryFlags, PxShape_getQueryFilterData,
};
use std::ptr::drop_in_place;

pub struct PhysxQueryFilter {
    pub(crate) filter: PxQueryFilterData,
    pub(crate) pre_filter_callback: Option<*mut PxQueryFilterCallback>,
    pub(crate) exclude_sensors_flag: bool,
    pub(crate) collision_mask_filter: Option<(u32, u32)>,
    pub(crate) is_predicate: bool,
    pub(crate) predicate: Box<dyn Fn(usize) -> bool>,
}

impl PhysxQueryFilter {
    /// Post-filter check for raycast/sweep hits against filter criteria.
    /// Returns true if the hit should be accepted.
    pub(crate) fn post_filter_hit(
        &self,
        shape_ptr: *const physx_sys::PxShape,
        collider_id: usize,
    ) -> bool {
        // Check sensor exclusion (word2 == 1 means sensor)
        if self.exclude_sensors_flag {
            let filter_data = unsafe { PxShape_getQueryFilterData(shape_ptr) };
            if filter_data.word2 != 0 {
                return false;
            }
        }

        // Check collision mask (bidirectional group/mask)
        if let Some((groups, mask)) = self.collision_mask_filter {
            let filter_data = unsafe { PxShape_getQueryFilterData(shape_ptr) };
            let shape_groups = filter_data.word0;
            let shape_mask = filter_data.word1;
            if (groups & shape_mask) == 0 || (shape_groups & mask) == 0 {
                return false;
            }
        }

        // Check predicate
        if self.is_predicate && !(self.predicate)(collider_id) {
            return false;
        }

        true
    }
}

impl Default for PhysxQueryFilter {
    fn default() -> Self {
        let mut filter = unsafe { PxQueryFilterData_new() };
        // Disable the built-in hardcoded filter. PhysX's hardcoded filter does
        // bitwise AND between query PxFilterData and shape PxFilterData — if the
        // result is zero, the shape is silently blocked. Since our query filter
        // data is all zeros but shapes have non-zero collision mask words,
        // the hardcoded filter would block every shape with a mask set.
        // We handle all filtering ourselves via pre-filter callback + post-filter.
        filter.flags |= PxQueryFlags::DisableHardcodedFilter;
        Self {
            filter,
            pre_filter_callback: None,
            exclude_sensors_flag: false,
            collision_mask_filter: None,
            is_predicate: false,
            predicate: Box::new(|_| true),
        }
    }
}

impl IQueryFilter<PhysxPhysicsShape, PhysxPhysicsCollider> for PhysxQueryFilter {
    fn exclude_collider(&mut self, collider: &PhysxPhysicsCollider) {
        let actor = collider.actor.read();
        self.pre_filter_callback =
            Some(unsafe { create_raycast_filter_callback(actor.as_ptr()) });
        // PhysX ignores the pre-filter callback unless ePREFILTER is set in query flags
        self.filter.flags |= PxQueryFlags::Prefilter;
    }

    fn exclude_sensors(&mut self) {
        self.exclude_sensors_flag = true;
    }

    fn collision_mask(&mut self, groups: u32, mask: u32) {
        self.collision_mask_filter = Some((groups, mask));
    }

    fn predicate(&mut self, predicate: Box<dyn Fn(usize) -> bool>) {
        self.predicate = predicate;
        self.is_predicate = true;
    }
}

impl Drop for PhysxQueryFilter {
    fn drop(&mut self) {
        if let Some(ptr) = self.pre_filter_callback.take() {
            unsafe { PxQueryFilterCallback_delete(ptr) };
            unsafe {
                drop_in_place(ptr);
            }
        }
    }
}
