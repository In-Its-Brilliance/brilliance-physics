use super::collider::{RapierPhysicsCollider, RapierPhysicsShape};
use crate::physics::IQueryFilter;
use rapier3d::prelude::{InteractionGroups, Group};
use rapier3d::prelude::{ColliderHandle, QueryFilter};

pub struct RapierQueryFilter {
    exclude_sensors: bool,
    excluded_colliders: Vec<ColliderHandle>,
    interaction_groups: Option<InteractionGroups>,
    pub(crate) is_predicate: bool,
    pub(crate) predicate: Box<dyn Fn(usize) -> bool>,
}

impl Default for RapierQueryFilter {
    fn default() -> Self {
        Self {
            exclude_sensors: false,
            excluded_colliders: Default::default(),
            interaction_groups: None,
            is_predicate: false,
            predicate: Box::new(|_| true),
        }
    }
}

impl RapierQueryFilter {
    pub(crate) fn get_filter<'a>(&'a self) -> QueryFilter<'a> {
        let mut filter = QueryFilter::default();

        if self.exclude_sensors {
            filter = filter.exclude_sensors();
        }

        if let Some(groups) = self.interaction_groups {
            filter = filter.groups(groups);
        }

        for h in &self.excluded_colliders {
            filter = filter.exclude_collider(*h);
        }
        filter
    }
}

impl IQueryFilter<RapierPhysicsShape, RapierPhysicsCollider> for RapierQueryFilter {
    fn exclude_collider(&mut self, collider: &RapierPhysicsCollider) {
        self.excluded_colliders.push(collider.collider_handle);
    }

    fn exclude_sensors(&mut self) {
        self.exclude_sensors = true;
    }

    fn collision_mask(&mut self, groups: u32, mask: u32) {
        self.interaction_groups = Some(
            InteractionGroups::new(
                Group::from_bits_truncate(groups),
                Group::from_bits_truncate(mask),
            )
        );
    }

    fn predicate(&mut self, predicate: Box<dyn Fn(usize) -> bool>) {
        self.predicate = predicate;
        self.is_predicate = true;
    }
}
