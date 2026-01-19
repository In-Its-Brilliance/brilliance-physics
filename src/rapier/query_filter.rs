use rapier3d::prelude::{ColliderHandle, QueryFilter};

use crate::physics::IQueryFilter;

use super::collider::{RapierPhysicsCollider, RapierPhysicsShape};

pub struct RapierQueryFilter {
    exclude_sensors: bool,
    excluded_colliders: Vec<ColliderHandle>,
    pub(crate) predicate: Box<dyn Fn(usize) -> bool>,
}

impl Default for RapierQueryFilter {
    fn default() -> Self {
        Self {
            exclude_sensors: false,
            excluded_colliders: Default::default(),
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

    fn predicate(&mut self, predicate: Box<dyn Fn(usize) -> bool>) {
        self.predicate = predicate;
    }
}
