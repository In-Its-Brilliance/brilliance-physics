use physx::foundation::DefaultAllocator;
use physx::math::PxVec3;
use physx::owner::Owner;
use physx::prelude::{Physics, PhysicsFoundation, SceneDescriptor};
use physx::scene::Scene;

use super::container::PhysxPhysicsContainer;
use super::types::{OnAdvance, OnCollision, PxScene, PxShape};

pub struct PhysxPhysicsController {
    // IMPORTANT: scene must be declared before physics so it is dropped first.
    // PhysX requires PxScene to be released before PxFoundation.
    pub(crate) scene: Owner<PxScene>,
    pub(crate) physics: PhysicsFoundation<DefaultAllocator, PxShape>,
}

impl PhysxPhysicsController {
    pub(crate) fn create() -> Self {
        let builder = physx::physics::PhysicsFoundationBuilder::default();
        let mut physics = builder.build().expect("building PhysX foundation failed");

        let scene: Owner<PxScene> = physics
            .create(SceneDescriptor {
                gravity: PxVec3::new(0.0, -9.81, 0.0),
                on_advance: Some(OnAdvance),
                on_collide: Some(OnCollision),
                ..SceneDescriptor::new(std::ptr::null())
            })
            .expect("scene creation error");
        Self { physics, scene }
    }

    pub(crate) fn step(&mut self, delta: f32, _physics_container: &PhysxPhysicsContainer) {
        self.scene
            .step(delta, None::<&mut physx_sys::PxBaseTask>, None, true)
            .expect("error occured during simulation");
    }
}
