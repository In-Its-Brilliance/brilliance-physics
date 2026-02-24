use common::chunks::position::Vector3;
use physx::prelude::*;

pub(crate) fn physx_to_network(other: &PxVec3) -> Vector3 {
    Vector3::new(other.x(), other.y(), other.z())
}

pub(crate) fn physx_sys_to_network(other: &physx_sys::PxVec3) -> Vector3 {
    Vector3::new(other.x, other.y, other.z)
}

pub(crate) fn network_to_physx(other: &Vector3) -> PxVec3 {
    PxVec3::new(other.x, other.y, other.z)
}

pub(crate) fn network_to_physx_sys(other: &Vector3) -> physx_sys::PxVec3 {
    physx_sys::PxVec3 {
        x: other.x,
        y: other.y,
        z: other.z,
    }
}

/// PhysX capsules are aligned along the X-axis by default.
/// This quaternion rotates 90° around Z to align the capsule along Y (vertical).
pub(crate) const CAPSULE_ROTATION: physx_sys::PxQuat = physx_sys::PxQuat {
    x: 0.0,
    y: 0.0,
    z: 0.70710678,
    w: 0.70710678,
};
