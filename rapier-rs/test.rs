use rapier3d::prelude::*;
fn main() {
    let mut world = RigidBodySet::new();
    let rb = RigidBodyBuilder::new(RigidBodyType::Dynamic).build();
    let handle = world.insert(rb);
    let (idx, gen) = handle.into_raw_parts();
    println!("idx={}, gen={}", idx, gen);
    let handle2 = RigidBodyHandle::from_raw_parts(idx, gen);
    assert_eq!(handle, handle2);
}
