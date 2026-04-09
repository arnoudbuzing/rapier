use nalgebra::UnitQuaternion;
use rapier3d::prelude::*;
use std::collections::HashMap;
use wolfram_library_link as wll;
use wolfram_library_link::NumericArray;

struct PhysicsWorld {
    gravity: Vector<Real>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
}

impl PhysicsWorld {
    fn new(gx: Real, gy: Real, gz: Real) -> Self {
        Self {
            gravity: vector![gx, gy, gz],
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
        }
    }
    
    fn step(&mut self, time_step: Real) {
        self.integration_parameters.dt = time_step;
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &(),
            &(),
        );
    }
}

use std::cell::{Cell, RefCell};
thread_local! {
    static WORLDS: RefCell<HashMap<usize, PhysicsWorld>> = RefCell::new(HashMap::new());
    static NEXT_WORLD_ID: Cell<usize> = Cell::new(1);
}

#[wll::export]
fn rapier_version() -> String {
    "0.23.0".to_string()
}

#[wll::export]
fn rapier_cuboid_mass(hx: f64, hy: f64, hz: f64, density: f64) -> f64 {
    let collider = ColliderBuilder::cuboid(hx as f32, hy as f32, hz as f32)
        .density(density as f32)
        .build();
    collider.mass() as f64
}

#[wll::export]
fn rapier_world_create(gx: f64, gy: f64, gz: f64) -> i64 {
    let id = NEXT_WORLD_ID.with(|next| {
        let current = next.get();
        next.set(current + 1);
        current
    });
    let world = PhysicsWorld::new(gx as Real, gy as Real, gz as Real);
    WORLDS.with(|worlds| {
        worlds.borrow_mut().insert(id, world);
    });
    id as i64
}

#[wll::export]
fn rapier_world_destroy(world_id: i64) -> bool {
    WORLDS.with(|worlds| {
        worlds.borrow_mut().remove(&(world_id as usize)).is_some()
    })
}

#[wll::export]
fn rapier_add_rigid_body(
    world_id: i64,
    x: f64, y: f64, z: f64,
    qx: f64, qy: f64, qz: f64, qw: f64,
    body_type: i64,
) -> i64 {
    WORLDS.with(|w| {
        let mut worlds = w.borrow_mut();
        if let Some(world) = worlds.get_mut(&(world_id as usize)) {
            let rb_ty = match body_type {
                0 => RigidBodyType::Dynamic,
                1 => RigidBodyType::Fixed,
                2 => RigidBodyType::KinematicPositionBased,
                3 => RigidBodyType::KinematicVelocityBased,
                _ => RigidBodyType::Fixed,
            };
            
            let mut builder = RigidBodyBuilder::new(rb_ty)
                .translation(vector![x as f32, y as f32, z as f32]);
                
            let q = nalgebra::Quaternion::new(qw as f32, qx as f32, qy as f32, qz as f32);
            let r_rot: UnitQuaternion<f32> = UnitQuaternion::from_quaternion(q);
            
            builder = builder.rotation(r_rot.scaled_axis()); 
            
            let mut rb = builder.build();
            rb.set_rotation(r_rot, true);
            
            let handle = world.rigid_body_set.insert(rb);
            
            let (index, gen) = handle.into_raw_parts();
            return ((index as i64) << 32) | (gen as i64);
        }
        -1
    })
}

#[wll::export]
fn rapier_add_collider_cuboid(
    world_id: i64,
    body_handle_raw: i64,
    hx: f64, hy: f64, hz: f64,
    density: f64
) -> i64 {
    WORLDS.with(|w| {
        let mut worlds = w.borrow_mut();
        if let Some(world) = worlds.get_mut(&(world_id as usize)) {
            let collider = ColliderBuilder::cuboid(hx as f32, hy as f32, hz as f32)
                .density(density as f32)
                .restitution(0.8)
                .build();
            
            let handle = if body_handle_raw == -1 {
                world.collider_set.insert(collider)
            } else {
                let index = (body_handle_raw >> 32) as u32;
                let gen = (body_handle_raw & 0xFFFFFFFF) as u32;
                let body_handle = RigidBodyHandle::from_raw_parts(index, gen);
                world.collider_set.insert_with_parent(
                    collider, body_handle, &mut world.rigid_body_set
                )
            };
            let (index, gen) = handle.into_raw_parts();
            return ((index as i64) << 32) | (gen as i64);
        }
        -1
    })
}

#[wll::export]
fn rapier_add_collider_sphere(
    world_id: i64,
    body_handle_raw: i64,
    radius: f64,
    density: f64
) -> i64 {
    WORLDS.with(|w| {
        let mut worlds = w.borrow_mut();
        if let Some(world) = worlds.get_mut(&(world_id as usize)) {
            let collider = ColliderBuilder::ball(radius as f32)
                .density(density as f32)
                .restitution(0.8)
                .build();
                
            let handle = if body_handle_raw == -1 {
                world.collider_set.insert(collider)
            } else {
                let index = (body_handle_raw >> 32) as u32;
                let gen = (body_handle_raw & 0xFFFFFFFF) as u32;
                let body_handle = RigidBodyHandle::from_raw_parts(index, gen);
                world.collider_set.insert_with_parent(
                    collider, body_handle, &mut world.rigid_body_set
                )
            };
            let (index, gen) = handle.into_raw_parts();
            return ((index as i64) << 32) | (gen as i64);
        }
        -1
    })
}

#[wll::export]
fn rapier_add_collider_cylinder(
    world_id: i64,
    body_handle_raw: i64,
    half_height: f64,
    radius: f64,
    density: f64
) -> i64 {
    WORLDS.with(|w| {
        let mut worlds = w.borrow_mut();
        if let Some(world) = worlds.get_mut(&(world_id as usize)) {
            let collider = ColliderBuilder::cylinder(half_height as f32, radius as f32)
                .density(density as f32)
                .restitution(0.8)
                .build();

            let handle = if body_handle_raw == -1 {
                world.collider_set.insert(collider)
            } else {
                let index = (body_handle_raw >> 32) as u32;
                let gen = (body_handle_raw & 0xFFFFFFFF) as u32;
                let body_handle = RigidBodyHandle::from_raw_parts(index, gen);
                world.collider_set.insert_with_parent(
                    collider, body_handle, &mut world.rigid_body_set
                )
            };
            let (index, gen) = handle.into_raw_parts();
            return ((index as i64) << 32) | (gen as i64);
        }
        -1
    })
}

#[wll::export]
fn rapier_add_collider_cone(
    world_id: i64,
    body_handle_raw: i64,
    half_height: f64,
    radius: f64,
    density: f64
) -> i64 {
    WORLDS.with(|w| {
        let mut worlds = w.borrow_mut();
        if let Some(world) = worlds.get_mut(&(world_id as usize)) {
            let collider = ColliderBuilder::cone(half_height as f32, radius as f32)
                .density(density as f32)
                .restitution(0.8)
                .build();

            let handle = if body_handle_raw == -1 {
                world.collider_set.insert(collider)
            } else {
                let index = (body_handle_raw >> 32) as u32;
                let gen = (body_handle_raw & 0xFFFFFFFF) as u32;
                let body_handle = RigidBodyHandle::from_raw_parts(index, gen);
                world.collider_set.insert_with_parent(
                    collider, body_handle, &mut world.rigid_body_set
                )
            };
            let (index, gen) = handle.into_raw_parts();
            return ((index as i64) << 32) | (gen as i64);
        }
        -1
    })
}

#[wll::export]
fn rapier_add_collider_capsule(
    world_id: i64,
    body_handle_raw: i64,
    half_height: f64,
    radius: f64,
    density: f64
) -> i64 {
    WORLDS.with(|w| {
        let mut worlds = w.borrow_mut();
        if let Some(world) = worlds.get_mut(&(world_id as usize)) {
            let collider = ColliderBuilder::capsule_y(half_height as f32, radius as f32)
                .density(density as f32)
                .restitution(0.8)
                .build();

            let handle = if body_handle_raw == -1 {
                world.collider_set.insert(collider)
            } else {
                let index = (body_handle_raw >> 32) as u32;
                let gen = (body_handle_raw & 0xFFFFFFFF) as u32;
                let body_handle = RigidBodyHandle::from_raw_parts(index, gen);
                world.collider_set.insert_with_parent(
                    collider, body_handle, &mut world.rigid_body_set
                )
            };
            let (index, gen) = handle.into_raw_parts();
            return ((index as i64) << 32) | (gen as i64);
        }
        -1
    })
}

#[wll::export]
fn rapier_world_step(world_id: i64, steps: i64, time_step: f64) {
    WORLDS.with(|w| {
        let mut worlds = w.borrow_mut();
        if let Some(world) = worlds.get_mut(&(world_id as usize)) {
            for _ in 0..steps {
                world.step(time_step as Real);
            }
        }
    });
}

// Returns a flat f64 tensor of shape {N*7} => [X, Y, Z, QX, QY, QZ, QW]
#[wll::export]
fn rapier_get_body_positions(world_id: i64) -> NumericArray<f64> {
    WORLDS.with(|w| {
        let worlds = w.borrow();
        if let Some(world) = worlds.get(&(world_id as usize)) {
            let count = world.rigid_body_set.len();
            let mut flat = Vec::with_capacity(count * 7);
            for (_handle, rb) in world.rigid_body_set.iter() {
                let trans = rb.translation();
                let rot = rb.rotation();
                flat.push(trans.x as f64);
                flat.push(trans.y as f64);
                flat.push(trans.z as f64);
                flat.push(rot.i as f64);
                flat.push(rot.j as f64);
                flat.push(rot.k as f64);
                flat.push(rot.w as f64);
            }
            return NumericArray::from_slice(&flat);
        }
        NumericArray::from_slice(&[])
    })
}

// Returns a flat i64 tensor of shape {N} => [BodyHandleRaw]
#[wll::export]
fn rapier_get_body_handles(world_id: i64) -> NumericArray<i64> {
    WORLDS.with(|w| {
        let worlds = w.borrow();
        if let Some(world) = worlds.get(&(world_id as usize)) {
            let count = world.rigid_body_set.len();
            let mut handles = Vec::with_capacity(count);
            for (handle, rb) in world.rigid_body_set.iter() {
                let (index, gen) = handle.into_raw_parts();
                let handle_raw = ((index as i64) << 32) | (gen as i64);
                let _ = rb;
                handles.push(handle_raw);
            }
            return NumericArray::from_slice(&handles);
        }
        NumericArray::from_slice(&[])
    })
}
