# RapierAddColliderCylinder

`RapierAddColliderCylinder[worldId, bodyId, {halfHeight, radius}, density]` adds a cylindrical collider to a rigid body.

## Arguments

- `worldId` (`Integer`): Physics world identifier returned by `RapierWorldCreate`.
- `bodyId` (`Integer`): Rigid body handle returned by `RapierAddRigidBody`.
- `{halfHeight, radius}` (`{Real, Real}`): Cylinder dimensions.
  - `halfHeight` is half the cylinder height along its principal axis.
  - `radius` is the cylinder radius.
- `density` (`Real`): Density used for collider mass properties.

## Returns

- `Integer`: Collider handle, or `-1` if the world does not exist.

## Example

```wl
worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];
bodyId = RapierAddRigidBody[worldId, {0.0, 0.0, 2.0}, "Dynamic"];
colliderId = RapierAddColliderCylinder[worldId, bodyId, {0.5, 0.25}, 1.0];
```

## Notes

- Current restitution is fixed internally in the native layer.
- Use `RapierWorldStep` to advance simulation and `RapierGetBodyPositions` to inspect body state.
