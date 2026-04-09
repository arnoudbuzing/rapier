# RapierAddColliderCone

`RapierAddColliderCone[worldId, bodyId, {halfHeight, radius}, density]` adds a conical collider to a rigid body.

## Arguments

- `worldId` (`Integer`): Physics world identifier returned by `RapierWorldCreate`.
- `bodyId` (`Integer`): Rigid body handle returned by `RapierAddRigidBody`.
- `{halfHeight, radius}` (`{Real, Real}`): Cone dimensions.
  - `halfHeight` is half the cone height along its principal axis (Y).
  - `radius` is the base radius of the cone.
- `density` (`Real`): Density used for collider mass properties.

## Returns

- `Integer`: Collider handle, or `-1` if the world does not exist.

## Example

```wl
worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];
bodyId = RapierAddRigidBody[worldId, {0.0, 0.0, 2.0}, "Dynamic"];
colliderId = RapierAddColliderCone[worldId, bodyId, {0.5, 0.3}, 1.0];
```

## Notes

- The cone apex points along the positive Y axis.
- Current restitution is fixed internally in the native layer.
- Use `RapierWorldStep` to advance simulation and `RapierGetBodyPositions` to inspect body state.
