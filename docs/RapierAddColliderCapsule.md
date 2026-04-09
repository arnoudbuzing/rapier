# RapierAddColliderCapsule

`RapierAddColliderCapsule[worldId, bodyId, {halfHeight, radius}, density]` adds a capsule collider to a rigid body.

## Arguments

- `worldId` (`Integer`): Physics world identifier returned by `RapierWorldCreate`.
- `bodyId` (`Integer`): Rigid body handle returned by `RapierAddRigidBody`.
- `{halfHeight, radius}` (`{Real, Real}`): Capsule dimensions.
  - `halfHeight` is the half-distance between the centers of the two sphere caps along the Y axis.
  - `radius` is the radius of each spherical cap (and the cylindrical body).
- `density` (`Real`): Density used for collider mass properties.

## Returns

- `Integer`: Collider handle, or `-1` if the world does not exist.

## Example

```wl
worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];
bodyId = RapierAddRigidBody[worldId, {0.0, 0.0, 2.0}, "Dynamic"];
colliderId = RapierAddColliderCapsule[worldId, bodyId, {0.6, 0.25}, 1.0];
```

## Notes

- The capsule is aligned along the Y axis (`capsule_y` variant).
- Total height of the capsule is `2 * (halfHeight + radius)`.
- Capsules are a common choice for character controllers due to smooth collision response.
- Current restitution is fixed internally in the native layer.
- Use `RapierWorldStep` to advance simulation and `RapierGetBodyPositions` to inspect body state.
