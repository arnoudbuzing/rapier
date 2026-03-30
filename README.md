# Rapier

A Wolfram Language paclet providing high-performance 3D physics simulation bindings to the Rust [rapier3d](https://rapier.rs/) physics engine via `wolfram-library-link`.

## Overview 

This paclet acts as a wrapper around the Rapier physics engine. To bridge the gap between Mathematica's functional nature and Rapier's stateful dependencies, it implements an internal Handle-Based Global memory map inside the dynamically linked Rust library.

The simulation world is persistent in memory, isolated securely per session. You iteratively create a world, populate it dynamically with bodies and colliders, step the physics pipeline, and retrieve flat matrix transformation arrays back instantly over WSTP for zero-latency 3D visual plotting.

## Building the Project

Ensure you have Rust/Cargo installed, as well as the Wolfram Engine or Mathematica.

1. Navigate to the project root.
2. Run the build script to compile the shared library and automatically route it to your OS-specific `$SystemID` paclet directory:
   ```bash
   ./scripts/build.wls
   ```

## Usage Example

Load the paclet directly in a Wolfram Notebook from your local directory:

```wl
PacletDirectoryLoad[FileNameJoin[{NotebookDirectory[], "Rapier"}]]
Needs["Rapier`"]
```

### Bouncing Sphere Setup

Create a new world with gravity pointing downwards:
```wl
(* Gravity pointing down on Z *)
worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];
```

Add a static ground plane (Cuboid with `10x10x1` half-extents at `Z=-1`):
```wl
floorId = RapierAddRigidBody[worldId, {0.0, 0.0, -1.0}, "Fixed"];
RapierAddColliderCuboid[worldId, floorId, {10.0, 10.0, 1.0}, 1.0];
```

Add a dynamic sphere falling from `Z = 5.0`:
```wl
ballId = RapierAddRigidBody[worldId, {0.0, 0.0, 5.0}, "Dynamic"];
RapierAddColliderSphere[worldId, ballId, 0.5, 1.0];
```

### Simulation & Visualization Loop

You can use `RapierWorldStep` continuously and retrieve `RapierGetBodyPositions` for mapping to interactive Wolfram structures.

Let's cache 200 simulation frames at 60 FPS:

```wl
frames = Table[
  RapierWorldStep[worldId, 1, 1.0 / 60.0];
  RapierGetBodyPositions[worldId],
  {200}
];
```

Now securely animate the state history we extracted!
Note that the returned raw matrix separates bodies by their Handle Index mapping. The Floor is index `1`, the Sphere is index `2`, and columns `2;;4` represent the absolute `X, Y, Z` coordinates!

```wl
ListAnimate[
  Table[
    Graphics3D[{
      (* Visualization for Floor Bounds *)
      Gray, Cuboid[{-10, -10, -2}, {10, 10, 0}],
      (* Translate Sphere object into dynamic coordinates logged at this step *)
      Red,
      Translate[Sphere[{0,0,0}, 0.5], pos[[2, 2;;4]]]
    }, 
    PlotRange -> {{-5, 5}, {-5, 5}, {-2, 6}},
    Axes -> True],
    {pos, frames}
  ],
  AnimationRate -> 60
]
```

### Cleanup
To prevent memory leaks over time during notebook restarts or heavy loads, properly destroy the simulation state when you are finished computing:
```wl
RapierWorldDestroy[worldId]
```
