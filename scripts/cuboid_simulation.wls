#!/usr/bin/env wolframscript

PacletDirectoryLoad[FileNameJoin[{DirectoryName[$InputFileName, 2], "Rapier"}]];
Needs["Rapier`"];

Print["--- Rapier Cuboid Simulation ---"];

(* 1. Initialize World with Gravity *)
worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];
Print["World ID: ", worldId];

(* 2. Create Fixed Floor *)
(* Positioned at z = -0.1 with half-thickness 0.1 -> Top surface at z = 0 *)
floorBodyId = RapierAddRigidBody[worldId, {0.0, 0.0, -0.1}, "Fixed"];
RapierAddColliderCuboid[worldId, floorBodyId, {10.0, 10.0, 0.1}, 1.0];
Print["Floor created (Fixed)"];

(* 3. Create Dynamic Cuboid *)
(* Edge size 2 -> Half-extents {1,1,1} *)
(* Random initial orientation *)
randomQuat = Normalize[RandomReal[{-1, 1}, 4]];
cuboidBodyId = RapierAddRigidBody[worldId, {0.0, 0.0, 5.0}, randomQuat, "Dynamic"];
RapierAddColliderCuboid[worldId, cuboidBodyId, {1.0, 1.0, 1.0}, 1.0];
Print["Dynamic cuboid created at {0,0,5} with edge size 2"];
Print["Initial Orientation (qx, qy, qz, qw): ", randomQuat];

(* 4. Simulation Loop *)
stepSize = 1.0/60.0;
totalSteps = 120; (* 2 seconds of simulation *)

Print["Simulating 2 seconds (120 steps)..."];
Do[
  RapierWorldStep[worldId, 1, stepSize];
  If[Mod[i, 30] == 0,
    bodies = RapierGetBodyPositions[worldId];
    (* Cuboid is the second body added *)
    pos = bodies[[2, 2;;4]];
    Print["Step ", i, ": Cuboid Z-position = ", pos[[3]]];
  ],
  {i, totalSteps}
];

finalBodies = RapierGetBodyPositions[worldId];
Print["\nFinal State:"];
Print[TableForm[finalBodies, TableHeadings -> {None, {"Handle", "X", "Y", "Z", "QX", "QY", "QZ", "QW"}}]];

(* 5. Cleanup *)
RapierWorldDestroy[worldId];
Print["\nSimulation Complete."];
