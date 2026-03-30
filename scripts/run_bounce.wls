#!/usr/bin/env wolframscript

PacletDirectoryLoad[FileNameJoin[{DirectoryName[$InputFileName, 2], "Rapier"}]];
Needs["Rapier`"];

Print["--- Starting Rapier 3D Simulation ---"];
worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];

Print["Created World ID: ", worldId];

floorId = RapierAddRigidBody[worldId, {0.0, 0.0, -1.0}, "Fixed"];
RapierAddColliderCuboid[worldId, floorId, {10.0, 10.0, 1.0}, 1.0];

ballId = RapierAddRigidBody[worldId, {0.0, 0.0, 5.0}, "Dynamic"];
RapierAddColliderSphere[worldId, ballId, 0.5, 1.0];

Print["Initial Matrix:"];
Print[RapierGetBodyPositions[worldId]];

Print["Stepping the physics world 60 times..."];
RapierWorldStep[worldId, 60, 1.0 / 60.0];

Print["Final Matrix (Z should drop!):"];
Print[RapierGetBodyPositions[worldId]];

destroyed = RapierWorldDestroy[worldId];
Print["World Destroyed: ", destroyed];
