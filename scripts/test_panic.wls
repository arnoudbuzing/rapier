#!/usr/bin/env wolframscript
PacletDirectoryLoad[FileNameJoin[{DirectoryName[$InputFileName, 2], "Rapier"}]];
Needs["Rapier`"];
Print["Init"];
worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];
floorId = RapierAddRigidBody[worldId, {0.0, 0.0, -1.0}, "Fixed"];
RapierAddColliderCuboid[worldId, floorId, {10.0, 10.0, 1.0}, 1.0];
Print["Floor added"];
ballId = RapierAddRigidBody[worldId, {0.0, 0.0, 5.0}, "Dynamic"];
RapierAddColliderSphere[worldId, ballId, 0.5, 1.0];
Print["Ball added"];
RapierWorldStep[worldId, 1, 1.0/60.0];
Print["Step 1"];
RapierWorldStep[worldId, 60, 1.0/60.0];
Print["Step 60"];
