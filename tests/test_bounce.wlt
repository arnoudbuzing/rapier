VerificationTest[
  PacletDirectoryLoad[FileNameJoin[{DirectoryName[$TestFileName, 2], "Rapier"}]],
  {__String},
  SameTest -> MatchQ,
  TestID -> "PacletDirectoryLoad-Bounce"
]

VerificationTest[
  Needs["Rapier`"],
  Null,
  TestID -> "NeedsRapier-Bounce"
]

VerificationTest[
  Module[{worldId, floorId, ballId, positionsStart, positionsEnd},
    worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];
    
    (* Floor at Z = -1, Fixed *)
    floorId = RapierAddRigidBody[worldId, {0.0, 0.0, -1.0}, "Fixed"];
    RapierAddColliderCuboid[worldId, floorId, {10.0, 10.0, 1.0}, 1.0];
    
    (* Ball at Z = 5.0, Dynamic *)
    ballId = RapierAddRigidBody[worldId, {0.0, 0.0, 5.0}, "Dynamic"];
    RapierAddColliderSphere[worldId, ballId, 0.5, 1.0];
    
    positionsStart = RapierGetBodyPositions[worldId];
    
    (* Step physics world 60 times at 60 FPS (1 second) *)
    RapierWorldStep[worldId, 60, 1.0 / 60.0];
    
    positionsEnd = RapierGetBodyPositions[worldId];
    
    RapierWorldDestroy[worldId];
    
    (* Verify ball is falling (Z is decreasing) *)
    {positionsStart[[2, 4]], positionsEnd[[2, 4]], positionsStart[[2, 4]] > positionsEnd[[2, 4]]}
  ],
  {5.0, current_ /; current < 5.0, True},
  SameTest -> MatchQ,
  TestID -> "RapierDropBallZ-Decrease"
]
