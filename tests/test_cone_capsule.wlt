VerificationTest[
  PacletDirectoryLoad[FileNameJoin[{DirectoryName[$TestFileName, 2], "Rapier"}]],
  {__String},
  SameTest -> MatchQ,
  TestID -> "PacletDirectoryLoad-ConeCapsule"
]

VerificationTest[
  Needs["Rapier`"],
  Null,
  TestID -> "NeedsRapier-ConeCapsule"
]

VerificationTest[
  Module[{worldId, bodyId, coneId, capsuleId, okDestroy},
    worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];
    bodyId = RapierAddRigidBody[worldId, {0.0, 0.0, 2.0}, "Dynamic"];
    coneId = RapierAddColliderCone[worldId, bodyId, {0.5, 0.3}, 1.0];
    capsuleId = RapierAddColliderCapsule[worldId, bodyId, {0.6, 0.25}, 1.0];
    okDestroy = RapierWorldDestroy[worldId];
    {IntegerQ[coneId], coneId > 0, IntegerQ[capsuleId], capsuleId > 0, okDestroy}
  ],
  {True, True, True, True, True},
  TestID -> "RapierAddColliderConeCapsule-ValidHandles"
]

VerificationTest[
  {
    RapierAddColliderCone[-111, -1, {0.5, 0.3}, 1.0],
    RapierAddColliderCapsule[-222, -1, {0.6, 0.25}, 1.0]
  },
  {-1, -1},
  TestID -> "RapierAddColliderConeCapsule-InvalidWorldReturnsMinusOne"
]

VerificationTest[
  Module[{worldId, floorId, coneBodyId, capBodyId, before, after, okDestroy},
    worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];

    floorId = RapierAddRigidBody[worldId, {0.0, 0.0, -1.0}, "Fixed"];
    RapierAddColliderCuboid[worldId, floorId, {8.0, 8.0, 1.0}, 1.0];

    coneBodyId = RapierAddRigidBody[worldId, {-0.8, 0.0, 4.5}, "Dynamic"];
    RapierAddColliderCone[worldId, coneBodyId, {0.5, 0.3}, 1.0];

    capBodyId = RapierAddRigidBody[worldId, {0.8, 0.0, 5.2}, "Dynamic"];
    RapierAddColliderCapsule[worldId, capBodyId, {0.6, 0.25}, 1.0];

    before = RapierGetBodyPositions[worldId][[All, 4]];
    RapierWorldStep[worldId, 60, 1.0/60.0];
    after = RapierGetBodyPositions[worldId][[All, 4]];
    okDestroy = RapierWorldDestroy[worldId];

    {
      before[[2]] > after[[2]],
      before[[3]] > after[[3]],
      okDestroy
    }
  ],
  {True, True, True},
  TestID -> "RapierAddColliderConeCapsule-BodiesFall"
]
