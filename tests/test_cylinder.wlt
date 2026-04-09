VerificationTest[
  PacletDirectoryLoad[FileNameJoin[{DirectoryName[$TestFileName, 2], "Rapier"}]],
  {__String},
  SameTest -> MatchQ,
  TestID -> "PacletDirectoryLoad-Cylinder"
]

VerificationTest[
  Needs["Rapier`"],
  Null,
  TestID -> "NeedsRapier-Cylinder"
]

VerificationTest[
  Module[{worldId, bodyId, colliderId, okDestroy},
    worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];
    bodyId = RapierAddRigidBody[worldId, {0.0, 0.0, 1.0}, "Dynamic"];
    colliderId = RapierAddColliderCylinder[worldId, bodyId, {0.5, 0.25}, 1.0];
    okDestroy = RapierWorldDestroy[worldId];
    {IntegerQ[colliderId], colliderId > 0, okDestroy}
  ],
  {True, True, True},
  TestID -> "RapierAddColliderCylinder-ReturnsValidHandle"
]

VerificationTest[
  Module[{badWorldId = -12345, colliderId},
    colliderId = RapierAddColliderCylinder[badWorldId, -1, {0.5, 0.25}, 1.0];
    colliderId
  ],
  -1,
  TestID -> "RapierAddColliderCylinder-InvalidWorldReturnsMinusOne"
]

VerificationTest[
  Module[{worldId, badBodyId, colliderId, okDestroy},
    worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];
    badBodyId = 999999999;
    colliderId = RapierAddColliderCylinder[worldId, badBodyId, {0.5, 0.25}, 1.0];
    okDestroy = RapierWorldDestroy[worldId];
    {IntegerQ[colliderId], colliderId > 0, okDestroy}
  ],
  {True, True, True},
  TestID -> "RapierAddColliderCylinder-UnknownBodyStillCreatesCollider"
]

VerificationTest[
  Module[{worldId, bodyId, before, after, okDestroy},
    worldId = RapierWorldCreate[{0.0, 0.0, -9.81}];
    bodyId = RapierAddRigidBody[worldId, {0.0, 0.0, 3.0}, "Dynamic"];
    RapierAddColliderCylinder[worldId, bodyId, {0.5, 0.25}, 1.0];
    before = RapierGetBodyPositions[worldId][[1, 4]];
    RapierWorldStep[worldId, 30, 1.0/60.0];
    after = RapierGetBodyPositions[worldId][[1, 4]];
    okDestroy = RapierWorldDestroy[worldId];
    {before > after, okDestroy}
  ],
  {True, True},
  TestID -> "RapierAddColliderCylinder-BodyFallsUnderGravity"
]
