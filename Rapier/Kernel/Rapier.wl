BeginPackage["Rapier`"];

RapierVersion::usage = "RapierVersion[] returns the version string of the linked rapier3d Rust crate.";
RapierCuboidMass::usage = "RapierCuboidMass[hx, hy, hz, density] returns the theoretical mass of a cuboid.";
RapierWorldCreate::usage = "RapierWorldCreate[{gx, gy, gz}] initializes a new simulation world and returns its ID.";
RapierWorldDestroy::usage = "RapierWorldDestroy[id] cleans up a simulation world.";
RapierAddRigidBody::usage = "RapierAddRigidBody[world_id, {x,y,z}, {qx,qy,qz,qw}, type] adds a rigid body.";
RapierAddColliderCuboid::usage = "RapierAddColliderCuboid[world_id, body_id, {hx,hy,hz}, density] adds a cuboid collider attached to the body.";
RapierAddColliderSphere::usage = "RapierAddColliderSphere[world_id, body_id, radius, density] adds a spherical collider attached to the body.";
RapierAddColliderCylinder::usage = "RapierAddColliderCylinder[world_id, body_id, {halfHeight, radius}, density] adds a cylindrical collider attached to the body.";
RapierWorldStep::usage = "RapierWorldStep[world_id, steps, dt] advances the simulation.";
RapierGetBodyPositions::usage = "RapierGetBodyPositions[world_id] returns all body positions and rotations as a matrix.";

Begin["`Private`"];

(* --- Library Loading logic --- *)

$rapierPacletDir = DirectoryName[$InputFileName, 2];

rapierLibraryPath[] := Module[{ext, libName = "librapier_rs"},
  ext = Switch[$OperatingSystem, "MacOSX", ".dylib", "Unix", ".so", "Windows", ".dll"];
  FileNameJoin[{$rapierPacletDir, "LibraryResources", $SystemID, libName <> ext}]
];

loadRapierLibrary[] := Module[{libPath},
  libPath = rapierLibraryPath[];
  If[FileExistsQ[libPath],
    libPath,
    $Failed
  ]
];

(* --- Function definition using LibraryLink --- *)

$rapierLib = loadRapierLibrary[];

If[$rapierLib =!= $Failed,
  $iRapierVersion = LibraryFunctionLoad[$rapierLib, "rapier_version", {}, "UTF8String"];
  $iRapierCuboidMass = LibraryFunctionLoad[$rapierLib, "rapier_cuboid_mass", {Real, Real, Real, Real}, Real];
  
  $iRapierWorldCreate = Echo @ LibraryFunctionLoad[$rapierLib, "rapier_world_create", {Real, Real, Real}, Integer];
  $iRapierWorldDestroy = Echo @ LibraryFunctionLoad[$rapierLib, "rapier_world_destroy", {Integer}, "Boolean"];
  $iRapierAddRigidBody = Echo @ LibraryFunctionLoad[$rapierLib, "rapier_add_rigid_body", {Integer, Real, Real, Real, Real, Real, Real, Real, Integer}, Integer];
  $iRapierAddColliderCuboid = Echo @ LibraryFunctionLoad[$rapierLib, "rapier_add_collider_cuboid", {Integer, Integer, Real, Real, Real, Real}, Integer];
  $iRapierAddColliderSphere = Echo @ LibraryFunctionLoad[$rapierLib, "rapier_add_collider_sphere", {Integer, Integer, Real, Real}, Integer];
  $iRapierAddColliderCylinder = Echo @ LibraryFunctionLoad[$rapierLib, "rapier_add_collider_cylinder", {Integer, Integer, Real, Real, Real}, Integer];
  $iRapierWorldStep = Echo @ LibraryFunctionLoad[$rapierLib, "rapier_world_step", {Integer, Integer, Real}, "Void"];
  $iRapierGetBodyPositions = Echo @ LibraryFunctionLoad[$rapierLib, "rapier_get_body_positions", {Integer}, {Real, 1}];

  RapierVersion[] := $iRapierVersion[];
  RapierCuboidMass[hx_?NumericQ, hy_?NumericQ, hz_?NumericQ, density_?NumericQ] := 
    $iRapierCuboidMass[hx, hy, hz, density];

  RapierWorldCreate[{gx_?NumericQ, gy_?NumericQ, gz_?NumericQ}] := $iRapierWorldCreate[gx, gy, gz];
  RapierWorldDestroy[id_Integer] := $iRapierWorldDestroy[id];
  
  RapierAddRigidBody[worldId_Integer, {x_?NumericQ, y_?NumericQ, z_?NumericQ}, {qx_?NumericQ, qy_?NumericQ, qz_?NumericQ, qw_?NumericQ}, type_String] :=
    Module[{typeId},
      typeId = Switch[type, "Dynamic", 0, "Fixed", 1, "KinematicPosition", 2, "KinematicVelocity", 3, _, 1];
      $iRapierAddRigidBody[worldId, x, y, z, qx, qy, qz, qw, typeId]
    ];
    
  (* Overload to allow default rotation {0,0,0,1} *)
  RapierAddRigidBody[worldId_Integer, {x_?NumericQ, y_?NumericQ, z_?NumericQ}, type_String] :=
    RapierAddRigidBody[worldId, {x, y, z}, {0.0, 0.0, 0.0, 1.0}, type];

  RapierAddColliderCuboid[worldId_Integer, bodyId_Integer, {hx_?NumericQ, hy_?NumericQ, hz_?NumericQ}, density_?NumericQ] :=
    $iRapierAddColliderCuboid[worldId, bodyId, hx, hy, hz, density];

  RapierAddColliderSphere[worldId_Integer, bodyId_Integer, radius_?NumericQ, density_?NumericQ] :=
    $iRapierAddColliderSphere[worldId, bodyId, radius, density];

  RapierAddColliderCylinder[worldId_Integer, bodyId_Integer, {halfHeight_?NumericQ, radius_?NumericQ}, density_?NumericQ] :=
    $iRapierAddColliderCylinder[worldId, bodyId, halfHeight, radius, density];

  RapierWorldStep[worldId_Integer, steps_Integer, dt_?NumericQ] := 
    $iRapierWorldStep[worldId, steps, dt];
    
  RapierGetBodyPositions[worldId_Integer] := 
    Module[{flat = $iRapierGetBodyPositions[worldId]},
      If[Length[flat] > 0,
        ArrayReshape[flat, {Length[flat]/8, 8}],
        {}
      ]
    ];

,
  Print["Warning: Rapier library not found at: ", rapierLibraryPath[]];
];

End[];
EndPackage[];
