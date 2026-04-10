BeginPackage["PhysicsLink`"];

RapierVersion::usage = "RapierVersion[] returns the version string of the linked rapier3d Rust crate.";
RapierCuboidMass::usage = "RapierCuboidMass[hx, hy, hz, density] returns the theoretical mass of a cuboid.";
RapierWorldCreate::usage = "RapierWorldCreate[{gx, gy, gz}] initializes a new simulation world and returns its ID.";
RapierWorldDestroy::usage = "RapierWorldDestroy[id] cleans up a simulation world.";
RapierAddRigidBody::usage = "RapierAddRigidBody[world_id, {x,y,z}, {qx,qy,qz,qw}, type] adds a rigid body.";
RapierAddColliderCuboid::usage = "RapierAddColliderCuboid[world_id, body_id, {hx,hy,hz}, density] adds a cuboid collider attached to the body.";
RapierAddColliderSphere::usage = "RapierAddColliderSphere[world_id, body_id, radius, density] adds a spherical collider attached to the body.";
RapierAddColliderCylinder::usage = "RapierAddColliderCylinder[world_id, body_id, {halfHeight, radius}, density] adds a cylindrical collider attached to the body.";
RapierAddColliderCone::usage = "RapierAddColliderCone[world_id, body_id, {halfHeight, radius}, density] adds a conical collider attached to the body.";
RapierAddColliderCapsule::usage = "RapierAddColliderCapsule[world_id, body_id, {halfHeight, radius}, density] adds a capsule collider attached to the body.";
QuaternionToTransformation::usage = "QuaternionToTransformation[{qx, qy, qz, qw}] converts a quaternion to a 3x3 rotation matrix.";
RapierWorldStep::usage = "RapierWorldStep[world_id, steps, dt] advances the simulation.";
RapierGetBodyPositions::usage = "RapierGetBodyPositions[world_id] returns all body positions and rotations as a matrix.";

CreatePhysicsModel::usage = "CreatePhysicsModel[{body1, body2, ...}, opts] creates a physics simulation from DynamicBody/FixedBody wrapped primitives. Returns a PhysicsModelObject.";
PhysicsModelIterate::usage = "PhysicsModelIterate[model, steps, dt] advances the physics simulation by steps*dt time and returns an updated PhysicsModelObject.";
PhysicsModelPlot::usage = "PhysicsModelPlot[model] renders a PhysicsModelObject as a Graphics3D scene.";
PhysicsModelObject::usage = "PhysicsModelObject[assoc] represents the state of a physics simulation.";
DynamicBody::usage = "DynamicBody[primitive, opts] wraps a graphics primitive as a dynamic (gravity-affected) rigid body.";
FixedBody::usage = "FixedBody[primitive, opts] wraps a graphics primitive as a fixed (immovable) rigid body.";
DestroyPhysicsModel::usage = "DestroyPhysicsModel[model] destroys the underlying Rapier world and frees resources.";
PhysicsModelVideo::usage = "PhysicsModelVideo[frames] creates an AnimationVideo from a list of PhysicsModelObject frames.";
PhysicsModelEvolve::usage = "PhysicsModelEvolve[model, frames, dt] evolves the model for the given number of frames at time step dt, returning a list of PhysicsModelObject states.";
PhysicsBoundaryBox::usage = "PhysicsBoundaryBox[{{xmin,ymin,zmin},{xmax,ymax,zmax}}] returns a list of FixedBody walls enclosing the given region. Use inside CreatePhysicsModel.";

Begin["`Private`"];

QuaternionToTransformation[{qx_?NumericQ, qy_?NumericQ, qz_?NumericQ, qw_?NumericQ}] := {
  {1 - 2 (qy^2 + qz^2), 2 (qx qy - qw qz), 2 (qx qz + qw qy)},
  {2 (qx qy + qw qz), 1 - 2 (qx^2 + qz^2), 2 (qy qz - qw qx)},
  {2 (qx qz - qw qy), 2 (qy qz + qw qx), 1 - 2 (qx^2 + qy^2)}
};

(* --- Library Loading logic --- *)

$physicsLinkPacletDir = DirectoryName[$InputFileName, 2];

physicsLinkLibraryPath[] := Module[{ext, libName = "librapier_rs"},
  ext = Switch[$OperatingSystem, "MacOSX", ".dylib", "Unix", ".so", "Windows", ".dll"];
  FileNameJoin[{$physicsLinkPacletDir, "LibraryResources", $SystemID, libName <> ext}]
];

loadPhysicsLinkLibrary[] := Module[{libPath},
  libPath = physicsLinkLibraryPath[];
  If[FileExistsQ[libPath],
    libPath,
    $Failed
  ]
];

(* --- Function definition using LibraryLink --- *)

$physicsLinkLib = loadPhysicsLinkLibrary[];

If[$physicsLinkLib =!= $Failed,
  $iRapierVersion = LibraryFunctionLoad[$physicsLinkLib, "rapier_version", {}, "UTF8String"];
  $iRapierCuboidMass = LibraryFunctionLoad[$physicsLinkLib, "rapier_cuboid_mass", {Real, Real, Real, Real}, Real];
  
  $iRapierWorldCreate = LibraryFunctionLoad[$physicsLinkLib, "rapier_world_create", {Real, Real, Real}, Integer];
  $iRapierWorldDestroy = LibraryFunctionLoad[$physicsLinkLib, "rapier_world_destroy", {Integer}, "Boolean"];
  $iRapierAddRigidBody = LibraryFunctionLoad[$physicsLinkLib, "rapier_add_rigid_body", {Integer, Real, Real, Real, Real, Real, Real, Real, Integer}, Integer];
  $iRapierAddColliderCuboid = LibraryFunctionLoad[$physicsLinkLib, "rapier_add_collider_cuboid", {Integer, Integer, Real, Real, Real, Real}, Integer];
  $iRapierAddColliderSphere = LibraryFunctionLoad[$physicsLinkLib, "rapier_add_collider_sphere", {Integer, Integer, Real, Real}, Integer];
  $iRapierAddColliderCylinder = LibraryFunctionLoad[$physicsLinkLib, "rapier_add_collider_cylinder", {Integer, Integer, Real, Real, Real}, Integer];
  $iRapierAddColliderCone = LibraryFunctionLoad[$physicsLinkLib, "rapier_add_collider_cone", {Integer, Integer, Real, Real, Real}, Integer];
  $iRapierAddColliderCapsule = LibraryFunctionLoad[$physicsLinkLib, "rapier_add_collider_capsule", {Integer, Integer, Real, Real, Real}, Integer];
  $iRapierWorldStep = LibraryFunctionLoad[$physicsLinkLib, "rapier_world_step", {Integer, Integer, Real}, "Void"];
  $iRapierGetBodyPositions = LibraryFunctionLoad[$physicsLinkLib, "rapier_get_body_positions", {Integer}, {Real, 1}];
  $iRapierGetBodyHandles = LibraryFunctionLoad[$physicsLinkLib, "rapier_get_body_handles", {Integer}, {Integer, 1}];

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

  RapierAddColliderCone[worldId_Integer, bodyId_Integer, {halfHeight_?NumericQ, radius_?NumericQ}, density_?NumericQ] :=
    $iRapierAddColliderCone[worldId, bodyId, halfHeight, radius, density];

  RapierAddColliderCapsule[worldId_Integer, bodyId_Integer, {halfHeight_?NumericQ, radius_?NumericQ}, density_?NumericQ] :=
    $iRapierAddColliderCapsule[worldId, bodyId, halfHeight, radius, density];

  RapierWorldStep[worldId_Integer, steps_Integer, dt_?NumericQ] := 
    $iRapierWorldStep[worldId, steps, dt];
    
  RapierGetBodyPositions[worldId_Integer] := 
    Module[{flatPoses = $iRapierGetBodyPositions[worldId], flatHandles = $iRapierGetBodyHandles[worldId], poses},
      If[Length[flatPoses] > 0 && Length[flatHandles] > 0,
        poses = ArrayReshape[flatPoses, {Length[flatPoses]/7, 7}];
        MapThread[Prepend[#1, #2] &, {poses, flatHandles}],
        {}
      ]
    ];

,
  Print["Warning: Rapier library not found at: ", physicsLinkLibraryPath[]];
];

(* =====================================================
   High-Level Physics Model API
   ===================================================== *)

(* --- rotationMatrixToQuaternion: 3x3 rotation matrix -> {qx, qy, qz, qw} --- *)
(* Shepperd's method for numerical stability *)
rotationMatrixToQuaternion[mat_] := Module[
  {m = N[mat], t, qw, qx, qy, qz, s},
  t = Tr[m];
  Which[
    t > 0,
      s = Sqrt[t + 1.0] * 2.0;
      qw = 0.25 * s;
      qx = (m[[3, 2]] - m[[2, 3]]) / s;
      qy = (m[[1, 3]] - m[[3, 1]]) / s;
      qz = (m[[2, 1]] - m[[1, 2]]) / s,
    m[[1, 1]] > m[[2, 2]] && m[[1, 1]] > m[[3, 3]],
      s = Sqrt[1.0 + m[[1, 1]] - m[[2, 2]] - m[[3, 3]]] * 2.0;
      qw = (m[[3, 2]] - m[[2, 3]]) / s;
      qx = 0.25 * s;
      qy = (m[[1, 2]] + m[[2, 1]]) / s;
      qz = (m[[1, 3]] + m[[3, 1]]) / s,
    m[[2, 2]] > m[[3, 3]],
      s = Sqrt[1.0 + m[[2, 2]] - m[[1, 1]] - m[[3, 3]]] * 2.0;
      qw = (m[[1, 3]] - m[[3, 1]]) / s;
      qx = (m[[1, 2]] + m[[2, 1]]) / s;
      qy = 0.25 * s;
      qz = (m[[2, 3]] + m[[3, 2]]) / s,
    True,
      s = Sqrt[1.0 + m[[3, 3]] - m[[1, 1]] - m[[2, 2]]] * 2.0;
      qw = (m[[2, 1]] - m[[1, 2]]) / s;
      qx = (m[[1, 3]] + m[[3, 1]]) / s;
      qy = (m[[2, 3]] + m[[3, 2]]) / s;
      qz = 0.25 * s
  ];
  {qx, qy, qz, qw}
];

(* --- rotationFromAxisToY: compute rotation that maps a given axis direction to the Y-axis --- *)
rotationFromAxisToY[axis_] := Module[
  {dir, yAxis = {0, 1, 0}, crossP, dotP, angle},
  dir = Normalize[N[axis]];
  dotP = dir . yAxis;
  If[Abs[dotP - 1.0] < 1.0*^-10,
    (* Already aligned with Y *)
    Return[IdentityMatrix[3]]
  ];
  If[Abs[dotP + 1.0] < 1.0*^-10,
    (* Anti-aligned: rotate 180 degrees around Z *)
    Return[RotationMatrix[Pi, {0, 0, 1}] // N]
  ];
  angle = ArcCos[Clip[dotP, {-1.0, 1.0}]];
  crossP = Normalize[Cross[dir, yAxis]];
  N[RotationMatrix[angle, crossP]]
];

(* --- extractPrimitiveInfo: pattern-match WL primitives to {center, shapeParams, intrinsicRotation} --- *)

(* Sphere[{x,y,z}, r] *)
extractPrimitiveInfo[Sphere[center:{_?NumericQ, _?NumericQ, _?NumericQ}, r_?NumericQ]] :=
  {N[center], {"Sphere", N[r]}, IdentityMatrix[3]};

(* Sphere[{x,y,z}] - default radius 1 *)
extractPrimitiveInfo[Sphere[center:{_?NumericQ, _?NumericQ, _?NumericQ}]] :=
  {N[center], {"Sphere", 1.0}, IdentityMatrix[3]};

(* Cuboid[{x1,y1,z1}, {x2,y2,z2}] *)
extractPrimitiveInfo[Cuboid[pmin:{_?NumericQ, _?NumericQ, _?NumericQ}, pmax:{_?NumericQ, _?NumericQ, _?NumericQ}]] :=
  Module[{center, halfExtents},
    center = N[(pmin + pmax) / 2];
    halfExtents = N[Abs[(pmax - pmin) / 2]];
    {center, {"Cuboid", halfExtents}, IdentityMatrix[3]}
  ];

(* Cylinder[{{x1,y1,z1},{x2,y2,z2}}, r] *)
extractPrimitiveInfo[Cylinder[{p1:{_?NumericQ, _?NumericQ, _?NumericQ}, p2:{_?NumericQ, _?NumericQ, _?NumericQ}}, r_?NumericQ]] :=
  Module[{center, axis, halfHeight, rot},
    center = N[(p1 + p2) / 2];
    axis = N[p2 - p1];
    halfHeight = Norm[axis] / 2.0;
    rot = rotationFromAxisToY[axis];
    {center, {"Cylinder", halfHeight, N[r]}, rot}
  ];

(* Cone[{{x1,y1,z1},{x2,y2,z2}}, r] *)
extractPrimitiveInfo[Cone[{p1:{_?NumericQ, _?NumericQ, _?NumericQ}, p2:{_?NumericQ, _?NumericQ, _?NumericQ}}, r_?NumericQ]] :=
  Module[{center, axis, halfHeight, rot},
    center = N[(p1 + p2) / 2];
    axis = N[p2 - p1];
    halfHeight = Norm[axis] / 2.0;
    rot = rotationFromAxisToY[axis];
    {center, {"Cone", halfHeight, N[r]}, rot}
  ];

(* CapsuleShape[{{x1,y1,z1},{x2,y2,z2}}, r] *)
extractPrimitiveInfo[CapsuleShape[{p1:{_?NumericQ, _?NumericQ, _?NumericQ}, p2:{_?NumericQ, _?NumericQ, _?NumericQ}}, r_?NumericQ]] :=
  Module[{center, axis, halfHeight, rot},
    center = N[(p1 + p2) / 2];
    axis = N[p2 - p1];
    halfHeight = Norm[axis] / 2.0;
    rot = rotationFromAxisToY[axis];
    {center, {"Capsule", halfHeight, N[r]}, rot}
  ];

(* --- createBody: register a single body+collider from a DynamicBody/FixedBody wrapper --- *)
(* Extract graphics directives from a list like {Red, Opacity[.5], Cuboid[...]} *)
separateDirectivesAndPrimitive[prim_List] :=
  Module[{directives, primitives},
    directives = Select[prim, !MatchQ[#, _Sphere | _Cuboid | _Cylinder | _Cone | _CapsuleShape] &];
    primitives = Select[prim, MatchQ[#, _Sphere | _Cuboid | _Cylinder | _Cone | _CapsuleShape] &];
    If[Length[primitives] =!= 1,
      Print["Expected exactly one graphics primitive, found: ", Length[primitives]];
      Return[$Failed]
    ];
    {directives, primitives[[1]]}
  ];

separateDirectivesAndPrimitive[prim_] := {{}, prim};

(* Multiple bare arguments: wrap into a list so directives get separated properly *)
createBody[worldId_Integer, DynamicBody[args__, opts:OptionsPattern[]]] :=
  iCreateBody[worldId, {args}, "Dynamic", {opts}] /; !MatchQ[{args}, {_List | _Sphere | _Cuboid | _Cylinder | _Cone | _CapsuleShape}];

createBody[worldId_Integer, FixedBody[args__, opts:OptionsPattern[]]] :=
  iCreateBody[worldId, {args}, "Fixed", {opts}] /; !MatchQ[{args}, {_List | _Sphere | _Cuboid | _Cylinder | _Cone | _CapsuleShape}];

createBody[worldId_Integer, DynamicBody[prim_, opts___]] :=
  iCreateBody[worldId, prim, "Dynamic", {opts}];

createBody[worldId_Integer, FixedBody[prim_, opts___]] :=
  iCreateBody[worldId, prim, "Fixed", {opts}];

iCreateBody[worldId_Integer, prim_, bodyType_String, opts_List] :=
  Module[{separated, directives, actualPrim, info, center, shapeParams, intrinsicRot, userRot, combinedRot, quat,
          density, handle, colliderHandle, shapeType},
    separated = separateDirectivesAndPrimitive[prim];
    If[separated === $Failed, Return[$Failed]];
    {directives, actualPrim} = separated;

    info = extractPrimitiveInfo[actualPrim];
    If[!ListQ[info],
      Print["Unsupported primitive: ", Head[prim]];
      Return[$Failed]
    ];
    {center, shapeParams, intrinsicRot} = info;
    shapeType = shapeParams[[1]];

    (* Get user options *)
    density = N["Density" /. opts /. {"Density" -> 1.0}];
    userRot = "Orientation" /. opts /. {"Orientation" -> IdentityMatrix[3]};
    userRot = N[userRot];

    (* Combined rotation: user rotation composed with intrinsic rotation *)
    combinedRot = userRot . intrinsicRot;
    quat = rotationMatrixToQuaternion[combinedRot];

    (* Add rigid body *)
    handle = RapierAddRigidBody[worldId, center, quat, bodyType];

    (* Add collider based on shape type *)
    colliderHandle = Switch[shapeType,
      "Sphere",
        RapierAddColliderSphere[worldId, handle, shapeParams[[2]], density],
      "Cuboid",
        RapierAddColliderCuboid[worldId, handle, shapeParams[[2]], density],
      "Cylinder",
        RapierAddColliderCylinder[worldId, handle, {shapeParams[[2]], shapeParams[[3]]}, density],
      "Cone",
        RapierAddColliderCone[worldId, handle, {shapeParams[[2]], shapeParams[[3]]}, density],
      "Capsule",
        RapierAddColliderCapsule[worldId, handle, {shapeParams[[2]], shapeParams[[3]]}, density]
    ];

    <|
      "Handle" -> handle,
      "BodyType" -> bodyType,
      "ShapeType" -> shapeType,
      "ShapeParams" -> Rest[shapeParams],
      "IntrinsicRotation" -> intrinsicRot,
      "Directives" -> directives,
      "Position" -> center,
      "Quaternion" -> quat
    |>
  ];

(* --- CreatePhysicsModel --- *)
Options[CreatePhysicsModel] = {"Gravity" -> {0, 0, -9.81}, "Graphics3DOptions" -> {}};

CreatePhysicsModel[bodies_List, opts:OptionsPattern[]] :=
  Module[{flatBodies, gravity, worldId, bodyData, g3dOpts},
    flatBodies = Flatten[bodies];
    gravity = N[OptionValue["Gravity"]];
    g3dOpts = OptionValue["Graphics3DOptions"];
    worldId = RapierWorldCreate[gravity];

    bodyData = Table[
      createBody[worldId, body],
      {body, flatBodies}
    ];
    bodyData = Select[bodyData, AssociationQ];

    Module[{model, icon},
      model = PhysicsModelObject[<|
        "WorldID" -> worldId,
        "Bodies" -> bodyData,
        "Time" -> 0.0,
        "Graphics3DOptions" -> g3dOpts,
        "Icon" -> None
      |>];
      icon = Quiet @ Rasterize[PhysicsModelPlot[model, Axes -> False, Boxed -> False], "Image", ImageSize -> 80];
      PhysicsModelObject[Append[model[[1]], "Icon" -> icon]]
    ]
  ];

(* --- PhysicsModelObject formatting --- *)
PhysicsModelObject /: MakeBoxes[obj:PhysicsModelObject[assoc_Association], StandardForm] :=
  Module[{nBodies, nDyn, nFixed, t, icon, alwaysVisible, sometimesVisible, shapeTypes, gravity},
    nBodies = Length[assoc["Bodies"]];
    nDyn = Count[assoc["Bodies"], _?(#["BodyType"] === "Dynamic" &)];
    nFixed = nBodies - nDyn;
    t = assoc["Time"];
    icon = If[ImageQ[assoc["Icon"]], assoc["Icon"], ConstantImage[Blue, {40, 40}]];

    alwaysVisible = {
      {BoxForm`SummaryItem[{"Dynamic bodies: ", nDyn}]},
      {BoxForm`SummaryItem[{"Fixed bodies: ", nFixed}]},
      {BoxForm`SummaryItem[{"Time: ", NumberForm[t, {5, 3}]}]}
    };

    shapeTypes = Counts[#["ShapeType"] & /@ assoc["Bodies"]];
    gravity = Lookup[assoc, "Gravity", "N/A"];

    sometimesVisible = {
      {BoxForm`SummaryItem[{"World ID: ", assoc["WorldID"]}]},
      {BoxForm`SummaryItem[{"Shape types: ", shapeTypes}]}
    };

    BoxForm`ArrangeSummaryBox[
      PhysicsModelObject,
      obj,
      icon,
      alwaysVisible,
      sometimesVisible,
      StandardForm
    ]
  ];

(* --- PhysicsModelIterate --- *)
PhysicsModelIterate[PhysicsModelObject[assoc_Association], steps_Integer, dt_?NumericQ] :=
  Module[{worldId, positions, handleToState, updatedBodies},
    worldId = assoc["WorldID"];

    RapierWorldStep[worldId, steps, dt];
    positions = RapierGetBodyPositions[worldId];

    (* Build handle -> {position, quaternion} lookup *)
    handleToState = Association @ Map[
      (#[[1]] -> <|"Position" -> #[[2 ;; 4]], "Quaternion" -> #[[5 ;; 8]]|>) &,
      positions
    ];

    (* Update body data *)
    updatedBodies = Map[
      Module[{body = #, h, state},
        h = body["Handle"];
        state = Lookup[handleToState, h, Missing[]];
        If[AssociationQ[state],
          Append[body, {
            "Position" -> state["Position"],
            "Quaternion" -> state["Quaternion"]
          }],
          body
        ]
      ] &,
      assoc["Bodies"]
    ];

    PhysicsModelObject[<|
      "WorldID" -> worldId,
      "Bodies" -> updatedBodies,
      "Time" -> assoc["Time"] + steps * N[dt],
      "Graphics3DOptions" -> Lookup[assoc, "Graphics3DOptions", {}],
      "Icon" -> assoc["Icon"]
    |>]
  ];

(* --- bodyToGraphics: map body to origin-centered WL primitive for GeometricTransformation --- *)
bodyToGraphics[body_Association] :=
  Module[{shapeType, params},
    shapeType = body["ShapeType"];
    params = body["ShapeParams"];
    Switch[shapeType,
      "Sphere",
        Sphere[{0, 0, 0}, params[[1]]],
      "Cuboid",
        Module[{he = params[[1]]},
          Cuboid[-he, he]
        ],
      "Cylinder",
        Module[{hh = params[[1]], r = params[[2]]},
          Cylinder[{{0, -hh, 0}, {0, hh, 0}}, r]
        ],
      "Cone",
        Module[{hh = params[[1]], r = params[[2]]},
          Cone[{{0, -hh, 0}, {0, hh, 0}}, r]
        ],
      "Capsule",
        Module[{hh = params[[1]], r = params[[2]]},
          CapsuleShape[{{0, -hh, 0}, {0, hh, 0}}, r]
        ]
    ]
  ];

(* --- PhysicsModelPlot --- *)
$defaultColors = {
  RGBColor[0.4, 0.6, 0.8],   (* steel blue *)
  RGBColor[0.9, 0.5, 0.2],   (* orange *)
  RGBColor[0.3, 0.7, 0.3],   (* green *)
  RGBColor[0.8, 0.3, 0.3],   (* red *)
  RGBColor[0.6, 0.4, 0.8],   (* purple *)
  RGBColor[0.9, 0.8, 0.2],   (* yellow *)
  RGBColor[0.4, 0.8, 0.8]    (* cyan *)
};

Options[PhysicsModelPlot] = {PlotRange -> Automatic, Axes -> False, Boxed -> False};

PhysicsModelPlot[PhysicsModelObject[assoc_Association], opts:OptionsPattern[]] :=
  Module[{bodies, primitives, nColors, storedOpts, mergedOpts, plotRange, axes, boxed},
    bodies = assoc["Bodies"];
    nColors = Length[$defaultColors];
    storedOpts = Lookup[assoc, "Graphics3DOptions", {}];

    (* Option precedence: explicit opts > stored Graphics3DOptions > PhysicsModelPlot defaults *)
    plotRange = OptionValue[PlotRange] /. Automatic -> (PlotRange /. storedOpts /. {PlotRange -> Automatic});
    axes = OptionValue[Axes] /. Automatic :> (Axes /. storedOpts /. {Axes -> True});
    boxed = OptionValue[Boxed] /. Automatic :> (Boxed /. storedOpts /. {Boxed -> True});

    primitives = MapIndexed[
      Module[{body = #1, idx = #2[[1]], prim, rotMat, pos, directives, defaultColor, transform},
        prim = bodyToGraphics[body];
        rotMat = QuaternionToTransformation[body["Quaternion"]];
        pos = body["Position"];
        transform = {rotMat, pos};

        directives = body["Directives"];
        (* If no user directives, use default coloring *)
        If[directives === {} || !ListQ[directives],
          defaultColor = If[body["BodyType"] === "Fixed",
            GrayLevel[0.7],
            $defaultColors[[Mod[idx - 1, nColors] + 1]]
          ];
          directives = {defaultColor}
        ];

        {Sequence @@ directives, GeometricTransformation[prim, transform]}
      ] &,
      bodies
    ];

    mergedOpts = FilterRules[
      Join[{opts}, storedOpts],
      Options[Graphics3D]
    ];

    Graphics3D[
      primitives,
      PlotRange -> plotRange,
      Axes -> axes,
      Boxed -> boxed,
      Lighting -> "Neutral",
      Sequence @@ mergedOpts
    ]
  ];

(* --- DestroyPhysicsModel --- *)
DestroyPhysicsModel[PhysicsModelObject[assoc_Association]] :=
  RapierWorldDestroy[assoc["WorldID"]];

(* --- PhysicsBoundaryBox --- *)
Options[PhysicsBoundaryBox] = {"Thickness" -> 0.01, "Directives" -> {Opacity[0]}};

PhysicsBoundaryBox[{pmin:{_?NumericQ, _?NumericQ, _?NumericQ}, pmax:{_?NumericQ, _?NumericQ, _?NumericQ}}, opts:OptionsPattern[]] :=
  Module[{t, d, xmin, ymin, zmin, xmax, ymax, zmax},
    t = OptionValue["Thickness"];
    d = OptionValue["Directives"];
    {xmin, ymin, zmin} = pmin;
    {xmax, ymax, zmax} = pmax;
    {
      FixedBody[{Sequence @@ d, Cuboid[{xmin, ymin, zmin - t}, {xmax, ymax, zmin}]}],
      FixedBody[{Sequence @@ d, Cuboid[{xmin, ymin, zmax}, {xmax, ymax, zmax + t}]}],
      FixedBody[{Sequence @@ d, Cuboid[{xmin - t, ymin, zmin}, {xmin, ymax, zmax}]}],
      FixedBody[{Sequence @@ d, Cuboid[{xmax, ymin, zmin}, {xmax + t, ymax, zmax}]}],
      FixedBody[{Sequence @@ d, Cuboid[{xmin, ymin - t, zmin}, {xmax, ymin, zmax}]}],
      FixedBody[{Sequence @@ d, Cuboid[{xmin, ymax, zmin}, {xmax, ymax + t, zmax}]}]
    }
  ];

(* --- PhysicsModelEvolve --- *)
PhysicsModelEvolve[model_PhysicsModelObject, frames_Integer, dt_?NumericQ] :=
  Module[{current = model, result},
    result = Table[
      current = PhysicsModelIterate[current, 1, N[dt]];
      current,
      {frames}
    ];
    result
  ];

(* --- PhysicsModelVideo --- *)
Options[PhysicsModelVideo] = {PlotRange -> {{-4.1, 4.1}, {-4.1, 4.1}, {-1.1, 5.1}}};

PhysicsModelVideo[frames_List, opts:OptionsPattern[]] :=
  Module[{plotOpts},
    plotOpts = FilterRules[{opts, Options[PhysicsModelVideo]}, Options[PhysicsModelPlot]];
    AnimationVideo[
      PhysicsModelPlot[frame, Sequence @@ plotOpts],
      {frame, frames}
    ]
  ];

End[];
EndPackage[];
