#!/usr/bin/env wolframscript

$projectDir = ExpandFileName[DirectoryName[$InputFileName, 2]];
$rustDir = FileNameJoin[{$projectDir, "rapier-rs"}];
$pacletDir = FileNameJoin[{$projectDir, "Rapier"}];
$libResourcesDir = FileNameJoin[{$pacletDir, "LibraryResources", $SystemID}];

Print["Project Dir: ", $projectDir];
Print["Rust Dir: ", $rustDir];
Print["Target Dir: ", $libResourcesDir];

Print["Building Rust library..."];
$res = RunProcess[{"cargo", "build", "--release"}, ProcessDirectory -> $rustDir];
If[$res["ExitCode"] =!= 0,
  Print["Error: cargo build failed!"];
  Print[$res["StandardError"]];
  Exit[1];
];

$libName = "librapier_rs" <> Switch[$OperatingSystem, "MacOSX", ".dylib", "Unix", ".so", "Windows", ".dll"];
$builtLib = FileNameJoin[{$rustDir, "target", "release", $libName}];

If[FileExistsQ[$builtLib],
  RunProcess[{"mkdir", "-p", $libResourcesDir}];
  $dest = FileNameJoin[{$libResourcesDir, $libName}];
  RunProcess[{"cp", $builtLib, $dest}];
  If[FileExistsQ[$dest],
    Print["Library copied to: ", $dest],
    Print["Error: Failed to copy library to: ", $dest];
    Exit[1]
  ];
,
  Print["Error: Built library not found at: ", $builtLib];
  Exit[1];
];
