VerificationTest[
  PacletDirectoryLoad[FileNameJoin[{DirectoryName[$TestFileName, 2], "Rapier"}]],
  {__String},
  SameTest -> MatchQ,
  TestID -> "PacletDirectoryLoad"
]

VerificationTest[
  Needs["Rapier`"],
  Null,
  TestID -> "NeedsRapier"
]

VerificationTest[
  StringQ[Rapier`RapierVersion[]],
  True,
  TestID -> "RapierVersionCall"
]
