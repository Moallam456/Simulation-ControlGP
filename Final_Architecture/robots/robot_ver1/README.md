# robot_ver1

`robot_ver1` is the URDF-backed, preliminary six-axis CAD model.

## Ownership

- `CAD/robot_ver1/urdf/robot_ver1.urdf` owns topology, fixed transforms,
  visual meshes, collision meshes, masses, COMs, and inertias.
- `robotDefinition.m` imports the URDF and derives the common robot metadata.
- `solveInverseKinematics.m` provides numerical multistart IK through the
  common `robot.solveIK(...)` handle.
- `dynamicsTestCases.m` owns robot-specific preliminary motion cases, including
  the 300 mm outer-reach horizontal line.
- `collision/collisionGeometry.m` describes mesh provenance and records the
  unresolved `Link_3`/`Wrist_2` home collision.

Joint 6's parent-to-child transform, axis, and the `End_Effector.STL` mesh
were copied from the preserved `URDF_Files(ver2)` export. The older mass
properties and the first five joints in that export were not copied.

The active masses are user-supplied preliminary values: `Base_Link` 4.0 kg,
`Link_1` 2.2 kg, `Link_2` 3.5 kg, `Link_3` 2.35 kg, `Wrist_1` and `Wrist_2`
0.575 kg each, and `End_Effector` 0.300 kg. Existing body-frame COMs are
retained. Each earlier Low-accuracy Inventor COM inertia tensor is scaled by
the ratio of new mass to old mass, which assumes unchanged mass distribution.

The renamed `robot_ver1_original_export.csv` is retained only as CAD-export
history. It is not read by MATLAB and does not own any active parameters.

## Current Limitations

- The model has six revolute joints, including the added `Joint_6` and
  `End_Effector` body.
- Joint limits are provisional `[-pi,pi]` and home angles are zero.
- The `End_Effector` origin is the temporary TCP.
- COMs remain from the Inventor export, and inertias are provisional
  mass-scaled estimates rather than newly calculated CAD properties.
- The `Link_3`/`Wrist_2` home collision remains unresolved.
- Final actuator sizing is blocked.

Run:

```matlab
addpath(genpath(fullfile(pwd,'Final_Architecture')))
robot = loadRobot("robot_ver1");
report = validate_robot_ver1(robot);
```
