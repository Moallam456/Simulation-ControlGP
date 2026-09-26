# robot_ver1

`robot_ver1` is the URDF-backed, intermediate five-axis CAD model.

## Ownership

- `CAD/robot_ver1/urdf/robot_ver1.urdf` owns topology, fixed transforms,
  visual meshes, collision meshes, masses, COMs, and inertias.
- `robotDefinition.m` imports the URDF and derives the common robot metadata.
- `solveInverseKinematics.m` provides numerical multistart IK through the
  common `robot.solveIK(...)` handle.
- `collision/collisionGeometry.m` describes mesh provenance and records the
  unresolved `Link_3`/`Wrist_2` home collision.

The renamed `robot_ver1_original_export.csv` is retained only as CAD-export
history. It is not read by MATLAB and does not own any active parameters.

## Current Limitations

- Joint 6 is absent, so the model has five DOF.
- Joint limits are provisional `[-pi,pi]` and home angles are zero.
- `Wrist_2` origin is the temporary TCP.
- Inventor physical properties were generated at Low accuracy.
- Final actuator sizing is blocked.

Run:

```matlab
addpath(genpath(fullfile(pwd,'Final_Architecture')))
robot = loadRobot("robot_ver1");
report = validate_robot_ver1(robot);
```
