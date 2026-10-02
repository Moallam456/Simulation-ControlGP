# robot_ver1

`robot_ver1` is the URDF-backed, preliminary six-axis CAD model.

## Ownership

- `CAD/robot_ver1/urdf/robot_ver1.urdf` owns topology, fixed transforms,
  visual meshes, collision meshes, masses, COMs, and inertias.
- `robotDefinition.m` imports the URDF and derives the common robot metadata.
- `solveInverseKinematics.m` provides numerical multistart IK through the
  common `robot.solveIK(...)` handle.
- `collision/collisionGeometry.m` describes mesh provenance and records the
  unresolved `Link_3`/`Wrist_2` home collision.

Joint 6's parent-to-child transform, axis, and the `End_Effector.STL` mesh
were copied from the preserved `URDF_Files(ver2)` export. The older mass
properties and the first five joints in that export were not copied. The
end-effector mass, COM, and inertia were instead converted from the supplied
Inventor screenshots into the end-effector body frame and SI units.

The renamed `robot_ver1_original_export.csv` is retained only as CAD-export
history. It is not read by MATLAB and does not own any active parameters.

## Current Limitations

- The model has six revolute joints, including the added `Joint_6` and
  `End_Effector` body.
- Joint limits are provisional `[-pi,pi]` and home angles are zero.
- The `End_Effector` origin is the temporary TCP.
- Inventor physical properties were generated at Low accuracy.
- The `Link_3`/`Wrist_2` home collision remains unresolved.
- Final actuator sizing is blocked.

Run:

```matlab
addpath(genpath(fullfile(pwd,'Final_Architecture')))
robot = loadRobot("robot_ver1");
report = validate_robot_ver1(robot);
```
