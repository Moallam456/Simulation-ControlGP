# robot_ver2

Six-revolute-joint CAD export integrated as a MATLAB `rigidBodyTree`. The URDF
and seven STL meshes are copied from
`URDF_Assembly_Robot_ver2_Fixed/URDF_Assembly_Robot_ver2_Fixed/URDF_Assembly_Robot_ver2/`.
That source folder is unchanged. The copied STL files are prefixed with
`robot_ver2_` and the copied URDF references were updated accordingly. This
prevents MATLAB from resolving same-named meshes from `robot_ver1` on its path.
The copied URDF is authoritative for the
joint topology, transforms, mesh paths, masses, COMs, and inertias; the robot
wrapper does not redefine those values.

```matlab
addpath(genpath(fullfile(pwd,'Final_Architecture')))
robot = loadRobot("robot_ver2");
show(robot.model,robot.params.joints.homePosition, ...
    'Visuals','on','Collisions','off','Frames','off');
report = validate_robot_ver2(robot);
```

`projectConfig.m` now selects `robot_ver2` by default. No-argument commands
such as `animate_robot_joints` use it automatically.
The new robot uses the same `robot.solveIK(targetPose,options)` and
`forward_kinematics(robot,q)` interface as the other models. Its IK is
numerical multistart; it does not promise every possible solution.

## Readiness

- The source URDF declares `0,0` limits for every joint. The MATLAB model
  temporarily uses `[-pi,pi]` for each joint. Replace these with measured
  mechanical limits before interpreting workspace or collision results.
- The temporary TCP is the `End_Effector` body origin. No welding-tool offset
  is included.
- The CSV beside the original URDF is an older CAD-export table. Its mass/COM
  values differ because the URDF was edited afterward; the CSV is not used by
  this model. The copied URDF now expresses the moving-link Inventor inertials
  in their link frames. Inventor's requested mass-property accuracy was Low;
  independently verify the frame alignment and CAD properties before final
  motor sizing. MATLAB's `rigidBodyTree` does not expose base-link mass
  properties through `model.Base`.
- With the correct meshes loaded, the zero-angle pose has no detected
  self-collision. Ten of 20 random configurations were collision-free in a
  smoke test. No pair is suppressed; CAD interference checks are still needed
  before treating clearance results as design limits.
- `robot.readiness.actuatorSizingAllowed` is false. FK, IK, visualization,
  workspace, collision inspection, and preliminary dynamics are available;
  final motor sizing is not validated for this export.

The copied STL files are attached as both visual and collision geometry.
MATLAB treats the imported collision meshes as convex geometry, so collision
results may differ from detailed CAD interference checks.

## Inertial Frame Conversion

The source Inventor COMs and COM inertias were reported in an assembly frame
with Y up. The robot's zero-pose base frame has Z up. The copied URDF uses
`R_base,CAD = [1 0 0; 0 0 -1; 0 1 0]`. For each moving body at zero joint
angles, its URDF inertial origin is `R_body,base * (R_base,CAD * c_CAD -
p_body,base)`, and its COM inertia tensor is rotated into that body frame.
Masses, joints, visual meshes, and collision meshes were not changed. The
original uploaded URDF package was not edited. Validation checks that each
imported COM lies within 1 mm of its own STL's bounding box and that the
inertias remain physically valid; this is not a substitute for a CAD review.

## Preliminary Load Study

From the project root, run:

```matlab
addpath(genpath(fullfile(pwd,'Final_Architecture')))
session = actuator_sizing_analysis;
```

The dashboard runs a stationary gravity search and `THREE_LEG_SWEEP`. The
moving case starts from an outer-reach, collision-free configuration with
J3 at 105 degrees (15 degrees lower than the previous trial) and the torch
pointing down. It moves the TCP horizontally 0.30 m, down 0.35 m, and
horizontally 0.50 m back past its starting side. J4 tilts the torch by 8
degrees on the first leg and returns it on the last leg; the resulting
joint motion is about 25 degrees across the path. Each turn includes a
smooth stop. In the dashboard, select `THREE_LEG_SWEEP` and press
`Animate case` to see the synchronized robot and torque plots. Playback opens
at 0.25x speed so the motion is easier to follow. Press `Play` to replay it
after it finishes, or change the speed in the viewer.

The 0.05 s trajectory passed IK, FK, joint-step, and time-step convergence
checks. Its 189 sampled configurations had no mesh collisions and at least
52 mm reported clearance. Recheck convergence after changing the motion.
These checks do not verify continuous swept-volume
clearance or the physical accuracy of CAD collision meshes. The results are
preliminary joint-side loads, not final motor ratings.

The dashboard also includes `JOINT_SWEEP_180`. Select it and press
`Animate case` to watch J2 move continuously from -90 to +90 degrees while
all five other joints keep moving as J2 crosses zero. It is a 12-second,
joint-space demonstration with about 51 mm minimum clearance at its 241
sampled configurations. The full provisional +/-180-degree J2 range is not
used because Link_2 collides with the base near those ends. This case is
marked `VISUAL_ONLY`: its synchronized torque plots are illustrative and are
not added to the actuator-sizing peak or RMS summary.
