# robot_ver2

Six-revolute-joint CAD export integrated as a MATLAB `rigidBodyTree`. The URDF
and seven STL meshes are copied from
`URDF_Assembly_Robot_ver2_Fixed/URDF_Assembly_Robot_ver2_Fixed/URDF_Assembly_Robot_ver2/`.
The source and copied STL files are prefixed with `robot_ver2_`, and their
URDF references match. This
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
The robot uses the same `robot.solveIK(targetPose,options)` and
`forward_kinematics(robot,q)` interface as the other models. Its IK uses the
control team's closed-form solver, with the equations and nominal geometry
contained as local functions in this robot's `solveInverseKinematics.m`.
No files outside `Final_Architecture` are needed by this robot. The original
`RecentIK` source is preserved but is no longer a runtime dependency.
Every branch returned through `robot.solveIK`
is checked against the imported URDF FK and the active joint limits. There is
no numerical IK fallback. Numerical multistart options from older callers are
accepted but do not affect this solver. Branches with effectively zero
`cos(q5)` are skipped; a different branch may still solve the pose. An empty
result is not proof that no physical solution exists.
The common loader binds this solver once, so repeated calls do not change the
MATLAB path or depend on which other robot was most recently loaded.

Run `report = validate_robot_ver2_ik` to cross-check hundreds of reachable
poses and all returned branches against the imported URDF FK, compare with
MATLAB's numerical IK, measure runtime, and test the sweep trajectory and
other robots' solver interfaces. This no-argument validator lives in
`Final_Architecture/validation`. Rerun it after revising joint frames or link
dimensions; the analytical geometry is specific to this robot revision.
The URDF rounds the Joint_6 frame angle to `1.5708` radians, so the nominal
closed-form solution has an approximately 5.2-microradian orientation
residual. This test checks the solver against URDF FK; it does not check
hardware encoder calibration or collisions.

Latest validation (2026-10-15): 627 FK-generated targets, including joint-limit
edges and near-singular wrist poses, produced 4,018 accepted TCP configurations
with no missing targets or invalid branches in that test set. Maximum errors
were `1.385e-6 m` and `5.195e-6 rad`. Both the analytical solver and MATLAB's
numerical IK solved all 40 comparison targets; their joint angles need not
match because different configurations can reach the same full pose.
`THREE_LEG_SWEEP` passed with 189 samples and finite inverse-dynamics torques.
The no-argument validator also checks output fields, branch ordering,
deduplication, configured limits, unreachable/invalid inputs, and solver
dispatch after loading `initial_trial` and `robot_ver1`.

Measured median public analytical-call time was about 3.5 ms versus 6.1 ms
for a single MATLAB numerical attempt. On the eight matched multistart timing
targets, the analytical solver was about 55 times faster than a 27-start
numerical workload. These are local benchmark results, not guarantees for
every pose or machine. Runtime FK validation remains enabled on every call.

## Model Inputs

- The source and copied URDFs declare provisional `[-pi,pi]` limits for every
  joint. Their 1000 N m effort and pi rad/s velocity limits are software
  placeholders, not actuator ratings. The physical joint stops remain to be
  entered.
- Workspace plots, FK, IK targets, Jacobians, and Cartesian trajectories use
  the fixed `tcp` frame described below, not the Joint 6 origin.
- The CSV beside the original URDF is an older CAD-export table. Its mass/COM
  values differ because the URDF was edited afterward; the CSV is not used by
  this model. The copied URDF now expresses the moving-link Inventor inertials
  in their link frames. Inventor's requested mass-property accuracy was Low.
  MATLAB's `rigidBodyTree` does not expose base-link mass properties through
  `model.Base`.
- With the correct meshes loaded, the zero-angle pose has no detected
  self-collision. The collision checker does not suppress any body pair beyond
  the connected pairs handled by the generic analysis.
- `robot.readiness.actuatorSizingAllowed` is false. FK, analytical IK,
  visualization, workspace, collision inspection, static holding torque, and
  the moving load-study dashboard are available. The dashboard does not label
  these cases as final motor sizing.

The copied STL files are attached as both visual and collision geometry.
MATLAB checks the imported collision meshes as convex geometry.

## TCP Definition

The authoritative tool transform is the `tcp_fixed` joint origin in
`CAD/URDF_Assembly_Robot_ver2/urdf/URDF_Assembly_Robot_ver2.urdf`:

```xml
<origin xyz="-0.02287 0.02053 0.24577" rpy="0 0 0" />
```

This is the user-supplied 2026-10-15 measurement relative to Joint 6's
`End_Effector` frame: -22.87 mm X, +20.53 mm Y, +245.77 mm Z. TCP axes are
parallel to that frame. `End_Effector` keeps the existing torch STL and
1.134 kg inertial properties; the new `tcp` body has zero mass/inertia and no
geometry, so it does not duplicate the tool or change joint-side dynamics.
There are still six moving joints, plus one fixed TCP joint.

`robotDefinition.m` reads this transform into `params.tool.TFlangeTCP`.
`structure.frames.flange` is `End_Effector`; `structure.frames.endEffector`
and `structure.frames.tcp` are `tcp`. FK returns `T_Base_TCP`, and IK accepts
that same pose and removes the fixed offset exactly once. The fixed body
makes MATLAB's geometric Jacobian refer to the tip directly. All workspace
graphs and their inspector use it automatically with their existing commands.
Old caches rebuild when the offset, its rotation, or tool/model version changes.

`validate_robot_ver2_tcp` tests the reported measurement independently of the
URDF values, offset FK/IK, a finite-difference Jacobian, cache invalidation,
mass/collision invariance, and a short Cartesian trajectory. It also supports
an optional independent base-frame CAD pose. Matching the supplied dimensions
is not a physical encoder/tool calibration of the assembled machine.

The standalone `URDF_Assembly_Robot_ver2_Fixed` distribution has a matching
`tcp_fixed` joint. It is a distribution copy, not a MATLAB runtime dependency.
Keep it synchronized if the authoritative URDF is revised.

For the controller, `params.tool.control.T_flange_tcp` exposes the same
transform as 16 row-major values in metres, matching the control team's
`T_flange_tcp[16]` convention. Its parent is `End_Effector`, not the base.
The controller must use the same Joint 6 frame convention and apply this
transform once. The separate controller's DH model and firmware deployment
are not modified or certified by the MATLAB tests.

## Inertial Frame Conversion

The source Inventor COMs and COM inertias were reported in an assembly frame
with Y up. The robot's zero-pose base frame has Z up. The copied URDF uses
`R_base,CAD = [1 0 0; 0 0 -1; 0 1 0]`. For each moving body at zero joint
angles, its URDF inertial origin is `R_body,base * (R_base,CAD * c_CAD -
p_body,base)`, and its COM inertia tensor is rotated into that body frame.
Masses and joint frames were not changed. The source URDF now has the same
frame-corrected inertials, provisional limits, and uniquely named meshes as
the project copy. Validation checks that each imported COM lies within 1 mm
of its own STL's bounding box and that the inertia tensors are physically
valid.

## Simscape Multibody

Run `open_robot_ver2_multibody` without arguments, then **Ctrl+D** and **Run**
in Simulink. `multibody/robot_ver2_multibody.slx` contains the torque-driven
physical plant, measured joint/TCP outputs and a removable PD demonstration.
The model uses this robot's corrected URDF, not a separate set of masses or
frames. See [Opening and Control Interface](multibody/README.md) for the
step-by-step guide. `validate_robot_ver2_multibody` compares the physical plant
with existing FK and dynamics. Motors, gearboxes and collision/contact forces
are not silently added by the URDF import.

## Load Study

`park_position_analysis` uses this robot's folded-back and upright/folded
seed angles in `robotDefinition.m` (`params.park`). Its default search limits
are 1.35 m height and 0.65 m radius, not new mechanical specifications.
It ranks peak holding torque in N m until joint-output continuous motor ratings
are supplied. Candidate poses must be above the mounting plane and pass
self-clearance checks. Only passing sampled return routes can be previewed;
this does not establish power-off/brake or workcell safety. The configured
zero/home pose is unchanged.

From the project root, run:

```matlab
addpath(genpath(fullfile(pwd,'Final_Architecture')))
session = actuator_sizing_analysis;
```

The dashboard runs a stationary gravity search and `THREE_LEG_SWEEP`. The
moving case starts from an outer-reach, collision-free configuration with
J3 and J4 at 120 degrees and the torch
pointing down. It moves the TCP horizontally 0.30 m, down 0.35 m, and
horizontally 0.50 m back past its starting side. J4 tilts the torch by 4
degrees on the first leg and returns it on the last leg. These settings
were retuned for the measured torch-tip TCP; the old wrist-origin preset
is not feasible when applied at the tip. Each turn includes a
smooth stop. In the dashboard, select `THREE_LEG_SWEEP` and press
`Animate case` to see the synchronized robot and torque plots. Playback opens
at 0.25x speed so the motion is easier to follow. Press `Play` to replay it
after it finishes, or change the speed in the viewer.

The 0.05 s trajectory passed IK, FK, and joint-step checks. Its 189 sampled
configurations had no mesh collisions and at least 56 mm reported clearance.
Time-step convergence must be rerun for this updated preset.
Those checks cover sampled configurations, not
continuous swept-volume clearance. The dashboard reports ideal rigid-body
joint-side loads, not motor ratings.

The dashboard also includes `JOINT_SWEEP_180`. Select it and press
`Animate case` to watch J2 move continuously from -90 to +90 degrees while
all five other joints keep moving as J2 crosses zero. It is a 12-second,
joint-space demonstration with about 51 mm minimum clearance at its 241
sampled configurations. The full provisional +/-180-degree J2 range is not
used because Link_2 collides with the base near those ends. This case is
marked `VISUAL_ONLY`: its synchronized torque plots are illustrative and are
not added to the actuator-sizing peak or RMS summary.
