# Final Architecture

This folder is organized so the active robot is selected once and generic
analysis code receives a common robot interface.

## Main Usage

```matlab
addpath(genpath(fullfile(pwd,'Final_Architecture')))

robot = loadRobot();

show(robot.model, robot.params.joints.homePosition, 'Frames', 'on')
plot_robot(robot, robot.params.joints.homePosition)

report = validate_kinematics(robot);
```

To change the default robot, edit only:

```matlab
projectConfig.m
```

For comparison studies, request a robot explicitly:

```matlab
robotA = loadRobot("initial_trial");
```

## Common Robot Interface

`loadRobot` returns:

```matlab
robot.id
robot.params
robot.structure
robot.chain
robot.model
robot.solveIK
robot.actuation
robot.collision
robot.readiness
```

Generic analyses should accept this `robot` struct explicitly.

## Robot-Specific Folder

The current default robot is `initial_trial`. The imported `robot_ver1`
model remains available explicitly for testing the URDF-backed workflow.

```text
robots/robot_ver1/
  robotDefinition.m
  solveInverseKinematics.m
  collision/collisionGeometry.m
  CAD/robot_ver1/
    urdf/robot_ver1.urdf
    meshes/*.STL
```

The URDF is the sole source for `robot_ver1` transforms, mesh geometry,
masses, COM locations, and inertias. `robotDefinition.m` imports it with
row-vector configurations and derives the common `params`, `structure`, and
`chain` metadata from the imported tree. MATLAB applies temporary `[-pi,pi]`
limits and zero home positions to the five joints without changing the CAD
export. The temporary TCP is the `Wrist_2` origin.

`robot.readiness` records that Joint 6 is missing, limits and TCP are
provisional, Inventor properties used Low accuracy, and final actuator sizing
is blocked. Preliminary FK, numerical IK, visualization, workspace,
collision inspection, and rigid-body dynamics remain available. The current
`Link_3`/`Wrist_2` home collision is intentionally reported as unresolved;
MATLAB treats imported mesh collision geometry as convex.

The earlier programmatic model remains available explicitly:

```text
robots/initial_trial/
  robotParameters.m
  robotStructure.m
  kinematicChain.m
  buildModel.m
  solveInverseKinematics.m
  gravityCandidatePoses.m
  actuation/actuatorParameters.m
  collision/collisionGeometry.m
  CAD/
```

`robotParameters.m` owns numerical physical data such as `l1...l10`,
joint limits, home position, and tool transform.

`robotStructure.m` owns topology, names, joint order, joint axes, and frame
identities.

`kinematicChain.m` derives the joint home frames, fixed parent-to-joint
transforms, and visual segments from the dimensions. Joint rotations are
applied separately about the axes in `robotStructure.m`. The active model
does not use a DH or modified-DH table.

`buildModel.m` constructs the MATLAB `rigidBodyTree` from supplied
`params`, `structure`, and `chain`. It does not read robot parameters itself,
which allows parameter sweeps.
The fixed `base_structure` owns the O0-to-J1 segment; each `link_i` owns
its outgoing Ji-to-J(i+1) segment, not the segment ending at Ji. The TCP
body owns the J6-to-TCP structure and tool. The J6 frame has zero mass.

`gravityCandidatePoses.m` defines robot-specific stationary screening poses.
`loadRobot` exposes these as `robot.gravityPoses`; generic gravity analyses
include them by default, alongside requested and random configurations.

## Inverse Kinematics

`robot.solveIK(targetPose,struct('qSeed',qStart))` returns the distinct,
joint-limit-valid configurations it finds for an exact TCP pose in
`solutions.q` (N-by-DOF, radians). Rows are ordered by joint-space distance
from `qSeed`, so `solutions.q(1,:)` remains the nearest returned candidate.
Check `solutions.valid` before using any row. `solutions.info` reports the
method, per-candidate FK errors, and whether analytical generation was
available. IK does not reject candidates based on the preliminary collision
shapes; its collision status is `UNKNOWN`.

For `initial_trial`, MATLAB's analytical IK generator supplies candidate
branches. The generated solver is cached outside this repository for each
distinct kinematic model and is regenerated when geometry or tool placement
changes. If analytical generation is unsupported, deterministic multistart
MATLAB numerical IK is used; numerical multistart does not guarantee every
possible branch. Cartesian trajectories evaluate the candidate sets across
the entire path and choose a continuous joint sequence.
For diagnostics, pass `struct('method',"numerical")` to test that fallback.

For `robot_ver1`, `robot.solveIK` uses deterministic numerical multistart IK.
It validates returned candidates with FK and joint limits, but it cannot
invent the missing sixth degree of freedom and does not perform collision
rejection.

## Parameter Sweep Pattern

```matlab
baseRobot = loadRobot("initial_trial");
params = baseRobot.params;
params.geometry.l2 = params.geometry.l2 + 0.05;

robot = loadRobot("initial_trial", params);
workspace = workspaceAnalysis(robot,"reachable",struct('showPlot',false));
```

Generic code outside `robots/` should consume the loaded `robot` interface
and should not call `robots/initial_trial` files directly.

## Workspace Analysis

The public workspace commands take no arguments. They load whichever robot
is selected by `projectConfig().robotID`. `projectConfig.m` owns only project
selection; it does not own analysis settings.

```matlab
reachable_workspace
collision_free_workspace
manipulability_workspace
usable_workspace
```

`collision_free_workspace` is currently an explicit pass-through stage. It
stores collision fields as `NOT_EVALUATED`/`NaN`, rejects no configurations,
and labels its figure accordingly. It must not be interpreted as verified
collision-free until the collision-analysis layer is connected.

The shared `workspaceAnalysis` engine retains every sampled configuration,
its TCP transform and point, its voxel assignment, and acceptance masks.
Future collision checks will mark and retain rejected configurations rather
than deleting their records. The public plots show one 3-D workspace figure;
common sampling, voxel, plotting, saving, and cache defaults are kept near the
top of `workspaceAnalysis.m`. The chosen manipulability index and motion type
are kept near the top of `manipulability_workspace.m`.

`manipulability_workspace` evaluates the linear TCP Jacobian at every sampled
configuration. Its inverse-condition score is
`sigma_min(Jv)/sigma_max(Jv)`: zero is singular or nearly singular, while a
value nearer one means the TCP can generate linear motion more evenly in all
directions. The plot uses one point per occupied Cartesian voxel and colors it
by the best sampled score available in that voxel. Raw per-configuration
scores remain in `workspace.manipulability.values`.

`usable_workspace` keeps only sampled configurations whose inverse-condition
score is at least the clearly labeled `manipulabilityThreshold` near the top
of that file. It retains all
original samples and records the accepted subset under `workspace.accepted`.
Because collision analysis is not connected yet, both manipulability and
usable plots clearly report that collision filtering was not evaluated. The
usable workspace therefore becomes final only after the collision layer is
implemented.

Expensive workspace data is reused automatically. `prepareWorkspaceDataset`
caches sampled joint configurations, FK transforms, and TCP points. Jacobian
and manipulability values are added lazily only when a manipulability-based
analysis first requests them. The cache signature includes the robot
kinematics, joint limits, TCP, sampling method, seed, and sample count, so a
relevant model or sampling change rebuilds the data automatically. Plotting,
voxel size, and manipulability-threshold changes reuse the same expensive
calculations. The cache settings live with the other common workspace defaults
near the top of `workspaceAnalysis.m`:

```matlab
settings.cacheEnabled = true;
settings.cacheFolder = fullfile(rootDir,'workspace_analysis','cache');
```

Generated cache and figure-output folders are ignored by Git. Cache behavior
is reported in `workspace.cache` and validated by
`validate_workspace_analysis`.

## Dynamics Analysis

`dynamics_analysis/` contains generic ideal rigid-body joint-side dynamics.
The fixed tool (2.0 kg) and J6-to-EE structure (0.71 kg) are separate
component masses combined once in the TCP body. Link and joint-module masses
are defined in `robotParameters.m`; `buildModel.m` estimates COM and inertia
from the simplified geometry unless explicit COM/inertia are supplied.
All current body properties are preliminary estimates. Confirm them against
CAD, datasheets, and measured properties before actuator selection.

```matlab
robot = loadRobot();
validation = validateDynamicModel(robot);
options = struct('gravity',[0 0 -9.81], 'numSamples',1000, 'seed',1);
gravityResults = analyzeGravityLoading(robot,options);

scenario.gravity = [0 0 -9.81]; % Base +Z is upward.
trajectory.time = [0; 1];       % N-by-1, strictly increasing seconds.
trajectory.q = zeros(2,robot.structure.dof);   % radians
trajectory.qd = zeros(2,robot.structure.dof);  % rad/s
trajectory.qdd = zeros(2,robot.structure.dof); % rad/s^2
results = analyzeTrajectoryDynamics(robot,trajectory,scenario);
disp(results.summaryTable)
figures = plotDynamicsResults(results,struct('joint',2));
```

For `robot_ver1`, these results are explicitly preliminary and include
`results.meta.readiness`. Calls to `createActuatorSizingCases` and
`runActuatorSizingCases` are blocked until Joint 6, final limits, TCP, mass
property accuracy, and collision status are resolved.

`results` contains `meta`, `time`, `state.q/qd/qdd`,
`torque.total/gravity/inertial/velocity/residual`, `power.joint`,
`torqueSpeed.speed/torque`, `summaryTable`, and `peakEvents`.
Torque and speed arrays are N-by-DOF. Joint order comes from
`robot.structure.jointNames`. Gravity is applied to a copy of the model;
the shared `robot.model` is not changed. `analyzeGravityLoading(robot,options)`
searches separate positive and negative gravity peaks for each joint using
stationary candidates and multiple local starts; `options.mode="poses"` only
evaluates specified stationary poses. The search requires Optimization Toolbox
and provides no global certificate. `example_dynamics_workflow()` is only
a J2-motion software smoke test, not an actuator-sizing case. Run
`example_actuator_sizing_cases()` for preliminary line, arc, full-circle,
multi-segment, and gravity load cases. See
[actuator_sizing_cases/README.md](actuator_sizing_cases/README.md) for case
definitions, limitations, outputs, and validation commands. Use
`validate_dynamics_analysis(robot)` for numerical dynamics checks and
`validate_actuator_sizing_cases(robot)` for trajectory and sampling checks.
For synchronized robot animation and progressively revealed torque graphs,
use `session = inspectDynamicsCase(robot,"LINE_WELD")` or select another
enabled case ID. `inspectDynamicsCase(robot,"GRAVITY_HOLD")` displays
stationary per-joint peak poses. See
[visualization/README.md](visualization/README.md) for commands and controls.

## Visual Joint Check

From MATLAB with `Final_Architecture` and `Final_Architecture/visualization`
on the path:

```matlab
robot = loadRobot();
animate_robot_joints(robot);
animate_robot_joints(robot,struct('jointIndices',[2 3], ...
    'amplitudeDeg',60,'framesPerSweep',30,'pauseSeconds',0.05));
```

The animation moves one joint at a time around the reference pose, stays
within joint limits, and returns to the reference pose before moving the
next joint. `qReference` (radians), `jointIndices`, `amplitudeDeg`,
`framesPerSweep`, and `pauseSeconds` are optional. To inspect a static
pose, use `plot_robot(robot,q)`. `validate_robot_visuals(robot)` checks
link endpoint attachment and COM ownership numerically at non-home poses.
