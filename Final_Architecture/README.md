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

The current default robot is `robot_ver2`. It is an imported, preliminary
six-axis model. `robot_ver1` remains available explicitly:

```matlab
oldRobot = loadRobot("robot_ver1");
```

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
limits and zero home positions to the six joints without changing the CAD
export. The temporary TCP is the `End_Effector` origin.

`Joint_6` and the `End_Effector` mesh come from the preserved
`URDF_Files(ver2)` export. Its older first-five-joint geometry and inertial
data are not used. `robot.readiness` records that limits and TCP are
provisional, Inventor properties used Low accuracy, and final actuator sizing
is blocked. FK, numerical IK, visualization, workspace, trajectory generation,
collision inspection, and preliminary rigid-body dynamics remain available. The current
`Link_3`/`Wrist_2` home collision is intentionally reported as unresolved;
MATLAB treats imported mesh collision geometry as convex.

The active six-axis URDF export is available through
`loadRobot()` or `loadRobot("robot_ver2")`. Its copied CAD package
and robot-specific readiness notes are in `robots/robot_ver2/README.md`.
Its source URDF limits and mass data need review. The copied mesh filenames
are unique to avoid clashes with `robot_ver1`. Run
`validate_robot_ver2` before using it for preliminary analysis.

The earlier programmatic `initial_trial` model remains available explicitly:

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
`params`, `structure`, `chain`, and collision definitions. It does not read
robot parameters itself, which allows parameter sweeps.
The fixed `base_structure` owns the O0-to-J1 segment; each `link_i` owns
its outgoing Ji-to-J(i+1) segment, not the segment ending at Ji. The TCP
body owns the J6-to-TCP structure and tool. The J6 frame has zero mass.
`collision/collisionGeometry.m` is the single source for the preliminary
collision shapes and their body-relative transforms. `buildModel.m` attaches
those exact shapes and stops if an attachment fails.

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
It validates returned candidates with FK and joint limits, but does not
perform collision rejection or guarantee that every possible IK branch was
found.

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

Every workspace figure overlays the selected robot's visual geometry. Use
the joint sliders on the right to pose it with the mouse within its mechanical
limits. Angle and TCP-position readouts update continuously; `Home` restores
the configured home position, and `Frames` toggles body frames. Rotating or
zooming the 3-D view keeps the robot and workspace point cloud synchronized.

`collision_free_workspace` evaluates self-collision at every sampled
configuration. Its graph retains every reachable TCP point: points reached by
colliding configurations are light gray, while collision-free configurations
are blue. Rejected configurations remain stored under `workspace.collision`
instead of being discarded.

The shared `workspaceAnalysis` engine retains every sampled configuration,
its TCP transform and point, its voxel assignment, and acceptance masks.
Collision checks mark and retain rejected configurations rather than deleting
their records. The public plots show one 3-D workspace figure;
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
Both manipulability and usable workspaces first reject self-colliding
configurations. The usable result therefore combines mechanical joint limits,
self-collision rejection, and the manipulability threshold.

Expensive analysis data is reused automatically. The generic
`analysis_cache/prepareWorkspaceDataset.m` caches sampled joint
configurations, FK transforms, TCP points, collision/clearance results, and
manipulability values in stages. Its signatures include kinematics, limits,
TCP, sampling, collision geometry, skipped pairs, clearance margin, and the
implemented environment status. Relevant physical changes invalidate the
correct stage; plotting, voxel-size, and manipulability-threshold changes
reuse the expensive calculations. Cache settings live near the top of
`workspaceAnalysis.m`:

```matlab
settings.cacheEnabled = true;
settings.cacheFolder = fullfile(rootDir,'analysis_cache','data');
```

Generated cache and figure-output folders are ignored by Git. Cache behavior
is reported in `workspace.cache` and validated by
`validate_workspace_analysis`.

## Collision Analysis

`collision_analysis/analyzeCollisions.m` is the generic calculation layer.
For each supplied configuration it records self-collision, closest clearance,
the relevant body pair, and available witness points. Directly connected
bodies are skipped, along with robot-specific intentional-contact pairs from
`robot.collision.ignoredSelfCollisionPairs`. A geometric collision and a
clearance-margin failure are kept as separate results.

Run the interactive collision-geometry inspector with no arguments:

```matlab
inspectCollisions
```

It samples the robot selected in `projectConfig.m` and lets you step through
colliding, near-collision, and clear configurations. Collision shapes for
`initial_trial` are still preliminary cylinders and spheres, so these results
validate the software pipeline, not final mechanical clearance. Environment,
workpiece, and trajectory collision checking are later layers.

## Dynamics Analysis

`dynamics_analysis/` contains generic ideal rigid-body joint-side dynamics.
For `initial_trial`, the fixed tool (2.0 kg) and J6-to-EE structure (0.71 kg) are separate
component masses combined once in the TCP body. Link and joint-module masses
are defined in `robotParameters.m`; `buildModel.m` estimates COM and inertia
from the simplified geometry unless explicit COM/inertia are supplied.
For `robot_ver1`, body masses, COMs, and inertias come only from its URDF.
All current body properties remain preliminary. Confirm them against CAD,
datasheets, and measured properties before actuator selection.

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
`results.meta.readiness`. The consolidated load study can run for design
exploration, but final motor sizing remains blocked until limits, TCP,
mass-property accuracy, case data, and collision status are resolved.

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
`session = actuator_sizing_analysis` for the configured robot's stationary
and moving load cases and one summary dashboard. See
[actuator_sizing_cases/README.md](actuator_sizing_cases/README.md) for case
definitions, limitations, outputs, and validation commands. Use
`validate_dynamics_analysis(robot)` for numerical dynamics checks and
`validate_actuator_sizing_cases(robot)` for trajectory and sampling checks.
The dashboard's `Animate case` and `Gravity poses` controls open the
synchronized motion and stationary-pose viewers. See
[visualization/README.md](visualization/README.md) for commands and controls.

## Visual Joint Check

From MATLAB with `Final_Architecture` and `Final_Architecture/visualization`
on the path:

```matlab
animate_robot_joints;
robot = loadRobot("robot_ver1"); % Optional: inspect a non-default robot.
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
