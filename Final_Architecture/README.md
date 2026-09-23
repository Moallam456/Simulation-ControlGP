# Final Architecture

This folder is organized so the active robot is selected once and generic
analysis code receives a common robot interface.

## Main Usage

```matlab
addpath(genpath(fullfile(pwd,'Final_Architecture')))

robot = loadRobot();

show(robot.model, robot.params.joints.homePosition, 'Frames', 'on')
plot_robot(robot, robot.params.joints.homePosition)

results = validate_kinematics(robot);
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
```

Generic analyses should accept this `robot` struct explicitly.

## Robot-Specific Folder

The current robot is `initial_trial`.

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

## Parameter Sweep Pattern

```matlab
baseRobot = loadRobot("initial_trial");
params = baseRobot.params;
params.geometry.l2 = params.geometry.l2 + 0.05;

robot = loadRobot("initial_trial", params);
workspace = reachable_workspace(robot, 10000);
```

Generic code outside `robots/` should consume the loaded `robot` interface
and should not call `robots/initial_trial` files directly.

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
