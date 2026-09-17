# Final Architecture

This folder is organized so the active robot is selected once and generic
analysis code receives a common robot interface.

## Main Usage

```matlab
addpath('Final_Architecture', ...
        'Final_Architecture\kinematics', ...
        'Final_Architecture\visualization', ...
        'Final_Architecture\validation', ...
        'Final_Architecture\workspace_analysis', ...
        'Final_Architecture\dynamics_analysis')

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
robot.dh
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
  dhParameters.m
  dhTransform.m
  buildModel.m
  inverseKinematics.m
  actuation/actuatorParameters.m
  collision/collisionGeometry.m
  CAD/
```

`robotParameters.m` owns numerical physical data such as `l1...l10`,
joint limits, home position, and tool transform.

`robotStructure.m` owns topology, names, joint order, joint axes, and frame
identities.

`dhParameters.m` derives the kinematic representation from parameters. For
`initial_trial`, the authoritative chain is a fixed-transform chain because
the hand sketch has compound CAD-style offsets. A reference modified-DH
table is retained for documentation and future IK work.

`dhTransform.m` is robot-specific and stays beside the robot's DH/IK files.

`buildModel.m` constructs the MATLAB `rigidBodyTree` from supplied
`params`, `structure`, and `dh`. It does not read robot parameters itself,
which allows parameter sweeps.

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
the shared `robot.model` is not changed. The sampled gravity maximum is an
observed maximum, not a global bound. Run `example_dynamics_workflow()` for
a full example and `validate_dynamics_analysis(robot)` for numerical checks.
