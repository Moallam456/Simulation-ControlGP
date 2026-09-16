# Final Architecture

This folder is organized so the active robot is selected once and generic
analysis code receives a common robot interface.

## Main Usage

```matlab
addpath('Final_Architecture', ...
        'Final_Architecture\kinematics', ...
        'Final_Architecture\visualization', ...
        'Final_Architecture\validation', ...
        'Final_Architecture\workspace_analysis')

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
