# Dynamics playback

Run these commands in the MATLAB Command Window with the repository root as
MATLAB's Current Folder. `addpath` is a MATLAB command, not a PowerShell
command.

```matlab
addpath('Final_Architecture', ...
    'Final_Architecture/kinematics', ...
    'Final_Architecture/trajectory', ...
    'Final_Architecture/dynamics_analysis', ...
    'Final_Architecture/actuator_sizing_cases', ...
    'Final_Architecture/visualization')
robot = loadRobot();
session = inspectDynamicsCase(robot,"LINE_WELD");
```

The case command accepts `"ARC_WELD"`, `"FULL_CIRCLE_WELD"`,
`"MULTI_SEGMENT"`, and `"GRAVITY_HOLD"` too. Use
`cases = createActuatorSizingCases(robot); disp(string({cases.id})')` to list
all IDs. Disabled cases report their reason and open no viewer.

To edit a case without changing its source file:

```matlab
cases = createActuatorSizingCases(robot);
custom = cases(2);                 % LINE_WELD
custom.motion.tcpSpeed = 0.05;    % m/s
session = inspectDynamicsCase(robot,custom);
```

The selected case is generated, validated, time-resolution checked, and
analyzed before playback. A failed or nonconverged case does not open the
viewer. `session.status`, `session.reason`, `session.caseResult`, and
`session.suite` retain the result.

Viewer options:

```matlab
opts = struct('gravity',[0 0 -9.81], 'joint',2, ...
    'playbackRate',0.25, 'autoplay',false);
session = inspectDynamicsCase(robot,"ARC_WELD",opts);
```

`gravity` is in base-frame m/s^2 (default `[0 0 -9.81]`). `joint` is an
integer joint index (default 2). `playbackRate` is 0.25, 1, or 2 times the
trajectory clock (default 1). `autoplay` defaults to true. The figure has
play/pause, restart, a time slider, speed and joint selectors, and a
selected-joint peak jump. The six small plots show required joint torque;
the focused plot splits it into gravity, acceleration-related inertial, and
velocity-related torque. The readout shows the *same sample's* angle, speed,
torque, and mechanical power. Peak circles indicate observed trajectory
peaks, not general worst-case guarantees.

For a trajectory supplied by another team, use the generic interface:

```matlab
% trajectory.time: N-by-1 seconds, strictly increasing
% trajectory.q:    N-by-6 radians
% trajectory.qd:   N-by-6 rad/s
% trajectory.qdd:  N-by-6 rad/s^2
report = validateTrajectory(robot,trajectory);
assert(report.pass,report.reason)
dynamics = analyzeTrajectoryDynamics(robot,trajectory, ...
    struct('gravity',[0 0 -9.81]));
viewer = playTrajectoryDynamics(robot,trajectory,dynamics);
```

`playTrajectoryDynamics` does not recalculate dynamics. Its inputs must
refer to the same robot and matching trajectory arrays. Playback reuses
precomputed samples; if rendering falls behind, it skips display frames
instead of changing the motion timing. It is not a forward-dynamics or
motor-control simulation.

For independent stationary configurations:

```matlab
session = inspectDynamicsCase(robot,"GRAVITY_HOLD");
session.viewer.selectJoint(3);  % J3's found peak pose
disp(session.viewer.getState())
```

The case runner's gravity defaults are 200 random poses and two optimizer
starts per torque direction. To search more thoroughly before opening the
same stationary viewer:

```matlab
opts = struct();
opts.runOptions = struct('gravitySamples',1000, ...
    'gravityStarts',4,'gravityMaxEvaluations',1000,'seed',1);
session = inspectDynamicsCase(robot,"GRAVITY_HOLD",opts);
```

This is still a numerical search, not a certified global maximum.
The stationary view jumps between found per-joint peaks and named candidate
poses; the jump is **not** a planned transition. Each bar is the torque
needed to hold one joint at the displayed pose. Zero speed means zero
mechanical joint power, even when holding torque is nonzero.

Numerical checks establish agreement among FK, mass properties, gravity
moments, inverse dynamics, and graph/sample timing. They do not establish
that the preliminary CAD dimensions, COMs, inertias, motor/gearbox mounting,
collision geometry, welding force, gearbox efficiency, or thermal duty
cycle match a real machine. Motion peaks may exceed static gravity peaks
because accelerating mass and velocity effects add torque.

Test the viewer with `validate_dynamics_viewer()` from
`Final_Architecture/validation`. It runs the enabled preliminary cases
and checks graph/robot synchronization in hidden figures.
