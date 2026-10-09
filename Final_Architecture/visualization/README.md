# Dynamics visualization

From the repository root, run:

```matlab
addpath(genpath(fullfile(pwd,'Final_Architecture')))
session = actuator_sizing_analysis;
```

The results dashboard selects a case and joint. `Animate case` opens the
synchronized robot and torque viewer for a passing motion. `Gravity poses`
opens the stationary viewer and lets you jump between the independently found
gravity-peak configurations.

The motion viewer provides play/pause, restart, a time slider, 0.25x/1x/2x
playback, joint selection, and jump-to-peak. Its six small plots show total
joint torque. The focused plot separates total, gravity, inertial, and
velocity-related torque. Angle, speed, torque, and power always refer to the
same displayed trajectory sample.

The viewer replays a precomputed trajectory. It does not apply torque to a
forward simulation and does not recalculate dynamics during playback.

For a trajectory supplied by another team, the lower-level generic interface
remains available:

```matlab
report = validateTrajectory(robot,trajectory);
assert(report.pass,report.reason)
dynamics = analyzeTrajectoryDynamics(robot,trajectory, ...
    struct('gravity',[0 0 -9.81]));
viewer = playTrajectoryDynamics(robot,trajectory,dynamics);
```

`trajectory.time` is N-by-1 seconds. `q`, `qd`, and `qdd` are N-by-DOF in
radians, radians per second, and radians per second squared. Dynamics and
trajectory data must refer to the same robot and samples.

Motion peaks can exceed stationary gravity peaks because acceleration and
velocity effects add to gravity torque. A nonzero stationary holding torque
still has zero mechanical power because joint speed is zero.

Run `validate_dynamics_viewer(robot,session.suite)` to check graph/sample
synchronization using hidden figures. These checks do not validate CAD mass
properties, collision geometry, welding forces, gearing, or thermal capacity.
