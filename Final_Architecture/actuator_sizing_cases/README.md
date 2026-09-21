# Preliminary actuator load cases

This folder defines *observed load cases*, not final motor or gearbox requirements. The current robot has estimated geometry, masses, COMs and inertias. Welding geometry and TCP limits here are preliminary Control-team examples, not approved client duty-cycle requirements. Collision, workpiece, thermal duty cycle, drive efficiency and gearbox effects are not included.

## Run in MATLAB

From the repository root in MATLAB:

```matlab
addpath('Final_Architecture', ...
        'Final_Architecture/kinematics', ...
        'Final_Architecture/trajectory', ...
        'Final_Architecture/dynamics_analysis', ...
        'Final_Architecture/actuator_sizing_cases', ...
        'Final_Architecture/validation');
suite = example_actuator_sizing_cases();
checks = validate_actuator_sizing_cases();
```

Or from PowerShell (the quotes make the command run *inside MATLAB*):

```powershell
matlab -batch "addpath('Final_Architecture','Final_Architecture/kinematics','Final_Architecture/trajectory','Final_Architecture/dynamics_analysis','Final_Architecture/actuator_sizing_cases','Final_Architecture/validation'); suite=example_actuator_sizing_cases(false); checks=validate_actuator_sizing_cases();"
```

For a specific case:

```matlab
robot = loadRobot();
cases = createActuatorSizingCases(robot);
[trajectory, report] = generateTrajectory(robot, cases(2));
assert(report.pass, report.reason);
dynamics = analyzeTrajectoryDynamics(robot, trajectory, struct('gravity',[0 0 -9.81]));
plotDynamicsResults(dynamics);
```

For synchronized robot/torque playback instead of separate plots:

```matlab
session = inspectDynamicsCase(robot,"LINE_WELD");
session = inspectDynamicsCase(robot,"GRAVITY_HOLD");
```

See [visualization/README.md](../visualization/README.md) for the figure
controls, editable-case examples, external trajectory input, and
interpretation. `example_dynamics_workflow()` remains a small J2 teaching
test, not a motor-sizing case.

`trajectory.time` is N x 1 seconds; `q`, `qd`, `qdd` are N x DOF in rad, rad/s, rad/s^2. `trajectory.cartesian` and `trajectory.meta` are optional diagnostics. The dynamics module consumes only the four core arrays.

## Current case catalog

| ID | Status of input data | Meaning |
| --- | --- | --- |
| GRAVITY_HOLD | preliminary | Joint-limit-domain numerical multistart gravity search, separately reported; not a certified global optimum |
| LINE_WELD | adapted preliminary | 40 mm straight line, constant orientation |
| ARC_WELD | adapted preliminary | Three-point arc with path-following orientation |
| FULL_CIRCLE_WELD | adapted preliminary | 40 mm-diameter full circle, constant closed orientation |
| MULTI_SEGMENT | adapted preliminary | Three lines; full stop at both internal corners, no blending |
| CONTROL_LINE_ORIGINAL | infeasible source, disabled | Original +500 mm X line from original Control qStart; endpoint is outside current robot's gross reach |
| TRANSFER_MOVE | requirements missing, disabled | Intended to exercise the same generator when approved move, speed, acceleration and jerk are supplied |
| FULL_CIRCLE_FOLLOW_ORIENTATION | failed continuity trial, disabled | No branch met the current spatial joint-step limit at sample 2 |

The enabled motion values are 0.10 m/s, 0.25 m/s^2, 1.0 m/s^3 and 0.05 s output step, with 5 mm spatial IK knots. These are Control-team preliminary test values, not welding specifications. Cases must be reconsidered whenever a different robot model is loaded. The generator itself uses only the generic `loadRobot` interface.

## Method and reliability

`generatePathGeometry` and `evaluatePathGeometry` make a line, three-point arc or full circle. `timeScalingProfile` maps physical TCP speed, acceleration and jerk through path length into normalized progress `s(t)`. Its S-curve starts and ends with zero speed and acceleration. Trapezoidal speed profiles are available for exploration, but their ideal acceleration jumps are excluded from sizing. A multi-line path concatenates separate segments and stops at each junction.

`solveTrajectoryIK` collects limit-valid, FK-checked IK candidates for each pose, then selects a continuous joint sequence across the full path. It records when no continuous branch exists rather than silently jumping. `computeJointDerivatives` fits each joint path versus geometric progress with a cubic spline, then applies the chain rule: `qd = dq/ds * sd`, `qdd = d2q/ds2 * sd^2 + dq/ds * sdd`. This avoids applying two finite-difference gradients to time samples. A spline is still an approximation of the IK path, so timed FK is checked again. No generator computes inverse dynamics itself.

The suite repeats each enabled trajectory at `dt`, `dt/2`, and `dt/4`, comparing peak absolute joint speed, acceleration, torque, and time-weighted case RMS torque between the final two resolutions. The default acceptance threshold is 8% relative change, with small denominators floored by explicit physical scales. Nonconverged and failed trajectories stay in `suite.cases` with reasons but are excluded from `suite.aggregate`. `suite.aggregate.torqueSpeed(j)` retains *paired* speed and torque samples with case ID and time. Extremum events retain case ID, time, configuration, speed, torque and acceleration. Per-case RMS is **not** combined into an imaginary mission RMS; a real duty cycle is needed for that. Time-step convergence does not prove convergence against the *spatial* IK-knot spacing; the current 5 mm knot setting is preliminary.

## Control migration decisions

The line, three-point circle and full-circle geometric constructions were refactored from `Control/Trajectory/generateLineWaypoints.m`, `generateCircleWaypoints.m`, and `generateFullCircleWaypoints.m` into parametric path data and evaluators. Physical-to-normalized time scaling and the seven-stage jerk-limited profile were adapted from `TimeScaling/sCurveTimeScaling.m`; trapezoidal scaling was retained for nonsizing studies. Sequential IK, FK checks and orientation-following ideas came from the `Pipeline/SingleSegmentTest_*` and `trajectoryPipelineArcLengthADLS.m` scripts, but use the generic robot interface. The old `generateCartesianPosePath.m` was not copied because its `slerpOrientation` dependency was missing; rotation interpolation now uses Robotics System Toolbox rotation conversion functions.

The exploratory `Testing/` scripts, duplicated quaternion helpers, UR5 configuration/model/station setup, `controlFK` and `ADLS_IK` were intentionally not migrated. Cubic and quintic time laws remain in the external Control source only; the initial load-case suite requires the validated S-curve, while trapezoidal remains available for comparison. `Final_Architecture` does not call or add the external Control tree to its MATLAB path.

The orientation-following full circle was attempted with all current analytical branches and failed the present spatial joint-step limit at sample 2; the enabled full-circle case explicitly uses constant orientation. A real welding tool-angle requirement may need a different start pose, smaller circle, or finer path spacing. This check does not establish that the orientation-following circle is physically impossible.

Before final actuator selection: replace estimated mass properties with CAD measurements; define real workpiece/weld/transfer geometry, orientation, cycle timing and speed limits; add collision and process constraints; validate IK branches and trajectory tracking; define a mission duty cycle; and include gearing, efficiency, reflected inertia, thermal ratings and margins.
