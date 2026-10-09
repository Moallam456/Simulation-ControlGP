# Actuator load study

This folder has one public MATLAB function:

```matlab
actuator_sizing_analysis
```

It loads the robot selected in `projectConfig.m`, obtains that robot's cases
from `robots/<robotID>/dynamicsTestCases.m`, runs the cases, checks time-step
convergence, and opens one results dashboard.

## What the analysis does

For a moving case, the workflow is:

```text
Cartesian or joint-space path -> joint trajectory -> inverse dynamics -> convergence check
```

For a stationary case, it searches the joint-limit domain for large gravity
holding torques. Passing moving cases are compared without separating torque
from its simultaneous speed. Static gravity peaks are shown at zero speed.

The dashboard contains:

- Case status and failure reasons.
- Static gravity and moving peak torque for every joint.
- Largest single-case RMS torque, speed, and power.
- The case responsible for each reported moving peak.
- Torque-component and paired torque-speed plots.
- Controls for opening the synchronized motion or gravity-pose viewer.

## Important interpretation

`PASS` means that the software generated a valid trajectory and that its
reported speed, acceleration, torque, and RMS torque passed the configured
time-step convergence check. It does not prove physical accuracy.
`VISUAL_ONLY` cases can be animated with synchronized exploratory torques,
but their peaks and RMS values are excluded from the motor-sizing summary.
Where requested by a case, collision and clearance are checked at sampled
configurations; this does not verify the swept motion between samples.

The result remains `PRELIMINARY LOAD STUDY - NOT FINAL MOTOR SIZING` while
robot limits, TCP, mass properties, collision status, case data, or other
readiness items remain preliminary. The analysis does not include friction,
gearing, drive efficiency, motor rotor inertia, thermal limits, safety margin,
external welding forces, or continuous swept-volume collision checking.

Per-case RMS torque describes heating during that one motion. A real mission
RMS requires the actual sequence of motions, dwell times, and repetition rate.

## Editable settings

The clearly marked settings at the top of `actuator_sizing_analysis.m` control
gravity, convergence tolerance, gravity-search effort, and dashboard display.
Motion geometry and timing belong to each robot's `dynamicsTestCases.m` file.

The returned session can be retained for programmatic inspection:

```matlab
session = actuator_sizing_analysis;
session.suite.jointSummary
session.suite.cases
```
