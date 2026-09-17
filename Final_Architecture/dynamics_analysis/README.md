# Dynamics Analysis

This folder reports **ideal rigid-body joint-side requirements**. It accepts
any serial robot returned by `loadRobot` with row-format `rigidBodyTree` data.
It does not size a motor or gearbox and does not assume external process forces.

## Files and Public Calls

| File | Role |
| --- | --- |
| `validateDynamicModel(robot)` | Checks topology, body mass, COM, inertia, parameter mapping, and provenance. Returns `pass`, `tests`, `errors`, `warnings`, `bodyNames`, `dataQuality`, `dataSource`, `preliminary`. |
| `analyzeGravityLoading(robot,options)` | Evaluates `options.q` and/or seeded `options.numSamples` at `options.gravity`; returns observed extrema and samples. |
| `analyzeTrajectoryDynamics(robot,trajectory,scenario)` | Calculates `massMatrix`, `velocityProduct`, `gravityTorque`, and `inverseDynamics` for every sample. |
| `summarizeDynamicsResults(results,robot)` | Builds joint summary and peak-event state records. |
| `plotDynamicsResults(results,options)` | Plots torque, paired torque/speed, power, and optional decomposition for `options.joint`. |
| `verifyDynamicsConsistency(robot,options)` | Reproducible decomposition, gravity, mass matrix, and forward/inverse checks. |
| `estimateGravityMomentFromCom(robot,q,gravity)` | Independent static COM lever-arm audit for a serial chain. |
| `example_dynamics_workflow()` | Runnable example; Control can replace its generated trajectory with real motion arrays. |

`prepareDynamicsModel`, `validateConfigurations`, and `dynamicsMetadata` are
shared internal helpers. Validation tests are in `validation/`:
`validate_dynamics_analysis.m` and `test_one_link_dynamics.m`.
`run_all_validation.m` includes the dynamics suite.

## Input Contract

```matlab
trajectory.time  % N-by-1, strictly increasing seconds, N >= 2
trajectory.q     % N-by-DOF radians
trajectory.qd    % N-by-DOF rad/s
trajectory.qdd   % N-by-DOF rad/s^2
scenario.gravity % 1-by-3 m/s^2 in the robot base frame
```

All arrays must be finite, and `q` must respect the loaded robot's joint
limits. There is no numerical differentiation of `q`. `options.gravity` is
equally required for static gravity analysis. The model is copied before
gravity is set, so the supplied robot is not changed. For `initial_trial`,
base +Z is up, so normal Earth gravity is `[0 0 -9.81]`.

## Outputs

`analyzeTrajectoryDynamics` returns `meta`, `time`, `state.q/qd/qdd`,
`torque.total/gravity/inertial/velocity/residual`, `power.joint`,
`torqueSpeed.speed/torque`, `summaryTable`, and `peakEvents`.
Torque, speed, and power matrices are N-by-DOF. `peakEvents(j)` contains
the full q, qd, qdd, time, sample index, signed torque, simultaneous speed
and power, and torque components at joint j's observed absolute torque peak.
`summaryTable` names joints and includes observed signed/absolute extrema,
time-weighted RMS torque over this trajectory, speed, acceleration, and
mechanical power extrema. Negative power means mechanical power is returned
at the joint; it is not a prediction of electrical regeneration.
Joint labels come from `robot.structure.jointNames` in model column order.

`analyzeGravityLoading` returns all evaluated `q` and torques, per-joint
maximum observed absolute torque and its configuration, signed extrema,
and `sampledMaximumIsGlobalBound=false`. Random samples use a local restored
RNG state. They do not prove a global worst case.

## Physical Data and Caveats

`robots/initial_trial/robotParameters.m` owns component masses, geometry,
estimated radii, and optional explicit COM/inertia. `buildModel.m` maps them
into the tree. Where COM/inertia are NaN, it estimates them from the stated
cylinder/sphere approximation in the body frame. Explicit finite 3-by-3
inertia matrices use kg*m^2 about the COM in the body frame and are converted
to MATLAB's `[Ixx Iyy Izz Iyz Ixz Ixy]` ordering. Check CAD reference point,
frame, sign, and units before replacing them. `params.tool.inertia` similarly
accepts an explicit tool tensor about the tool COM in TCP axes; the 0.71 kg
structural segment is then combined by the parallel-axis theorem. `validateDynamicModel` checks
explicit property mapping and inertia tensor eigenvalues.

The 2.0 kg welding tool/payload is a fixed TCP mass. The separate 0.71 kg
J6-to-EE structural segment is combined with it in the TCP body once. Link
component masses include the preliminary J3-J6 joint modules once. The
provided horizontal COM distances are retained as reference measurements;
the current body COMs are calculated from the 3D routed geometry, so they are
not assumed to be identical in the drawn home pose.

**All seven bodies currently have estimated mass properties.** `link_1`
(12 kg) and `link_2` (10 kg) masses are provisional. The J3-to-J4 offset
split, several other lengths, radii, and all inertias also need CAD checks.
The tool mass is a preliminary combined tool/payload estimate. Results are
marked `PRELIMINARY`; numerical consistency does not validate physical data.
No friction, reflected rotor inertia, transmission loss, variable payload,
external wrench, thermal duty cycle, or safety margin is included.

## Run

In the MATLAB Command Window from the repo root:

```matlab
addpath('Final_Architecture','Final_Architecture/dynamics_analysis', ...
        'Final_Architecture/validation')
output = example_dynamics_workflow();
check = validate_dynamics_analysis(loadRobot());
```

In PowerShell:

```powershell
matlab -batch "addpath('Final_Architecture','Final_Architecture/dynamics_analysis','Final_Architecture/validation'); output=example_dynamics_workflow(); check=validate_dynamics_analysis(loadRobot());"
```

The example uses a smooth 4-second J2 move. Its values are illustrative;
replace the trajectory with a representative mission before making design
decisions. To compare geometries, copy `robot.params`, edit its geometry or
mass properties, call `loadRobot(robot.id,modifiedParams)`, and analyze that
new robot with the same scenario and motion contract.
