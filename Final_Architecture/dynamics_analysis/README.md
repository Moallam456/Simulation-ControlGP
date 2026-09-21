# Dynamics Analysis

This folder reports **ideal rigid-body joint-side requirements**. It accepts
any serial robot returned by `loadRobot` with row-format `rigidBodyTree` data.
It does not size a motor or gearbox and does not assume external process forces.

## Files and Public Calls

| File | Role |
| --- | --- |
| `validateDynamicModel(robot)` | Checks topology, body mass, COM, inertia, parameter mapping, and provenance. Returns `pass`, `tests`, `errors`, `warnings`, `bodyNames`, `dataQuality`, `dataSource`, `preliminary`. |
| `analyzeGravityLoading(robot,options)` | Evaluates robot-specific stationary candidate poses by default, plus `options.q` and/or seeded `options.numSamples`; returns observed extrema and samples. |
| `optimizeGravityLoading(robot,options)` | Separately searches positive and negative gravity peaks for every joint using sampled starts and bounded `fmincon`. Requires Optimization Toolbox. |
| `analyzeTrajectoryDynamics(robot,trajectory,scenario)` | Calculates `massMatrix`, `velocityProduct`, `gravityTorque`, and `inverseDynamics` for every sample. |
| `summarizeDynamicsResults(results,robot)` | Builds joint summary and peak-event state records. |
| `plotDynamicsResults(results,options)` | Plots torque, paired torque/speed, power, and optional decomposition for `options.joint`. |
| `example_dynamics_workflow()` | Runnable example; Control can replace its generated trajectory with real motion arrays. |

`prepareDynamicsModel`, `validateConfigurations`, and `dynamicsMetadata` are
shared internal helpers. Validation tests are in `validation/`:
`validate_dynamics_analysis.m`, `verifyDynamicsConsistency.m`,
`estimateGravityMomentFromCom.m`, and `test_one_link_dynamics.m`.
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
and `sampledMaximumIsGlobalBound=false`. `poseNames` identifies user, named
candidate, and random rows; `candidatePoseCount` counts the included catalog.
`options.includeCandidatePoses=false` omits the catalog when evaluating only
user-supplied poses. Random samples use a local restored
RNG state. They do not prove a global worst case.

`optimizeGravityLoading` improves this by separately maximizing positive
and negative gravity torque for every joint, then choosing the larger
absolute result. Each joint can have a different maximizing configuration.
Its `summaryTable` reports the best found torque; `qAtMaxAbs`,
`qAtPositiveMax`, and `qAtNegativeMin` contain the corresponding full poses.
`sampled` retains the initial samples, and `nonpositiveExitCount` shows
optimizer runs that hit a stopping limit. This is still a numerical search,
not a certified global maximum (`meta.globalMaximumCertified=false`).
The default is 1,000 random poses plus home and the robot-specific candidate
poses, then up to four local starts
for each sign of each joint. Its domain is joint limits only; collision and
workpiece constraints are not included.

## Physical Data and Caveats

`robots/initial_trial/robotParameters.m` owns component masses, geometry,
estimated radii, and optional explicit COM/inertia. `buildModel.m` maps them
into the tree. Where COM/inertia are NaN, it estimates them from the stated
cylinder/sphere approximation in the body frame. Explicit finite 3-by-3
inertia matrices use kg*m^2 about the COM, aligned with the body axes.
`buildModel` shifts them to the body-frame origin with the parallel-axis
theorem before converting to MATLAB's `[Ixx Iyy Izz Iyz Ixz Ixy]` ordering.
The same final shift is applied to the simplified geometry estimates and
the combined tool/structural segment. Check CAD reference point,
frame, sign, and units before replacing them. `params.tool.inertia` similarly
accepts an explicit tool tensor about the tool COM in TCP axes; the 0.71 kg
structural segment is then combined by the parallel-axis theorem. `validateDynamicModel` checks
explicit property mapping and inertia tensor eigenvalues.

The 12 kg base assembly is a fixed `base_structure` body; it does not rotate
with J1. Each moving body `link_i` carries its outgoing Ji-to-J(i+1)
segment. The preliminary J3-J6 motor/gearbox modules are assigned at their
joint centers to the upstream body, an assumption to confirm against CAD.
`link_6` is a massless joint frame. The 2.0 kg welding tool/payload is a
fixed TCP mass. The separate 0.71 kg J6-to-EE structural segment is combined
with it in the TCP body once. The
provided horizontal COM distances are retained as reference measurements;
the current body COMs are calculated from the 3D routed geometry, so they are
not assumed to be identical in the drawn home pose.

**All positive-mass bodies currently have estimated mass properties.** The
fixed base (12 kg) and J1-to-J2 structure (10 kg) are provisional. The J3-to-J4 offset
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

The example uses a smooth 4-second J2 move. It is a teaching/smoke test,
not the actuator-sizing input. Its values are illustrative;
it has 81 time samples, starts at home, and moves J2 by +20 degrees while
all other joints stay at home. `output.inputTrajectory` holds the actual
time, q, qd, and qdd arrays. `analyzeTrajectoryDynamics` evaluates these
snapshots; it does not itself animate or command a physical robot.
The example's gravity check evaluates home, robot-specific candidates, and
100 random poses. The validation gravity check uses 20 random poses.
For a separate stronger gravity search, run:

```matlab
robot = loadRobot();
options = struct('gravity',[0 0 -9.81], 'numSamples',1000, ...
                 'numStarts',4, 'seed',1);
peaks = optimizeGravityLoading(robot,options);
disp(peaks.summaryTable)
disp(peaks.qAtMaxAbs) % row j is the pose for joint j's absolute peak
```

These results are still preliminary because the robot mass properties are
estimated. Replace the example trajectory with a representative mission
before making design decisions. To compare geometries, copy `robot.params`, edit its geometry or
mass properties, call `loadRobot(robot.id,modifiedParams)`, and analyze that
new robot with the same scenario and motion contract.

## Stationary Pose Catalog And Load Cases

`robots/initial_trial/gravityCandidatePoses.m` owns named home and horizontal
shoulder poses plus a joint-limit-filtered shoulder/elbow/wrist screening grid.
`loadRobot` exposes it as `robot.gravityPoses`. These are stationary **search
seeds**, not a claim that one standard horizontal pose is the worst for every
joint. Gravity torque depends on the downstream COM lever arms about each
joint axis; wrist orientation can change those arms. The grid is not collision
checked. A different robot can supply its own catalog in its robot folder.

From MATLAB, after adding `Final_Architecture`, `dynamics_analysis`,
`actuator_sizing_cases`, `trajectory`, `kinematics`, and `visualization` to the
path, inspect named poses and hold torques:

```matlab
robot = loadRobot();
disp(table(robot.gravityPoses.names,rad2deg(robot.gravityPoses.q), ...
    'VariableNames',{'Pose','JointAngles_deg'}))
staticResult = analyzeGravityLoading(robot,struct('gravity',[0 0 -9.81], ...
    'numSamples',1000,'seed',1));
disp(staticResult.summaryTable)
j = 2;
disp(rad2deg(staticResult.qAtMaxAbs(j,:)))
```

Each row of `staticResult.q` is an independent stationary pose, and the matching row
of `staticResult.torque` is its six-joint holding torque. `staticResult.poseNames` labels each
row. For just one pose without the catalog or random samples:

```matlab
q = deg2rad([0 90 0 0 0 0]);
one = analyzeGravityLoading(robot,struct('gravity',[0 0 -9.81], ...
    'q',q,'includeCandidatePoses',false));
disp(one.torque)
```

For a stronger *per-joint* numerical search (Optimization Toolbox required):

```matlab
peaks = optimizeGravityLoading(robot,struct('gravity',[0 0 -9.81], ...
    'numSamples',1000,'numStarts',4,'seed',1));
disp(peaks.summaryTable)
disp(rad2deg(peaks.qAtMaxAbs(2,:))) % J2's found worst pose in degrees
plot_robot(robot,peaks.qAtMaxAbs(2,:))
```

`peaks.qAtMaxAbs(j,:)` is joint `j`'s own maximizing configuration; rows do
not need to be the same pose. `peaks.sampled` contains every starting pose.
Set `includeCandidatePoses=false` to omit the catalog, or supply `q` as extra
starting rows. Increasing `numSamples` and `numStarts` improves exploration
but does not certify a global maximum. No stationary pose alone bounds the
torque of a moving robot.

For motion loads, use the trajectory cases in `actuator_sizing_cases`:

```matlab
suite = example_actuator_sizing_cases();
disp(suite.aggregate.caseComparison)
disp(suite.gravity.summaryTable)
disp(suite.cases(2).dynamics.summaryTable) % LINE_WELD if status is PASS
plotDynamicsResults(suite.cases(2).dynamics,struct('joint',2))
```

`suite.cases` records PASS, FAIL, DISABLED, or NOT_CONVERGED for each case.
`suite.aggregate` includes only passing motion cases and preserves paired
torque-speed samples. To run one motion with edited path or timing, create
the case catalog, change the desired case, call `generateTrajectory`, then
pass its time/q/qd/qdd into `analyzeTrajectoryDynamics` (see the
`actuator_sizing_cases/README.md` example). The example cases do not cover a
real transfer move or true cycle-level thermal RMS; both require mission data.
