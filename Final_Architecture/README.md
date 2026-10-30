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
robot.dynamicsCases
robot.gravityPoses
```

Generic calculations accept this `robot` struct. The public workspace,
collision, visualization, static-analysis, and dashboard commands load the
robot selected in `projectConfig.m` when called without arguments.

## Robot Models

`projectConfig.m` currently selects `robot_ver2`. Each model has its own
folder, while analysis code remains shared.

| Model | Construction | IK | Physical data |
| --- | --- | --- | --- |
| `robot_ver2` | Imported six-joint URDF and seven STLs | Control-team closed form inside its own `solveInverseKinematics.m` | Masses, COMs, and inertias from its URDF |
| `robot_ver1` | Imported six-joint URDF and seven STLs | Numerical multistart | User-supplied masses with retained COMs and mass-scaled inertias in its URDF |
| `initial_trial` | Programmatic `rigidBodyTree` | MATLAB analytical generation, with numerical fallback | Values and simplified shapes from `robotParameters.m` and `collisionGeometry.m` |

### robot_ver2

```text
robots/robot_ver2/
  robotDefinition.m
  solveInverseKinematics.m
  dynamicsTestCases.m
  multibody/
    robot_ver2_multibody.slx
    open_robot_ver2_multibody.m
    build_robot_ver2_multibody.m
  collision/collisionGeometry.m
  CAD/URDF_Assembly_Robot_ver2/
    urdf/URDF_Assembly_Robot_ver2.urdf
    meshes/robot_ver2_*.STL
```

`robotDefinition.m` imports the URDF and derives the common robot interface.
The URDF owns joint frames, mesh references, masses, COMs, and inertias;
`solveInverseKinematics.m` contains the control-team equations and nominal
geometry as local functions, exposes `robot.solveIK`, and checks each returned
branch against imported URDF FK and active limits. No IK source outside
`Final_Architecture` is required. The original `RecentIK` folder is retained
as historical source, not loaded by this robot.
The model has six moving joints, and the copied mesh names are unique to
`robot_ver2`. See [robot_ver2/README.md](robots/robot_ver2/README.md) for its
CAD conversion and case details.

For **robot_ver2 only**, run `open_robot_ver2_multibody`, then **Ctrl+D** and
**Run** in Simulink. The saved Simscape Multibody control bench contains a
six-joint torque-driven `RobotPlant`, a removable demonstration controller,
and measured joint/TCP signals. The URDF and meshes remain in this robot's
folder. This model is explicitly robot_ver2 regardless of `projectConfig.m`.
See [Multibody Instructions](robots/robot_ver2/multibody/README.md) for opening,
signal units, regeneration, validation and modeling scope. No other robot
model or existing analysis is converted or replaced.

**Open inputs for `robot_ver2`:** The `[-pi,pi]` joint ranges are configured
placeholders, not measured mechanical stops. Workspace plots use the fixed
`tcp` frame with the user-measured Joint-6-relative offset
`[-22.87, +20.53, +245.77] mm` and unchanged axes. The URDF's current mass properties are the values
used by MATLAB. The model's `readiness.actuatorSizingAllowed=false` flag means
the dashboard labels its outputs as a load study, not final motor sizing.

### robot_ver1

```text
robots/robot_ver1/
  robotDefinition.m
  solveInverseKinematics.m
  dynamicsTestCases.m
  collision/collisionGeometry.m
  CAD/robot_ver1/
    urdf/robot_ver1.urdf
    meshes/*.STL
```

The `robot_ver1` URDF owns its transforms, visual/collision meshes, and
inertial values. Its Joint 6 frame and end-effector mesh came from the
preserved `URDF_Files(ver2)` export. Its configured limits are `[-pi,pi]`;
the CAD export has zero-width limits. Unlike `robot_ver2`, `robot_ver1` has a
**known `Link_3`/`Wrist_2` collision at its zero-angle pose**. That detected
collision is reported by its collision metadata, not silently suppressed.
See [robot_ver1/README.md](robots/robot_ver1/README.md) for its mass list.

### initial_trial

```text
robots/initial_trial/
  robotParameters.m
  robotStructure.m
  kinematicChain.m
  buildModel.m
  solveInverseKinematics.m
  dynamicsTestCases.m
  gravityCandidatePoses.m
  actuation/actuatorParameters.m
  collision/collisionGeometry.m
```

`robotParameters.m` owns dimensions, masses, limits, home pose, and tool
transform. `robotStructure.m` owns topology and axes; `kinematicChain.m`
derives fixed transforms; and `buildModel.m` attaches the shapes supplied by
`collisionGeometry.m`. This model is programmatic and uses simplified
collision primitives rather than imported CAD mesh collisions. It supports
parameter sweeps without editing a URDF. Its robot-specific gravity poses
are exposed as `robot.gravityPoses`.

## Inverse Kinematics

`robot.solveIK(targetPose,struct('qSeed',qStart))` returns the distinct,
joint-limit-valid configurations it finds for an exact TCP pose in
`solutions.q` (N-by-DOF, radians). Rows are ordered by joint-space distance
from `qSeed`, so `solutions.q(1,:)` remains the nearest returned candidate.
Check `solutions.valid` before using any row. `solutions.info` reports the
method and per-candidate FK errors. IK does not filter self-colliding poses;
run `analyzeCollisions` on a selected configuration before using it.
`loadRobot` binds the selected robot's solver file once at load time. Repeated
IK calls do not add/remove robot folders from the MATLAB path, and loading
another robot does not redirect an existing robot's solver handle.

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

For `robot_ver2`, `robot.solveIK` uses the control team's closed-form solver
inside `robots/robot_ver2/solveInverseKinematics.m`. Every returned branch is
checked against imported URDF FK and configured limits. It has no numerical
fallback and does not filter collisions. `validate_robot_ver2_ik` checks
random, joint-boundary, and near-singular poses; compares against MATLAB's
numerical IK; measures runtime; and exercises the existing sweep trajectory
and the other robots' solver interfaces. It needs no arguments.
Branches with effectively zero `cos(q5)` are skipped by the closed form;
other branches may still solve that pose. Failure to find a branch is not
proof that the target is physically unreachable. Neither finite tests nor
numerical multistart establish completeness at singularities. The geometry
constants are specific to this robot revision: rerun validation after changing
URDF joint frames or link dimensions.

## Parameter Sweep Pattern

```matlab
baseRobot = loadRobot("initial_trial");
params = baseRobot.params;
params.geometry.l2 = params.geometry.l2 + 0.05;

robot = loadRobot("initial_trial", params);
workspace = workspaceAnalysis(robot,"reachable",struct('showPlot',false));
```

Generic code outside `robots/` should consume the loaded `robot` interface
and should not call robot-specific files directly.

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
the joint sliders on the right to pose it within the selected model's
configured joint limits. Angle and point-position readouts update
continuously; `Home` restores the configured home position, and `Frames`
toggles body frames. Rotating or
zooming the 3-D view keeps the robot and workspace point cloud synchronized.

For `robot_ver2`, FK, IK targets, Jacobians, plotted points, the point inspector,
and Cartesian trajectories all refer to the fixed `tcp` body. Its transform is
defined once by `tcp_fixed` in the robot's URDF and read into
`params.tool.TFlangeTCP`; it must not be multiplied a second time after
`getTransform(...,'tcp')`. The torch mass and mesh stay on `End_Effector`;
`tcp` is a massless frame. `robot_ver1` still uses the `End_Effector` origin
(no measured tip offset supplied for that model). `initial_trial` retains
its existing programmatic tool definition. These are separate robot models.

Click a plotted point to inspect its sampled joint angles and TCP position.
The inspector lists other sampled configurations in the same 25 mm voxel,
with their linear inverse-condition score, self-collision status, clearance,
and closest body pair. A voxel-center marker is a summary location, not
necessarily a position the robot exactly reaches; the selected sample's
actual TCP position is shown separately. **Reach** jumps the displayed robot
and sliders to that candidate. Colliding poses can also be shown, with a red
warning, for diagnosis. **Find alternatives** runs full-pose IK on demand;
new solutions are labeled and do not change workspace plots or statistics.
This checks self-collision at a pose, not clearance along a path or against
workcell objects.

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

The usable plot's **joint_histograms** button opens one angle histogram per
joint (six for the current robots). The default counts every accepted sampled
configuration once. The alternative **Best index per plotted voxel** counts
only the highest-scoring accepted sample in each displayed voxel, matching
the configuration behind its color; score ties use the earliest sample.
Both views exclude rejected samples and inspector-generated IK alternatives.
They reuse the current dataset without additional IK, FK, or collision checks.
Angles are in degrees and the vertical axis is configuration count. Set
`histogramBinWidthDeg` near the top of `usable_workspace.m` to change the
default 10-degree bins; this does not invalidate cached analysis results.
These are sampling-dependent workspace distributions, not time spent at an
angle, operating duty cycles, or an exhaustive count of possible IK solutions.

The usable window also offers **Points**, **Envelope**, and **Envelope + points**
in its bottom-left view selector. The envelope is a translucent alpha-shape
surface reconstructed on demand from accepted TCP samples. It is a visual
approximation, not evidence that every enclosed position or orientation is
usable. The single-color surface does not represent interpolated manipulability.
Points retain their original scores and remain inspectable in the combined view;
the surface does not intercept clicks. Robot sliders work in every view.
`envelopeAlphaRadius_m` and `envelopeOpacity` live near the top of
`usable_workspace.m`. The default NaN radius uses MATLAB's smallest all-points
alpha radius without enlargement. Larger explicit radii bridge wider gaps and
can erase cavities, even with hole filling disabled. A cross-section is more
reliable than an external translucent view for inspecting internal cavities.
Hole filling and small-region removal are disabled, but inferred boundaries
still depend on sample density and radius. Sparse, planar, or empty datasets
that cannot form a 3-D surface keep the point view. This display mode does not
alter sample acceptance, voxel volume, histograms, or cached analysis results.

Expensive analysis data is reused automatically. The generic
`analysis_cache/prepareWorkspaceDataset.m` caches sampled joint
configurations, FK transforms, TCP points, collision/clearance results, and
manipulability values in stages. Its signatures include kinematics, limits,
TCP transform and tool/model versions, sampling, collision geometry and STL contents, skipped pairs, clearance margin, and the
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
colliding, near-collision, and clear configurations. `robot_ver1` and
`robot_ver2` use their URDF STL meshes as collision geometry; MATLAB checks
mesh collisions using convex geometry. `robot_ver1` has the reported
`Link_3`/`Wrist_2` zero-pose collision, while `robot_ver2` has no detected
zero-pose self-collision. `initial_trial` uses simplified primitives.
Environment/workpiece collisions and continuously swept trajectory clearance
are not part of these pose-level results.

## Dynamics Analysis

`dynamics_analysis/` contains generic ideal rigid-body joint-side dynamics.
For `initial_trial`, the fixed tool (2.0 kg) and J6-to-EE structure (0.71 kg) are separate
component masses combined once in the TCP body. Link and joint-module masses
are defined in `robotParameters.m`; `buildModel.m` estimates COM and inertia
from the simplified geometry unless explicit COM/inertia are supplied.
For `robot_ver1`, body masses, COMs, and inertias come from its URDF;
its masses were set by the team and its COM inertias were scaled with them.
For `robot_ver2`, these properties come from its URDF after the Inventor
assembly-frame to link-frame conversion. The programmatic `initial_trial`
model uses its own parameter file and simplified inertia construction.

Run `static_analysis` without arguments to pose the robot with sliders and
calculate signed gravity-holding torque for every joint at that pose. It
uses the robot selected in `projectConfig.m`.

Run `park_position_analysis` without arguments for the robot in `projectConfig.m`.
No home angles or URDF values are overwritten. Settings are at the top of the
entry point; `optimizeParkConfiguration.m` is the single computation engine.
`findParkPosition.m` is only a compatibility alias, not another optimizer.

The search uses practical posture seeds plus local/global scrambled Sobol
samples (seeded random fallback), followed by multistart constrained SQP.
When Global Optimization Toolbox is available, a bounded feasible pattern-search
polish handles nonsmooth mesh-clearance boundaries without accepting collisions.
For **robot_ver2**, its `robotDefinition.m` contains folded-back forearm seeds
and default park-envelope limits: height 1.35 m and radial extent 0.65 m.
These are editable search requirements, not physical robot specifications.
Other models use their own optional `params.park` settings or generic seeds;
robot_ver2 joint-angle conventions are never applied to another robot.
Seeds follow the geometric principles of compactness, COM close to the base,
and distance from stops in
[Yaskawa's model-dependent transport-posture guidance](https://knowledge.motoman.com/hc/en-us/articles/33649986738834-Shipping-Position).
They are not copied shipping angles or proof of power-off stability.

With joint-output continuous holding ratings supplied in
`settings.allowableHoldingTorque_Nm`, the objective is
`max(abs(gravityTorque)./allowableHoldingTorque_Nm)`. Without ratings, the
objective is peak absolute joint torque in N m and utilization is **unavailable**.
URDF effort placeholders are never treated as motor ratings.
Alternatives target compactness, J2, and J3 under a bounded peak-load budget;
duplicate poses are omitted and load/compactness Pareto membership is recorded.
At least one home-accessible seed/refinement is retained when found, even if
an inaccessible pose dominates its static load and size. Candidates with
passing sampled routes from every valid start are preferred in the final list.
The search is finite and does not certify a global optimum.

Every candidate satisfies joint bounds with a stop margin, modeled
self-clearance, and whole-visual-model clearance above base Z=0. Actual mesh
scale and body-relative transforms are used; hull vertices preserve exact
plane extrema. The base and torch are included without adding tool mass.
Optional height/radius constraints encourage practical compact postures.
Gravity curvature is reported only as a diagnostic, **not** imposed as a hard
constraint that pushes the arm into a high-torque horizontal pose. Active
holding/brakes remain necessary; no power-off safety claim is made.

Return routes are checked from home and saved dynamics-case start poses.
Invalid starts are recorded, not silently treated as feasible. Direct joint
interpolation, a reference waypoint, and optionally MATLAB `manipulatorRRT`
are available. The slower RRT fallback is off by default and has a shared
attempt/iteration budget when enabled. Every accepted route is independently checked against the
mounting plane, self-clearance, and joint limits at a maximum per-joint step
of 2 degrees. This is **sampled**, not continuous swept-volume certification;
workcell obstacles and cable routing are not modeled. Failed routes remain
explicit and must not be sent to hardware. **Preview return** animates a
passing route. Welding-ready poses remain task-specific, separate from park.

The robot, table, and bars describe the displayed pose. Home comparison bars
are off by default. Run `validate_park_position` for analytic torque, objective,
geometry, route, and model regression checks.

## Force Analysis

`force_analysis/` contains `force_analysis.m` and `joint1ReactionLoads.m`.

Run `force_analysis` without arguments. It follows `projectConfig.m` and
provides live joint sliders, a **Calculate** button for static loads, a
collision-screened static search, and dynamic loads along the selected
robot's saved trajectory cases. This first version evaluates **Joint 1
only**; the existing `static_analysis` still gives holding torque at all joints.
Search settings, gravity, clearance margin, and preferred trajectory are at
the top of `force_analysis/force_analysis.m`. The default `robot_ver2`
trajectory is `JOINT_SWEEP_180`; `THREE_LEG_SWEEP` is also selectable.

The results are axial force, radial force, bending moment, and torque about
the joint axis. Signed force and moment vectors are **support-on-arm**, in
robot-base axes, with moments taken about the Joint-1 frame origin. The load
the arm applies to the support is the negative of these vectors. The joint
axis comes from the model, not an assumed global Z direction.

`force_analysis/joint1ReactionLoads.m` is the reusable calculation layer.
It attaches six stationary, massless virtual joints to a private model copy
and uses MATLAB [`inverseDynamics`](https://www.mathworks.com/help/robotics/ref/rigidbodytree.inversedynamics.html)
to recover the complete support wrench.
Only masses downstream of Joint 1 contribute; pedestal mass is excluded,
fixed tools are included, and the massless TCP adds no load. The original
robot, its six physical DOFs, inertias, and geometry are not changed.
The wrench is shifted from the base origin to Joint 1 before decomposition:

```text
M_J1 = M_base - p_J1 x F
axial force = abs(axis dot F)
radial force = norm(F - axis*(axis dot F))
bending moment = norm(M_J1 - axis*(axis dot M_J1))
axis torque = abs(axis dot M_J1)
```

For the current `robot_ver2`, supported mass is 17.495 kg. With gravity
`[0 0 -9.81]`, static axial force is 171.626 N and static radial force is
approximately zero. Bending changes with pose. **Zero net radial force does
not mean zero radial load in each bearing:** separated bearings must react
the bending moment. Bearing spacing, arrangement, thrust-load sharing,
preload, ratings, duty cycle, and service factors are needed for selection.
This tool supplies joint-level resultants, not a bearing recommendation.

Choose bending moment, axial force, radial force, or axis torque from the
dropdown. **Find peak configuration** evaluates 1,500 deterministic random
configurations plus home/current/individual limit poses and displays the
largest qualifying value of the selected quantity. Changing the dropdown
immediately selects that quantity's peak from the existing samples; if no
search exists, it starts one. Samples and collision checks are reused.
Candidates are screened in descending selected-load order; unchecked
lower-ranked samples are not labelled collision-free. The default clearance
margin is 10 mm. Bending is the magnitude of the moment perpendicular to the
Joint-1 axis, so its peak measures the largest sampled bending demand on the
support. For the vertical Joint 1 under gravity alone, axial force, radial
force, and axis torque are constant within numerical tolerance. These choices
display the first qualifying representative and explicitly report a tied
peak with no unique worst pose. Bending is never used to break their ties.
The search provides no global maximum proof, and reaching a displayed pose
is not a collision-checked motion.

Trajectory graphs include all generated samples and report clearance
failures. Their separate window has its own animated robot, joint-angle and
dynamic-load readouts, synchronized graph cursors, a time slider, play/pause,
restart, and speed controls. Playback starts automatically. Peak buttons pause
and display the corresponding pose in that same trajectory window. These
controls do not modify the static window's pose or results. Replay speed changes
animation speed only, not the physical trajectory timing or calculated loads. Playback uses
precomputed states/loads/collision evidence, without rerunning dynamics or
collision detection on each frame. Failed trajectory generation produces no load curves.
Sampled clearance does not establish continuously swept or environment
clearance. No contact, cable/process force, friction, preload, flexibility,
motor rotor inertia beyond the modeled bodies, or safety factor is added.
Model-readiness warnings remain available with the results.

Optional inspection: `session = force_analysis;`, then `state = session.getState();`
returns the current loads, checked collision evidence, search candidates,
trajectory states/loads, and model-quality metadata. Ordinary use requires
only `force_analysis`. Run `validate_joint_loads` for analytic, equilibrium,
COM-acceleration, and torque checks on all three robot models;
`validate_joint_loads(true)` also tests the active robot's UI and, for
`robot_ver2`, the 180-degree trajectory and peak-load convergence when the
time step is halved.

## Other Dynamics Tools

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

For `robot_ver1` and `robot_ver2`, dynamics results include
`results.meta.readiness`. Their current `actuatorSizingAllowed=false` flags
keep the consolidated dashboard in load-study mode. `robot_ver2` also has a
robot-specific `THREE_LEG_SWEEP` moving case and a `JOINT_SWEEP_180`
visualization-only case; `robot_ver1` defines `EXTREME_LONG_LINE`.

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

## Validation

```matlab
robot = loadRobot();
validate_kinematics(robot);
validateDynamicModel(robot);
validate_workspace_analysis(robot);
```

For the active `robot_ver2` URDF, run `validate_robot_ver2(robot)`.
To test its robot-local control-team IK after adding `Final_Architecture`:

```matlab
report = validate_robot_ver2_ik;
tcpReport = validate_robot_ver2_tcp;
```

For `robot_ver1`, use `validate_robot_ver1(loadRobot("robot_ver1"))`.
The `initial_trial` model uses `validate_kinematics(loadRobot("initial_trial"))`.
Its collision-geometry ownership test is
`validate_collision_analysis(loadRobot("initial_trial"))`; that validator does
not target either imported URDF model.
