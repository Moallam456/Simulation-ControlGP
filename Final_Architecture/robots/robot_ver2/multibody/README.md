# robot_ver2 Multibody Control Bench

## Open and Run

1. Open MATLAB. Set **Current Folder** to the repository folder containing
   `Final_Architecture`.
2. Run this once per MATLAB session:

   ```matlab
   addpath(genpath(fullfile(pwd,'Final_Architecture')))
   ```

3. Run the no-argument command:

   ```matlab
   open_robot_ver2_multibody
   ```

4. In Simulink, press **Ctrl+D** (Update Diagram). The 3D **Mechanics Explorer**
   opens in MATLAB R2024b. Press the green **Run** button in Simulink.
5. The included eight-second demonstration moves all six joints. Replay the
   recorded motion with the playback controls in Mechanics Explorer.
6. Double-click **RobotPlant** to see the actual links, joints, World frame,
   torque converters and TCP sensor. Double-click **Measurements**, then
   **JointSignals**, for joint angle, speed and applied-torque plots.

Requires MATLAB, Simulink, Simscape, Simscape Multibody and Robotics System
Toolbox. The delivered SLX was generated for R2024b. It can also be opened
directly from this folder; the helper command is the convenient entry point.
The opener builds it if missing, but does not overwrite an existing model.

## Control Interface

**RobotPlant** is a torque-driven physical plant, not prescribed joint motion.
Its single Simulink input is a six-element joint-output torque vector in
**N m**, ordered **Joint_1 through Joint_6**. Positive directions match the
URDF and the existing `loadRobot("robot_ver2")` interface.

| Plant output | Meaning |
| --- | --- |
| `q_rad` | Six measured joint angles, radians |
| `qd_rad_s` | Six measured joint speeds, rad/s |
| `qdd_rad_s2` | Six measured joint accelerations, rad/s^2 |
| `tcp_position_m` | Measured torch-tip position in the robot base/World axes, metres |
| `tcp_rotation` | 3-by-3 base-to-TCP orientation matrix, same convention as FK |

The fixed base is attached to World. Gravity is `[0 0 -9.81]` m/s^2.
The TCP is the existing zero-mass fixed frame, offset from Joint 6 by
`[-22.87, 20.53, 245.77]` mm, with parallel axes. The torch mesh and mass remain
on `End_Effector`; they are not added again at the TCP.

**DemoController** is a removable demonstration: position/speed PD feedback
plus a constant gravity-holding torque calculated at home. It is not a
controller deployment design or tuned motor controller. Replace it with the
control team's controller, supplying the same six joint-side torque inputs.
Motor-side torque requires the actual gearbox model and conventions.

Demo references, gains, initial configuration and holding torques are stored
in the **Model Workspace**, not the MATLAB base workspace. The source settings
for regeneration are at the top of `build_robot_ver2_multibody.m`.
Measurements are logged into the simulation output as `mb_q`, `mb_qd`,
`mb_qdd`, `mb_tau`, `mb_tcp_position`, and `mb_tcp_rotation` timeseries.

## Source and Scope

The builder imports the existing corrected project URDF using
[`smimport`](https://www.mathworks.com/help/sm/ref/smimport.html). It preserves
link transforms, joint-axis directions, meshes and URDF inertials. Visual
solids have zero inertial mass; separate imported inertia blocks carry the
actual mass properties. No CAD/URDF source files or other robot models are changed.

The SLX embeds imported numeric parameters. Editing the URDF does **not**
automatically update those blocks. The opener warns when the URDF changes.
Run `build_robot_ver2_multibody` explicitly to regenerate; an existing saved
SLX is backed up first. Unsaved model edits must be saved/closed before rebuilding.
Regeneration recreates the demo; it does not merge custom controller edits.
Keep this folder beside its existing `../CAD/URDF_Assembly_Robot_ver2/meshes/`
folder when moving/sharing it. Geometry paths are resolved relative to the SLX,
not to an absolute developer-machine path.

This is an **ideal rigid-body joint-torque plant**. Motors, gearboxes, current
loops, saturation ratings, friction, backlash, cables, braking and external
process forces are not supplied implicitly. Joint ranges remain provisional;
the imported joint stops are simulated soft stops, not verified physical stops.
STL visualization does not create self-collision or environment-contact forces.
Do not interpret an animation passing through geometry as collision validation.

## Validation

Run `validate_robot_ver2_multibody`. It checks joint setup, mesh resolution,
mass accounting, gravity holding, TCP FK, forced accelerations and inverse
dynamics against the existing rigidBodyTree, then runs the demo. It also
checks that a relocated SLX uses the relocated meshes. Save any open model
edits first; the tests do not overwrite the delivered SLX.
