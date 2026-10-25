# URDF Assembly Robot ver2

Share this entire package folder, not the URDF file alone. The URDF is in
`urdf/URDF_Assembly_Robot_ver2.urdf`; it references seven STL files in
`meshes/` for both visual and collision geometry. The package name in the
`package://` mesh URIs is `URDF_Assembly_Robot_ver2`.

The six revolute joints use provisional position limits of -pi to +pi radians
(-180 to +180 degrees). The URDF's 1000 N m effort and pi rad/s velocity
values are software placeholders so tools can move the joints; they are **not**
motor ratings or verified mechanical limits. Replace them before controller
design or actuator sizing.

Moving-link COM positions and COM inertia tensors were converted from the
Inventor Y-up assembly frame into each URDF link frame at the zero pose.
Masses and joint frames are unchanged from the edited CAD export. Inventor
mass properties were requested at Low accuracy. Check the CAD alignment and
mass properties independently before using dynamics results for hardware
selection. The `urdf/*.csv` file is an older exporter snapshot with different
mass properties and zero-width joint limits; use the URDF, not that CSV, as
the current model source.

The massless fixed `tcp` frame is attached to `End_Effector` (Joint 6).
Its `tcp_fixed` joint origin is `[-0.02287, 0.02053, 0.24577]` metres with
zero roll, pitch, and yaw, from the supplied CAD measurement on 2026-10-15.
Use `tcp` for torch-tip poses and `End_Effector` for flange poses. The torch
mesh and its 1.134 kg mass remain on `End_Effector`; the TCP adds neither
geometry nor mass. This implements the supplied CAD offset, not a physical
tool calibration. Collision meshes are the visual STLs; collision results
depend on each tool's mesh interpretation.

The URDF and mesh paths were tested by importing this folder directly in
MATLAB. The exporter-generated ROS launch files were not tested here; verify
them in your target ROS installation before relying on those launch scripts.
