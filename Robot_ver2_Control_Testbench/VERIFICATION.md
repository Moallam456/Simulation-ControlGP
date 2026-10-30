# Verification Record

Executed on Windows MATLAB R2024b and Ubuntu 24.04 / GCC 13.3, 2026-10-29.
This is software verification, not hardware commissioning.

## Passed

- 200 random robot_ver2 configurations compared against the packaged URDF:
  maximum C FK matrix error `9.34e-16`, Jacobian error `2.64e-15`.
- 200/200 random TCP poses produced FK-validated C IK branches. Largest position
  residual across accepted branches was `1.287e-6 m`. Singular/incomplete branch
  enumeration limitations remain documented; this is not exhaustive IK proof.
- Portable C test: 1,000 FK-to-IK round trips plus finite-difference Jacobians.
- Original C admittance versus its unsaturated Tustin equation: error below
  `2.78e-17`. Native R2024b Simulink reference: below `5.43e-19`.
- Original C acceleration/speed limiting and timing-fault rejection.
- Compact home passes threshold and geometry checks. Home mass-matrix minimum
  eigenvalue exceeds `0.0017`; imported moving masses match Multibody blocks.
- Finite short-duration Multibody motion with synthetic actuator dynamics.
- Four-second **actual C admittance + mapper / Multibody** run completed without
  a safety fault, including a known applied operator force. Runtime used three
  exact collision evaluations and 3,999 conservative pairwise-bound checks.
- Relocated runtime simulated with developer paths removed. Editing a copied
  STL vertex changed the geometry signature and started a fresh collision cache.
- Collision clearance-bound checks compared against independent uncached
  checks at nearby configurations. Environment obstruction, joint limits,
  linear/full-task guards and invalid home-route rejection were exercised.
- **15/15 CTest checks** passed, including actual robot-specific API dispatch,
  eight existing UR5 trajectory/planner regressions, encoder scaling and
  original admittance/sensor-configuration/AVATAR protocol tests.
- **Actual SIL Kit 5.0.7 network test**: all six AVATAR nodes returned the
  deliberately different measured physical-fixture positions, not requested
  targets. This test used a physics fixture, not MATLAB or the full Supervisor.

## Not Claimed

- Validated motor/gearbox/electrical/brake or vendor F/T sensor behavior.
- Full production CM7/CM4/FreeRTOS-to-MATLAB end-to-end acceptance at 500 Hz.
- Hardware deadlines, certified stopping, passivity, surface-contact physics,
  continuous collision detection or complete workspace/IK enumeration.
- A collision-free route to home from every possible working configuration.

The delivered model uses fixed-step RK4 at 0.5 ms with 2 ms controller updates.
An applied-force check against rigidBodyTree inverse dynamics had a maximum
joint-torque residual below `6.40e-14 N m`. Halving the integration step to
0.25 ms changed the short pulse test's final angles by at most `3.07e-6 rad`.
Desktop full-mesh checking is not a hard-real-time safety implementation. Exact
mesh checks and conservative bounds are both used; lower-bound clearances are
identified as such. The production controller's timing/authorization checks
must not be disabled to turn an unsuccessful live test into a pass.

## Repeat

From this folder in MATLAB:

```matlab
run_testbench_tests
```

For relocation/STL-content invalidation checks:

```matlab
addpath('tests')
validate_relocation
```

Portable and actual CAN bridge test instructions are in `integration/README.md`.
The original repositories and Final_Architecture are unchanged by this package.
