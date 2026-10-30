# Teammate Handoff: Changes And Deployment

Original repositories and vendored snapshots are not edited. The preparation
script creates a separate integration checkout and refuses an existing target.
Review that checkout before adopting any changes upstream.

| Change | Simulation build | Deployment |
|---|---|---|
| Add robot_ver2 initializer, FK, Jacobian and closed-form IK | Required | Required, with geometry/calibration approval |
| Select compact home rather than UR5 demo home | Required | Required if team adopts this park/home target; not an encoder-zero reset |
| Set actual encoder directions/offsets/reductions/limits | Synthetic fixture initially | Required before motion authorization |
| Replace virtual motor-bank follower with physics adapter | Required for physical loop | Never deployed |
| Native R2024b reference subsystems/dashboard | Laboratory only | Not deployed directly |
| MATLAB collision/workspace checks | Enforced at simulated actuation | Not automatically present onboard; separate reviewed implementation required |

## Exact Integration Surface

`integration/prepare_control.py` copies the pinned control snapshot into a new
destination, then performs deterministic, inspectable edits:

1. Adds `RobotVer2/` C sources and includes to the existing `control_core` target.
2. Adds robot-ID dispatch inside `control_fk`, `control_jacobian` and the legacy
   `adls_ik` entry point. UR5 behavior remains available for existing regressions.
3. For robot_ver2, the legacy one-solution API selects the nearest FK-validated
   analytical branch. It reports zero iterations, not fictitious ADLS iterations.
   `rb2_ik` separately exposes all found branches. This API name is retained only
   to avoid changing all planner call sites. It is NOT ADLS for robot_ver2.
   ADLS-only damping/history fields are not populated by the analytical shim;
   use the existing Jacobian singularity service for singularity diagnostics.
4. Updates both simulation composition roots (`supervisor_silkit_main.c` and
   `Simulation/Robot/robot_runtime.c`) to select robot_ver2 and its compact home.
5. Replaces only the virtual motor-bank executable source with the measured-
   feedback adapter. NMT/SDO/RPDO4/SYNC/TPDO4/heartbeat handling still uses their
   existing AVATAR node and SIL Kit implementations.

No Supervisor safety authorization is fabricated. No STM32 board provider,
commissioning flag, sensor decoder, HAL initialization or storage authorization
is supplied by this package. Hardware builds must explicitly include reviewed
robot-specific sources/configuration; the simulation edits are not a firmware
patch that authorizes motion.

The existing hand-guiding DLS mapper remains a differential velocity mapper;
it is not replaced by pose IK. It receives the matching robot_ver2 Jacobian.
Admittance equations, state-machine behavior and C limiter/reset code remain
unchanged. The Simulink lab does not silently reproduce all those C guards.

The default in-process C test mode links their original admittance and mapper
sources directly as a MEX helper. It replaces only the geometry/configuration
dependency. This is not a complete RTOS/firmware execution, and its synthetic
sensor fixture does not bypass physical sensor commissioning on hardware.

Live desktop CAN/MATLAB timing must be measured. Missing physics updates revoke
feedback, rather than reporting ideal tracking or changing the real controller's
timing limits to make a demonstration pass. If host timing gates trip, report
that run as failed; it is not evidence of physical instability or an STM32 WCET.
