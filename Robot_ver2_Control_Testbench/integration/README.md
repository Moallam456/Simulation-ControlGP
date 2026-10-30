# C / CANopen Physics Connection

This is a **simulation-only motor-bank replacement**, not another controller.

```text
Existing C Supervisor / planners / hand-guiding
    -> existing CANopen master and SIL Kit CAN1
    -> existing AVATAR virtual node protocol
    -> physics_motor_bank.cpp (no ideal follower)
    <-> Windows MATLAB robot_ver2_testbench
    -> measured positions returned as AVATAR TPDO4
```

## Prepare A Separate Checkout

From this package folder, run in Windows Python or WSL Python:

```text
python integration/prepare_control.py
```

It prints a new `build/control_integrated` path. It refuses to overwrite it.
Choose `--destination /your/new/path` to create a separate Linux-filesystem
checkout for WSL builds. Review `ROBOT_VER2_INTEGRATION.txt` and the diff against
the vendor snapshot. Original checkouts and source snapshots are untouched.

The prepared checkout retains upstream build dependencies: FreeRTOS, SIL Kit
5.0.7 and raylib. The GitHub source ZIP does not contain Git submodule contents.
Use your team's configured dependency installations/build instructions, or
populate the dependency paths in the separate checkout. No SDK is implicitly
installed and no network connection to physical drives is used.

## Build The Adapter Independently

In WSL/Linux, with SIL Kit installed:

```bash
cmake -S integration -B build/adapter -DRB2_BUILD_CAN_ADAPTER=ON -DSILKIT_ROOT=/path/to/SilKit
cmake --build build/adapter -j4
```

This builds `rb2_physics_motor_bank`. Run it **instead of**, never alongside,
the original motor-bank executable. In a prepared full checkout the original
target name already builds the replacement source, so its launcher can retain
the existing executable name. Set `ROBOT_PHYSICS_IP` to the Windows MATLAB IP;
this is separate from existing `MATLAB_IP` display telemetry.
Set `cfg.adapter.peer` in `testbench_config.m` to the expected WSL/controller
sender IP, then rebuild the MATLAB model. A different sender latches a fault.

Open MATLAB's dashboard, select **External C / CANopen**, then Run. Start the
team's SIL Kit registry/controller/HMI and the replacement motor bank using
their existing configuration. The model initially holds compact home while
waiting; the demo connection timeout is 15 seconds. For long interactive runs
increase `duration_s` in `testbench_config.m`, not the controller's safety gates.

UDP 5006 receives physics commands; UDP 5007 returns physical feedback.
Existing display/status port 5005 and HMI/F/T ports 5010–5013 are unchanged.
Allow only the intended WSL/Windows peers through the firewall. These UDP
packets are unencrypted simulation transport, not a hardware safety protocol.

Private physics v1 uses big-endian words and IEEE754 binary64 values:

- Command, 72 bytes: `RB2C`, version, sequence, enabled, simulation time, q[6].
- Feedback, 120 bytes: `RB2F`, version, echoed sequence, fault, simulation time,
  measured q[6], measured qd[6].

q is joint-side radians. The virtual node still uses upstream encoder/reduction
conversions. For now, 32768 units/motor revolution and 50:1 are a **synthetic
shared fixture**, not approved physical calibration.

No TPDO4 refresh is synthesized from a target. The adapter sends it only after
a newer MATLAB measurement time. Malformed/out-of-order data, MATLAB faults or
physics timeout terminate the adapter; the existing controller must inhibit on
missing/faulted feedback. Restart is explicit. Desktop scheduling can trip real
controller freshness/deadline checks; do not loosen them to claim acceptance.

The upstream protocol simulator normally replies to RPDO4 with its cached last
position. The adapter deliberately defers that TPDO4 until a newer physical
measurement. CAN frame formats are unchanged, but this response-timing change
is essential to avoid presenting repeated stale physics as fresh feedback.
During commissioning, disabled drives use synthetic gravity-hold standby;
after the first enable, a disable latches the simulation stop. Neither standby
nor active braking claims to reproduce a real unpowered motor or brake.

## Applied Forces

The MATLAB laboratory supplies a known wrench at the TCP for physical response
tests. In external mode, the existing C virtual-F/T GUI remains its sensor source;
the dashboard does not automatically compete for port 5013. To compare matching
force experiments, supply the same known wrench/timebase at that existing input
and configure the physical applied wrench accordingly. This is not yet a
vendor serial-sensor/physical-contact hardware-in-loop test.

## Portable Regression Tests

```bash
cmake -S integration -B build/tests
cmake --build build/tests -j4
ctest --test-dir build/tests --output-on-failure
```

Add `-DRB2_PREPARED_CONTROL=/path/to/new/checkout` to compile/test the actual
ControlCore dispatch changes and keep UR5 pipeline regression executables.
The package never requires source files from Final_Architecture.
