# Hardware Data Needed

Enter supported values in `testbench_config.m`, with source and verification
status. Repeated motors/drives may share documents, but identify each joint.

- **J1–J6 motor models/datasheets:** continuous/peak torque and current, rated/
  maximum speed, torque constant, rotor inertia, resistance and inductance.
- **Drive models/manuals:** configured operating mode, control-loop parameters,
  current/torque/speed limits, following-error limits and fault/stop behavior.
- **Gearbox models/ratios:** efficiency, torque/load ratings, inertia, backlash,
  stiffness and any additional belt/pulley reduction.
- **Encoder/calibration settings:** effective position units, electronic gearing,
  installed joint directions and zero offsets. Specify motor-side versus
  joint-side scaling so reduction is not applied twice.
- **Verified joint travel, speed and acceleration limits.** Current +/-180-degree
  position limits and demo speed values are not commissioned physical limits.
- **Brakes:** fitted axes, holding torque, engagement/release times and behavior
  on drive disable, E-stop and power loss.
- **F/T sensor model/manual:** range, resolution, noise, latency, sampling rate,
  calibration and actual communication format.
- **Mounting measurements:** flange-to-sensor and sensor-to-grip transforms;
  masses/inertias and which motor, gearbox, sensor and tool masses CAD already
  includes. Do not double-count these when refining the plant.
- **Actual workcell:** mounting arrangement, table/workpiece dimensions and
  poses, payload and intended applied/process forces.
- **Measured hardware response:** tracking/settling, friction, backlash,
  saturation and stopping response to identify and validate physical models.

Until these are provided and models validated, actuator/sensor results are
demonstration results and hardware suitability remains NOT EVALUATED.
