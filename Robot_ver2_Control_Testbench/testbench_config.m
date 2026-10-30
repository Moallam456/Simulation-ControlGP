function cfg = testbench_config()
% TESTBENCH_CONFIG Authoritative package settings. SI units unless stated.
cfg.robotID = 'robot_ver2';
cfg.mode = 'demonstration'; % 'hardware_validation' refuses unset hardware data.
cfg.controller = 'c_sil'; % 'c_sil', 'laboratory', 'external_can', 'demonstration'.
cfg.sampleTime_s = .002;
cfg.duration_s = 8;
cfg.gravity_m_s2 = [0 0 -9.81];
cfg.home_deg = [0 0 -120 60 131.25 0];
cfg.home_rad = deg2rad(cfg.home_deg);
cfg.provisionalLimits_rad = repmat([-pi pi],6,1);
cfg.minimumLinearInverseCondition = .20;
cfg.minimumSelfClearance_m = .010;
cfg.minimumEnvironmentClearance_m = .010;
cfg.motionCheckStep_rad = deg2rad(.5);
cfg.startupTimeout_s = 15;
cfg.adapter.commandPort = 5006;
cfg.adapter.feedbackPort = 5007;
cfg.adapter.peer = '127.0.0.1'; % Expected controller/WSL sender IP, not MATLAB IP.
cfg.adapter.staleTimeout_s = .15; % DEMO transport tolerance, not safety rating.
cfg.adapter.maxJointStep_rad = deg2rad(2);
cfg.upstream.controlCommit = '15aa22efc071554fe06f1e42e1007633dee15cc9';
cfg.upstream.admittanceCommit = '589df87e0b36dfb707394ef959a6441456e408e6';

% SYNTHETIC actuator fixture. These are NOT the selected robot's motor specs.
cfg.demo.status = 'SYNTHETIC_NOT_HARDWARE';
cfg.demo.reduction = 50*ones(1,6);
cfg.demo.encoderUnitsPerMotorRev = 32768*ones(1,6);
cfg.demo.direction = ones(1,6);
cfg.demo.zeroOffset_units = zeros(1,6);
cfg.demo.kp_joint_Nm_rad = [80 120 80 8 5 2];
cfg.demo.kd_joint_Nm_s_rad = [15 20 12 .8 .5 .2];
cfg.demo.motorTorqueLimit_Nm = [4 5 4 1 .8 .5];
cfg.demo.motorTorqueTimeConstant_s = .002;
cfg.demo.gearEfficiency = .90*ones(1,6);
cfg.demo.viscousFriction_Nm_s_rad = [.1 .1 .1 .02 .02 .01];
cfg.demo.jointSpeedLimit_rad_s = 2*ones(1,6);
cfg.demo.sensor_T_flange = eye(4); % Synthetic identity ONLY; not commissioning.
cfg.demo.virtualMass = [2 2 2 .2 .2 .2];
cfg.demo.virtualDamping = [20 20 20 2 2 2];
cfg.demo.operatorWrench_base_tcp = [1 0 0 0 0 0];
cfg.demo.wrenchActive_s = [1 3];
cfg.demo.admittanceSpeedLimit = [.05 .05 .05 .15 .15 .15];
cfg.demo.admittanceAccelerationLimit = [.2 .2 .2 .5 .5 .5];
cfg.demo.fullJacobianMinimumSigma = 1e-6; % Mixed units; not the linear index.

% UNKNOWN hardware data. Keep NaN/empty until supported by vendor/calibration.
for j = 1:6
    h.motorModel = ''; h.driveModel = ''; h.gearboxModel = '';
    h.ratedMotorTorque_Nm = NaN; h.peakMotorTorque_Nm = NaN;
    h.ratedMotorSpeed_rad_s = NaN; h.maxMotorSpeed_rad_s = NaN;
    h.ratedCurrent_A = NaN; h.peakCurrent_A = NaN;
    h.torqueConstant_Nm_A = NaN; h.resistance_Ohm = NaN;
    h.inductance_H = NaN; h.rotorInertia_kg_m2 = NaN;
    h.reduction = NaN; h.additionalReduction = NaN;
    h.gearEfficiency = NaN; h.gearOutputTorqueLimit_Nm = NaN;
    h.backlash_rad = NaN; h.torsionalStiffness_Nm_rad = NaN;
    h.encoderUnitsPerMotorRev = NaN; h.direction = NaN;
    h.zeroOffset_units = NaN; h.positionLimits_rad = [NaN NaN];
    h.speedLimit_rad_s = NaN; h.accelerationLimit_rad_s2 = NaN;
    h.brakePresent = []; h.brakeHoldingTorque_Nm = NaN;
    h.brakeEngagementTime_s = NaN; h.stopBehavior = '';
    h.driveControlParameters = []; h.source = ''; h.verified = false;
    cfg.hardware.joints(j) = h;
end
cfg.hardware.sensor.model = '';
cfg.hardware.sensor.flange_T_sensor = NaN(4);
cfg.hardware.sensor.sensor_T_grip = NaN(4);
cfg.hardware.sensor.noiseStd = NaN(1,6);
cfg.hardware.sensor.range = NaN(1,6);
cfg.hardware.sensor.latency_s = NaN;
cfg.hardware.sensor.protocol = '';
cfg.hardware.sensor.massAlreadyInCAD = [];
cfg.hardware.sensor.verified = false;
cfg.hardware.actuatorModelValidated = false;
cfg.hardware.massAccountingVerified = false;
cfg.hardware.stopBehaviorValidated = false;
cfg.hardware.source = '';

% Synthetic floor/workpiece. Edit poses/dimensions for the actual workcell.
cfg.environment(1) = struct('name','Floor','size_m',[3 3 .05], ...
    'center_m',[0 0 -.08],'rpy_rad',[0 0 0]);
cfg.environment(2) = struct('name','Workpiece','size_m',[.30 .30 .10], ...
    'center_m',[.85 .65 .35],'rpy_rad',[0 0 0]);
cfg.sensor.bias = zeros(6,1);
cfg.sensor.noise_std = [0.08;0.08;0.06;.004;.004;.003];
cfg.sensor.drift_per_s = [.005;.003;-.004;.0002;0;0];
cfg.sensor.coupling = eye(6);
cfg.sensor.quantization_step = [.01;.01;.01;.001;.001;.001];
cfg.sensor.saturation = [100;100;100;10;10;10];
cfg.sensor.filterCutoff_Hz = 10;
cfg.sensor.seed = 17;
end
