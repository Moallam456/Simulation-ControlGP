function cfg = ft_lab_config()
%FT_LAB_CONFIG Reproducible synthetic F/T laboratory configuration.
% All values below are illustrative, NOT vendor sensor specifications.
% Six axes: [Fx Fy Fz Tx Ty Tz]; units: [N N N N*m N*m N*m].
cfg.Ts = 0.002;                 % 500 Hz
cfg.duration_s = 4.0;
cfg.seed = 17;
cfg.filter_cutoff_hz = 10;     % Example only; NOT approved for hardware.
cfg.quiet_window_s = [0.20 0.44];
cfg.step_window_s = [0.50 1.50];
cfg.step_amplitude_N = 5.0;

cfg.sensor.bias = [0.10; -0.08; 0.05; 0.008; -0.006; 0.004];
cfg.sensor.noise_std = [0.08; 0.08; 0.06; 0.004; 0.004; 0.003];
cfg.sensor.drift_per_s = [0.005; 0.003; -0.004; 0.0002; 0; 0];
cfg.sensor.quantization_step = [0.01; 0.01; 0.01; 0.001; 0.001; 0.001];
cfg.sensor.saturation = [100; 100; 100; 10; 10; 10];

% Demonstrate within-force and within-torque cross-axis coupling only.
% Entries mixing forces and torques require dimensioned calibration.
cfg.sensor.coupling = eye(6);
cfg.sensor.coupling(2,1) = 0.015;
cfg.sensor.coupling(5,4) = 0.020;
end
