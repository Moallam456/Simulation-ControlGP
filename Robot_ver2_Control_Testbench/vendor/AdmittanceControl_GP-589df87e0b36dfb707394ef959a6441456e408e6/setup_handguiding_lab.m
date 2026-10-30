
%% HAND-GUIDING LABORATORY — INITIALIZATION
% Shared parameters for F/T Sensor and Admittance Simulink models
% Run this once before opening any experiments/ script.

%% 1. Initialize project
main

%% 2. Controller design
% Calculates optimized six-axis M, D and Ts
design_6axis_admittance

% Tustin coefficients (element-wise)
A6 = (2*M - D*Ts) ./ (2*M + D*Ts);
B6 = Ts ./ (2*M + D*Ts);

%% 3. F/T sensor configuration
cfg = ft_lab_config();

% Ensure sensor and controller have the same sample time
assert(abs(cfg.Ts - Ts) < 1e-12);

Fs = 1/Ts;

%% 4. Generate synthetic six-axis wrench
[t, Wtrue] = ft_generate_wrench(cfg);

ft_wrench_ts = timeseries(Wtrue, t);

%% 5. Sensor noise configuration
noise_mean = zeros(6,1);
noise_variance = cfg.sensor.noise_std.^2;
noise_seed = (cfg.seed + (0:5))';

%% 6. Calibration
% Initially zero; estimate from unloaded recorded data
bias_est = zeros(6,1);

%% 7. EMA low-pass filter
fc = cfg.filter_cutoff_hz;

alpha = 1 - exp(-2*pi*fc/Fs);

%% 8. Butterworth low-pass filter
[bButter, aButter] = butter(2, fc/(Fs/2), 'low');

%% 9. Initialization complete
fprintf('\n=== HAND-GUIDING LAB INITIALIZED ===\n');
fprintf('Sampling frequency: %.0f Hz\n', Fs);
fprintf('Sampling period: %.3f ms\n', Ts*1000);
fprintf('Filter cutoff: %.1f Hz\n', fc);
fprintf('EMA alpha: %.6f\n', alpha);
fprintf('Wrench samples: %d\n', numel(t));
