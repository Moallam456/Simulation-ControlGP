% M8025(B)E25B80L parameters in SI units. Run before the model.
% Source: supplied AVATAR M catalogue, M8025 table, pages 9-10.
% The electrical model is an uncalibrated averaged DC equivalent.

M8025.gear_ratio = 81;
M8025.rotor_inertia = 1.74e-4;         % kg*m^2; catalogue number, unit interpreted as inertia
M8025.nominal_dc_voltage = 36;        % V
M8025.winding_resistance = 0.53;      % ohm; winding convention not stated
M8025.winding_inductance = 0.5e-3;    % H; winding convention not stated
M8025.rated_output_torque = 82;       % N*m; reducer rated torque
M8025.peak_start_stop_output_torque = 178; % N*m; duration not stated
M8025.listed_output_speed = 18*2*pi/60;   % rad/s; catalogue speed-range upper end
M8025.rated_motor_current = 7;        % A; current convention not stated
M8025.encoder_counts_per_revolution = 32768; % 15-bit single-turn; location/scaling to verify

% Conditional DC-equivalent estimate. The listed 36 V, 7 A and 1000 rpm
% are treated as one steady motor operating point. This is not measured Kt/Ke.
M8025.eq_R = M8025.winding_resistance;
M8025.eq_L = M8025.winding_inductance;
M8025.eq_rated_speed = 1000*2*pi/60; % rad/s; motor-side rated speed
M8025.Ke_est = (M8025.nominal_dc_voltage ...
    - M8025.eq_R*M8025.rated_motor_current)/M8025.eq_rated_speed;
M8025.Kt_est = M8025.Ke_est;          % DC-equivalent SI assumption

% Initial controller tuning copied from the M8010 model design rule.
% These are simulation gains, not measured settings in the integrated drive.
M8025.current_tau_design = 1e-3;     % s; chosen target
M8025.current_P_design = M8025.eq_L/M8025.current_tau_design;
M8025.current_I_design = M8025.eq_R/M8025.current_tau_design;
M8025.eq_voltage_limit = M8025.nominal_dc_voltage; % assumed +/-36 V limit
M8025.Ke_comp_est = M8025.Ke_est;    % nominal back-EMF compensation

% Harmonic reducer flexibility: copied M8010 trial scenario, not M8025 data.
% Obtain actuator-specific torque/twist data before using it predictively.
M8025.gear_stiffness_trial = 1e4;     % N*m/rad; initial trial only
M8025.gear_damping_trial = 2;         % N*m*s/rad; initial trial only
