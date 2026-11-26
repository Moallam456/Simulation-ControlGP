% M6030G9 parameters in SI units. Run before the model.
% Source: supplied M6030G9 Planetary Actuator datasheet, pages 1-2.
% This is the 1500 rpm M6030G9, not the 2500 rpm M6030HG9.
% The electrical model is an uncalibrated averaged DC equivalent.

M6030G9.gear_ratio = 9;
M6030G9.rotor_inertia = 0.69e-4;       % kg*m^2; catalogue number, unit interpreted as inertia
M6030G9.nominal_dc_voltage = 36;      % V; chosen upper end of listed 24-36 V range
M6030G9.winding_resistance = 0.86;    % ohm; winding convention not stated
M6030G9.winding_inductance = 0.8e-3;  % H; winding convention not stated
M6030G9.rated_output_torque = 5.4;    % N*m; gearbox-output rating
M6030G9.peak_start_stop_output_torque = 9; % N*m; listed peak output torque
M6030G9.listed_output_speed = 166*2*pi/60; % rad/s; catalogue speed-range upper end
M6030G9.rated_motor_current = 3.5;    % A; current convention not stated
M6030G9.encoder_counts_per_revolution = 32768; % motor-end 15-bit single-turn

% Conditional DC-equivalent estimate at the chosen 36 V operating point.
% The 36 V, 3.5 A and 1500 rpm values are treated as one steady motor point.
% This is not a measured Kt/Ke or an identified three-phase motor model.
M6030G9.eq_R = M6030G9.winding_resistance;
M6030G9.eq_L = M6030G9.winding_inductance;
M6030G9.eq_rated_speed = 1500*2*pi/60; % rad/s; motor-side rated speed
M6030G9.Ke_est = (M6030G9.nominal_dc_voltage ...
    - M6030G9.eq_R*M6030G9.rated_motor_current)/M6030G9.eq_rated_speed;
M6030G9.Kt_est = M6030G9.Ke_est;      % DC-equivalent SI assumption

% Initial controller tuning copied from the M8010 model design rule.
% These are simulation gains, not measured settings in the integrated drive.
M6030G9.current_tau_design = 1e-3;   % s; chosen target
M6030G9.current_P_design = M6030G9.eq_L/M6030G9.current_tau_design;
M6030G9.current_I_design = M6030G9.eq_R/M6030G9.current_tau_design;
M6030G9.eq_voltage_limit = M6030G9.nominal_dc_voltage; % assumed +/-36 V limit
M6030G9.Ke_comp_est = M6030G9.Ke_est; % nominal back-EMF compensation

% The planetary gearbox is rigid in the present models. Its finite
% flexibility may matter physically, but stiffness and damping are not
% specified. No harmonic-drive stiffness value has been transferred here.
