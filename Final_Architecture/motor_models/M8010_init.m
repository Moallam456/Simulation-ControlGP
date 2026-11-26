% M8010 parameters in SI units. Run before opening/running the model.
% Source: user-supplied AVATAR M catalogue, M8010 table, page 6.
% Ratio 81 is provisional: the older public catalogue lists 80.
% Rotor inertia uses kg*m^2 as the interpretation of the printed value.

M8010.gear_ratio = 81;
M8010.rotor_inertia = 6.9e-5;           % kg*m^2, motor-side rotor
M8010.nominal_dc_voltage = 36;          % V
M8010.winding_resistance = 0.86;        % ohm; phase/line convention unresolved
M8010.winding_inductance = 0.8e-3;       % H; phase/line and dq conventions unresolved
M8010.rated_output_torque = 35;         % N*m; gearbox-output continuous rating
M8010.peak_start_stop_output_torque = 56; % N*m; permitted duration unknown
M8010.listed_output_speed = 18*2*pi/60; % rad/s
M8010.rated_motor_current = 3.5;        % A; current convention unresolved
M8010.encoder_counts_per_revolution = 32768; % encoder location/scaling to verify

% PROVISIONAL AVERAGED DC-EQUIVALENT ELECTRICAL MODEL
% User authorized estimating the motor constants from the catalogue.
% This is not an identified three-phase PMSM or the actual AVATAR drive.
% Assumptions: the listed 36 V, 3.5 A and 1500 rpm refer to one steady
% equivalent-DC operating point; the winding R/L can be used directly as
% effective armature parameters. Bus/phase/RMS conventions remain unresolved.
M8010.eq_R = M8010.winding_resistance; % assumed DC-equivalent mapping
M8010.eq_L = M8010.winding_inductance; % assumed DC-equivalent mapping
M8010.eq_rated_speed = 1500*2*pi/60;   % rad/s
M8010.Ke_est = (M8010.nominal_dc_voltage ...
    - M8010.eq_R*M8010.rated_motor_current)/M8010.eq_rated_speed;
M8010.Kt_est = M8010.Ke_est; % SI numerical equality for lossless DC conversion
% Result: about 0.21002086 V*s/rad and N*m/A respectively.
% Alternative catalogue torque/current fit: 1/3.5 = 0.285714 N*m/A.
% That alternative predicts E=44.88 V at 1500 rpm, incompatible with the
% assumed 36 V DC-equivalent point. Catalogue entries are not a unique fit.
% Do not interpret the estimate as validated phase, bus or RMS current data.

% PROVISIONAL CURRENT CONTROLLER DESIGN (not an actual drive parameter)
% PI pole cancellation for electrical plant 1/(L*s+R), choosing a 1 ms
% closed-loop time constant. Back-EMF acts as a disturbance; voltage limits
% and electromechanical coupling affect the full response.
M8010.current_tau_design = 1e-3; % s; chosen tuning, not measured latency
M8010.current_P_design = M8010.eq_L/M8010.current_tau_design; % 0.8 V/A
M8010.current_I_design = M8010.eq_R/M8010.current_tau_design; % 860 V/(A*s)
M8010.eq_voltage_limit = M8010.nominal_dc_voltage; % assumed +/-36 V averaged drive

% BACK-EMF FEEDFORWARD CONTROLLER ESTIMATE
% Keep this separate from the simulated motor constant so uncertainty can
% later be studied by changing controller compensation without changing plant.
M8010.Ke_comp_est = M8010.Ke_est;       % V*s/rad; nominal matched case

% PROVISIONAL FLEXIBLE GEARBOX SCENARIO (NOT IDENTIFIED M8010 DATA)
% Linear output-referred stiffness borrowed as a reference from the separate
% Harmonic Drive CSG-17-80-2A-R component (K1 = 1e4 N*m/rad):
% https://www.harmonicdrive.net/products/component-sets/cup-type/csg-2a/csg-17-80-2a-r
% This does not establish the identity of the M8010's internal reducer.
M8010.gear_stiffness_trial = 1e4;       % N*m/rad; linear reference value
M8010.gear_damping_trial = 2;           % N*m*s/rad; chosen trial, no datasheet basis
% For the present 0.05 kg*m^2 bench load, these imply about 75 Hz free
% torsional resonance and a damping ratio near 0.047 before active control.
% Damping stays a fixed assumed component value; it is not a load parameter.
% Add Spring and Damper in parallel between ideal Gear O and load-side shaft.
% Do not bypass the coupling with a wire or connect it to fixed housing.
% Gear friction, hysteresis, nonlinear stiffness, and drive delays remain omitted.
