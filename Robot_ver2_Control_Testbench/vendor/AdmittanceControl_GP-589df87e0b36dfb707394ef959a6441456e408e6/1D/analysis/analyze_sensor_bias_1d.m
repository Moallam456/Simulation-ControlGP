%% Single-Axis Admittance: Sensor Bias and Position Drift

clearvars;
clc;
close all;

%% Illustrative parameters
m = 10;              % Virtual mass [kg]
b = 100;             % Virtual damping [N*s/m]

F_bias = 0.2;        % Constant sensor bias [N]
Ts = 0.002;          % Output sampling interval [s]

t = (0:Ts:10)';

tau = m/b;

%% Continuous-time transfer functions

Gv = admittance_1d_continuous(m, b);

% Position is the integral of velocity
Gx = Gv * tf(1, [1 0]);

%% Pole analysis

disp('Velocity poles:');
disp(pole(Gv));

disp('Position poles:');
disp(pole(Gx));

%% MATLAB responses

v_matlab = step(F_bias * Gv, t);
x_matlab = step(F_bias * Gx, t);

%% Analytical solutions

v_analytical = (F_bias/b) * ...
    (1 - exp(-t/tau));

x_analytical = (F_bias/b) * ...
    (t - tau*(1 - exp(-t/tau)));

%% Verification

velocity_error = max(abs(v_matlab - v_analytical));
position_error = max(abs(x_matlab - x_analytical));

fprintf('Final velocity: %.4f mm/s\n', ...
    v_matlab(end)*1000);

fprintf('Displacement after 10 s: %.3f mm\n', ...
    x_matlab(end)*1000);

fprintf('Maximum velocity error: %.3e m/s\n', ...
    velocity_error);

fprintf('Maximum position error: %.3e m\n', ...
    position_error);

assert(velocity_error < 1e-9);
assert(position_error < 1e-9);

%% Visualization

figure;
tiledlayout(2,1);

nexttile;
plot(t, v_matlab*1000, 'b', 'LineWidth', 1.5);
ylabel('Velocity [mm/s]');
title('Commanded Velocity Due to Sensor Bias');
grid on;

nexttile;
plot(t, x_matlab*1000, 'r', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Displacement [mm]');
title('Accumulated Position Drift Due to Sensor Bias');
grid on;