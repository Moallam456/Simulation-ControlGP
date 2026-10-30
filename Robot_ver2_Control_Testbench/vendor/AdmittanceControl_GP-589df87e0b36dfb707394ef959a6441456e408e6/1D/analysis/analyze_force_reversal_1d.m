%% Single-Axis Admittance: Force Reversal

clearvars;
clc;
close all;

%% Parameters
m = 10;                 % Virtual mass [kg]
b = 100;                % Virtual damping [N*s/m]
F0 = 10;                % Force magnitude [N]
t_reverse = 0.5;        % Reversal time [s]

t = (0:0.002:1.2)';

tau = m/b;

%% Continuous-time model
Gv = admittance_1d_continuous(m, b);

%% Applied force
F = F0 * ones(size(t));

idx = t >= t_reverse;
F(idx) = -F0;

%% Velocity response using superposition
% +F0 initially, then a -2*F0 step at reversal.

v = F0 * step(Gv, t);

v(idx) = v(idx) - 2*F0 * step( ...
    Gv, t(idx) - t_reverse);

%% Accumulated displacement
x = cumtrapz(t, v);

%% Analytical zero-velocity time
v_reverse = (F0/b)*(1-exp(-t_reverse/tau));

t_zero = t_reverse + ...
    tau*log(1 + b*v_reverse/F0);

fprintf('Velocity at reversal: %.5f m/s\n', v_reverse);
fprintf('Zero-velocity time: %.4f s\n', t_zero);
fprintf('Reversal delay: %.2f ms\n', ...
    (t_zero-t_reverse)*1000);

%% Plots
figure;
tiledlayout(3,1);

nexttile;
plot(t, F, 'LineWidth', 1.5);
xline(t_reverse, 'k--');
ylabel('Force [N]');
title('Applied Force');
grid on;

nexttile;
plot(t, v, 'LineWidth', 1.5);
hold on;
yline(0, 'k:');
xline(t_reverse, 'k--');
xline(t_zero, 'r--');
ylabel('Velocity [m/s]');
title('Commanded Velocity');
grid on;

nexttile;
plot(t, x*1000, 'LineWidth', 1.5);
hold on;
xline(t_reverse, 'k--');
xline(t_zero, 'r--');
xlabel('Time [s]');
ylabel('Displacement [mm]');
title('Accumulated Commanded Displacement');
grid on;