
%% Continuous vs Backward Euler Admittance
clearvars;
clc;
close all;

%% Parameters
m = 10;          % Virtual mass [kg]
b = 100;         % Virtual damping [N*s/m]
Ts = 0.002;      % Sampling period [s]
F0 = 10;         % Applied force [N]

t = (0:Ts:0.5)';
N = length(t);

tau = m / b;
v_ss = F0 / b;

%% 1. Analytical continuous-time response

v_cont = v_ss * (1 - exp(-t/tau));

x_cont = v_ss * ...
    (t - tau*(1 - exp(-t/tau)));

%% 2. Backward Euler simulation

v_BE = zeros(N,1);
x_BE = zeros(N,1);

% Initial states: v(0) = 0, x(0) = 0

for k = 2:N

    [x_BE(k), v_BE(k)] = ...
        admittance_1d_backward_euler( ...
        x_BE(k-1), v_BE(k-1), F0, m, b, Ts);

end

%% 3. Independent discrete analytical check

alpha = m / (m + b*Ts);
cycles = (0:N-1)';

v_BE_exact = v_ss * (1 - alpha.^cycles);

assert(max(abs(v_BE-v_BE_exact)) < 1e-12);

%% 4. Quantify approximation error

velocity_error = max(abs(v_cont-v_BE));
position_error = max(abs(x_cont-x_BE));

fprintf('Maximum velocity error: %.6e m/s\n', ...
    velocity_error);

fprintf('Maximum displacement error: %.6f mm\n', ...
    position_error*1000);

fprintf('Relative peak velocity error: %.3f %%\n', ...
    velocity_error/v_ss*100);

%% 5. Plot comparison

figure;
tiledlayout(2,1);

nexttile;
plot(t, v_cont, 'b-', 'LineWidth', 2);
hold on;
plot(t, v_BE, 'r--', 'LineWidth', 1.5);
ylabel('Velocity [m/s]');
title('Continuous vs Backward Euler: Velocity');
legend('Continuous', 'Backward Euler');
grid on;

nexttile;
plot(t, x_cont*1000, 'b-', 'LineWidth', 2);
hold on;
plot(t, x_BE*1000, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Displacement [mm]');
title('Continuous vs Backward Euler: Displacement');
legend('Continuous', 'Backward Euler');
grid on;
