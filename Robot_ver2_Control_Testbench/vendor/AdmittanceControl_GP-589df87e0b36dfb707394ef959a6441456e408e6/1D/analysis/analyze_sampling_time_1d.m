%% Effect of Sampling Time on Backward Euler Accuracy

clearvars;
clc;
close all;

%% Parameters
m = 10;
b = 100;
F0 = 10;

tau = m/b;
v_ss = F0/b;

% Sampling periods to compare
Ts_values = [0.002, 0.01, 0.02, 0.05];

figure;
hold on;

% Continuous-time reference
t_ref = linspace(0, 0.5, 1000)';
v_ref = v_ss*(1-exp(-t_ref/tau));

plot(t_ref, v_ref, 'k-', 'LineWidth', 2, ...
    'DisplayName', 'Continuous');

%% Compare different sampling periods

for Ts = Ts_values

    t = (0:Ts:0.5)';
    N = length(t);

    v_BE = zeros(N,1);
    x_BE = zeros(N,1);

    for k = 2:N

        [x_BE(k), v_BE(k)] = ...
            admittance_1d_backward_euler( ...
            x_BE(k-1), v_BE(k-1), F0, m, b, Ts);

    end

    % Exact reference at the same sample times
    v_cont = v_ss*(1-exp(-t/tau));

    % Maximum velocity error
    max_error = max(abs(v_BE-v_cont));

    fprintf('Ts = %.0f ms | Error = %.3f %%\n', ...
        Ts*1000, max_error/v_ss*100);

    plot(t, v_BE, 'o-', ...
        'DisplayName', sprintf('Ts = %.0f ms', Ts*1000));

end

xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Effect of Sampling Time on Backward Euler');

legend('Location', 'southeast');
grid on;