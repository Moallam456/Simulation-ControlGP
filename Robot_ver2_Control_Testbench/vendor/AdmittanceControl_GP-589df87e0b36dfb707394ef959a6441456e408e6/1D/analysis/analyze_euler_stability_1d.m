
%% Forward vs Backward Euler: Numerical Stability

clearvars;
clc;
close all;

%% Parameters
m = 10;
b = 100;

v0 = 0.1;          % Initial velocity [m/s]
F = 0;             % No applied force
tEnd = 2;          % Simulation duration [s]

Ts_values = [0.02, 0.1, 0.2, 0.25];

tau = m/b;

figure;
tiledlayout(2,2);

for i = 1:length(Ts_values)

    Ts = Ts_values(i);

    t = (0:Ts:tEnd)';
    N = length(t);

    %% Initial states
    vFE = zeros(N,1);
    vBE = zeros(N,1);

    xFE = 0;
    xBE = 0;

    vFE(1) = v0;
    vBE(1) = v0;

    %% Simulate both methods
    for k = 2:N

        [xFE, vFE(k)] = admittance_1d_forward_euler( ...
            xFE, vFE(k-1), F, m, b, Ts);

        [xBE, vBE(k)] = admittance_1d_backward_euler( ...
            xBE, vBE(k-1), F, m, b, Ts);

    end

    %% Calculate poles
    zFE = 1 - b*Ts/m;
    zBE = 1 / (1 + b*Ts/m);

    fprintf('Ts = %.0f ms | FE pole = %.3f | BE pole = %.3f\n', ...
        Ts*1000, zFE, zBE);

    %% Exact continuous reference
    tFine = linspace(0,tEnd,1000);
    vExact = v0*exp(-tFine/tau);

    %% Plot
    nexttile;

    plot(tFine, vExact, 'k-', 'LineWidth', 1.5);
    hold on;

    plot(t, vFE, 'o-', 'LineWidth', 1.2);
    plot(t, vBE, 's-', 'LineWidth', 1.2);

    yline(0, 'k:');

    title(sprintf('Ts = %.0f ms', Ts*1000));
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');

    legend('Exact', 'Forward Euler', 'Backward Euler', ...
        'Location', 'best');

    grid on;

end

sgtitle('Numerical Stability of Euler Methods');
