%% Single-Axis Admittance: Pole Analysis

clearvars;
clc;
close all;

%% Illustrative parameters
m = 10;     % Virtual mass [kg]
b = 100;    % Virtual damping [N*s/m]

%% Continuous-time model
Gv = admittance_1d_continuous(m, b);

%% Pole analysis
p = pole(Gv);

%% Time constant
tau = m / b;

fprintf('Pole: %.4f 1/s\n', p);
fprintf('Time constant: %.4f s\n', tau);

%% Verify the relationship
assert(abs(tau + 1/p) < 1e-10);

%% Force-Step Response Analysis

F0 = 10;                  % Applied force [N]
t = (0:0.002:0.5)';       % Simulation time [s]

% Theoretical steady-state velocity
v_ss = F0 / b;

% Theoretical initial acceleration
a0 = F0 / m;

% Analytical velocity response
v_analytical = v_ss * (1 - exp(-t/tau));

% MATLAB transfer-function response
v_matlab = step(F0 * Gv, t);

%% Numerical verification
maxError = max(abs(v_matlab(:) - v_analytical(:)));

fprintf('\nForce-step analysis:\n');
fprintf('Steady-state velocity: %.4f m/s\n', v_ss);
fprintf('Initial acceleration: %.4f m/s^2\n', a0);
fprintf('Maximum model error: %.3e m/s\n', maxError);

assert(maxError < 1e-9, 'Response verification failed');

%% Plot comparison
figure;

plot(t, v_analytical, 'b-', 'LineWidth', 2);
hold on;
plot(t, v_matlab, 'r--', 'LineWidth', 1.5);

yline(v_ss, 'k:', 'Steady-state velocity');

xlabel('Time [s]');
ylabel('Commanded velocity [m/s]');
title('Single-Axis Admittance — Force-Step Response');

legend('Analytical', 'MATLAB', 'Steady state', ...
       'Location', 'southeast');

grid on;

%% Effect of Virtual Mass

m_values = [5, 10, 20];   % Virtual mass [kg]
b_fixed = 100;            % Damping [N*s/m]
F0 = 10;                  % Applied force [N]

t = (0:0.002:1)';

figure;
hold on;

for i = 1:length(m_values)

    m_current = m_values(i);

    G = admittance_1d_continuous(m_current, b_fixed);

    v = step(F0 * G, t);

    plot(t, v, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('m = %g kg', m_current));

end

yline(F0/b_fixed, 'k--', 'Steady-state velocity');

xlabel('Time [s]');
ylabel('Commanded velocity [m/s]');
title('Effect of Virtual Mass on Admittance Response');

legend('Location', 'southeast');
grid on;


%% Effect of Virtual Damping

b_values = [50, 100, 200];  % Damping [N*s/m]
m_fixed = 10;               % Virtual mass [kg]
F0 = 10;                    % Applied force [N]

t = (0:0.002:1)';

figure;
hold on;

for i = 1:length(b_values)

    b_current = b_values(i);

    G = admittance_1d_continuous(m_fixed, b_current);

    v = step(F0 * G, t);

    plot(t, v, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('b = %g N*s/m', b_current));

end

xlabel('Time [s]');
ylabel('Commanded velocity [m/s]');
title('Effect of Virtual Damping on Admittance Response');

legend('Location', 'best');
grid on;

%% Force Release and Stopping Distance

m = 10;             % Virtual mass [kg]
b = 100;            % Virtual damping [N*s/m]
v0 = 0.1;           % Velocity at release [m/s]

tau = m / b;

t = (0:0.002:0.7)';

%% Analytical solution

v_analytical = v0 * exp(-t/tau);

x_analytical = v0 * tau * (1 - exp(-t/tau));

%% Independent numerical solution

opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-12);

[t_num, v_num] = ode45( ...
    @(t,v) -(b/m)*v, t, v0, opts);

x_num = cumtrapz(t_num, v_num);

%% Stopping distance

d_stop = v0 * tau;

fprintf('\nForce-release analysis:\n');
fprintf('Initial velocity: %.4f m/s\n', v0);
fprintf('Time constant: %.4f s\n', tau);
fprintf('Theoretical stopping distance: %.2f mm\n', ...
    d_stop * 1000);

%% Plot results

figure;
tiledlayout(2,1);

nexttile;
plot(t, v_analytical, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_num, v_num, 'r--');
ylabel('Velocity [m/s]');
title('Velocity After Force Release');
legend('Analytical', 'ODE45');
grid on;

nexttile;
plot(t, x_analytical * 1000, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_num, x_num * 1000, 'r--');
yline(d_stop * 1000, 'k:', 'Stopping distance');
xlabel('Time After Release [s]');
ylabel('Displacement [mm]');
title('Displacement After Force Release');
legend('Analytical', 'Numerical', 'Asymptotic limit');
grid on;

%% Single-Axis Admittance: Force Pulse Analysis

clearvars;
clc;
close all;

%% Controller parameters
m = 10;                 % Virtual mass [kg]
b = 100;                % Virtual damping [N*s/m]

F0 = 10;                % Applied force [N]
t_release = 0.5;        % Force release time [s]

t = (0:0.002:1.3)';

%% Continuous-time model
Gv = admittance_1d_continuous(m, b);

tau = m / b;

%% Force input
F = F0 * double(t < t_release);

%% Velocity response
% A force pulse is equivalent to:
% One positive step at t = 0,
% followed by a negative step at t = t_release.

v = F0 * step(Gv, t);

idx = t >= t_release;

v(idx) = v(idx) - F0 * step( ...
    Gv, t(idx) - t_release);

%% Displacement
x = cumtrapz(t, v);

%% Analytical predictions
v_release = (F0/b) * (1 - exp(-t_release/tau));

x_before = (F0/b) * ...
    (t_release - tau*(1-exp(-t_release/tau)));

x_after = v_release * tau;

x_final = x_before + x_after;

fprintf('Velocity at release: %.5f m/s\n', v_release);
fprintf('Displacement before release: %.3f mm\n', x_before*1000);
fprintf('Additional travel after release: %.3f mm\n', x_after*1000);
fprintf('Final predicted displacement: %.3f mm\n', x_final*1000);

%% Plot
figure;
tiledlayout(3,1);

nexttile;
plot(t, F, 'LineWidth', 1.5);
xline(t_release, 'k--');
ylabel('Force [N]');
title('Applied Force');
grid on;

nexttile;
plot(t, v, 'LineWidth', 1.5);
xline(t_release, 'k--');
ylabel('Velocity [m/s]');
title('Commanded Velocity');
grid on;

nexttile;
plot(t, x*1000, 'LineWidth', 1.5);
hold on;
xline(t_release, 'k--');
yline(x_final*1000, 'k:', 'Final displacement');
xlabel('Time [s]');
ylabel('Displacement [mm]');
title('Accumulated Commanded Displacement');
grid on;