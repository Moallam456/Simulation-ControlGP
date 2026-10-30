%% Step 30 - Six-Axis Continuous Admittance Response
clearvars;
clc;
close all;

%% 1. Illustrative controller parameters
M = [10; 12; 8; 0.5; 0.6; 0.4];
B = [100; 90; 120; 5; 4; 6];

%% 2. Applied wrench
% Forces [N], followed by torques [N*m]
wrench = [10; -6; 4; 1; -0.5; 0.2];

%% 3. Time vector
t = (0:0.002:0.8)';

%% 4. Continuous-time model
sys6 = admittance_6d_continuous(M, B);

%% 5. MATLAB state-space response
U = repmat(wrench.', length(t), 1);

V_matlab = lsim(sys6, U, t);

%% 6. Independent analytical solution
tau = M ./ B;
Vss = wrench ./ B;

V_analytical = ...
    (1 - exp(-t ./ tau.')) .* Vss.';

%% 7. Numerical verification
error = max(abs(V_matlab - V_analytical), [], 1);

disp('Maximum error per axis:');
disp(error);

assert(all(error < 1e-10));

%% 8. Plot responses
figure;
tiledlayout(2,1);

% Translational velocities
nexttile;
plot(t, V_matlab(:,1:3)*1000, 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Linear velocity [mm/s]');
title('Translational Admittance Response');
legend('Vx', 'Vy', 'Vz');
grid on;

% Rotational velocities
nexttile;
plot(t, V_matlab(:,4:6), 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
title('Rotational Admittance Response');
legend('Wx', 'Wy', 'Wz');
grid on;