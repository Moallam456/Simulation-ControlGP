
%% Step 32 - Six-Axis Continuous vs Tustin
clearvars;
clc;
close all;

%% 1. Illustrative parameters
Ts = 0.002;

M = [10; 12; 8; 0.5; 0.6; 0.4];
B = [100; 90; 120; 5; 4; 6];

% Forces [N], torques [N*m]
wrench = [10; -6; 4; 1; -0.5; 0.2];

t = (0:Ts:0.8)';
N = length(t);

%% 2. Continuous-time reference
sys6 = admittance_6d_continuous(M, B);

U = repmat(wrench.', N, 1);

V_cont = lsim(sys6, U, t);

%% 3. Simulate six-axis Tustin
V_T = zeros(N, 6);

for k = 2:N

    Vnext = admittance_6d_tustin( ...
        V_T(k-1,:).', ...
        wrench, wrench, M, B, Ts);

    V_T(k,:) = Vnext.';

end

%% 4. Analytical verification of continuous model
tau = M ./ B;
Vss = wrench ./ B;

V_analytical = ...
    (1-exp(-t ./ tau.')) .* Vss.';

assert(max(abs(V_cont-V_analytical), [], 'all') < 1e-10);

%% 5. Numerical errors
error = V_T - V_cont;

maxError = max(abs(error), [], 1);

% Normalize by each axis's steady-state speed magnitude
relativeError = 100 * maxError ./ abs(Vss.');

axisName = ["X";"Y";"Z";"Rx";"Ry";"Rz"];
unit = ["m/s";"m/s";"m/s";"rad/s";"rad/s";"rad/s"];

results = table(axisName, maxError.', unit, relativeError.', ...
    'VariableNames', ...
    {'Axis','MaxError_SI','Unit','RelativeError_percent'});

disp(results);

%% 6. Plot responses
figure;
tiledlayout(2,2);

% Translation responses
nexttile;
plot(t, V_cont(:,1:3)*1000, 'LineWidth', 1.7);
hold on;
plot(t, V_T(:,1:3)*1000, '--', 'LineWidth', 1);

title('Translational Velocity');
xlabel('Time [s]');
ylabel('Velocity [mm/s]');
legend('X cont','Y cont','Z cont', ...
       'X Tustin','Y Tustin','Z Tustin');
grid on;

% Rotation responses
nexttile;
plot(t, V_cont(:,4:6), 'LineWidth', 1.7);
hold on;
plot(t, V_T(:,4:6), '--', 'LineWidth', 1);

title('Rotational Velocity');
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
legend('Rx cont','Ry cont','Rz cont', ...
       'Rx Tustin','Ry Tustin','Rz Tustin');
grid on;

% Translation errors
nexttile;
plot(t, error(:,1:3)*1000, 'LineWidth', 1.3);

title('Translation Error');
xlabel('Time [s]');
ylabel('Error [mm/s]');
legend('X','Y','Z');
grid on;

% Rotation errors
nexttile;
plot(t, error(:,4:6), 'LineWidth', 1.3);

title('Rotation Error');
xlabel('Time [s]');
ylabel('Error [rad/s]');
legend('Rx','Ry','Rz');
grid on;

sgtitle('Six-Axis Continuous vs Tustin - 500 Hz');

%% 7. Regression check
% Illustrative numerical-accuracy criterion only
assert(all(relativeError < 0.01));

disp('Six-axis Tustin comparison PASSED.');
