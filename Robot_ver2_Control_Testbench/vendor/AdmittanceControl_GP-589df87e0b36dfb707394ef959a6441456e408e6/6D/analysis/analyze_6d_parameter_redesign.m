
%% Step 42 - Six-Axis Admittance Parameter Redesign
% Objective:
% Redesign virtual inertia and damping to satisfy
% combined-motion constraints without output saturation.

clearvars;
clc;
close all;

%% 1. Original design requirements (Step 39)

axisName = ["X"; "Y"; "Z"; "Rx"; "Ry"; "Rz"];

w_design = [10; 8; 6; 0.9; 0.7; 0.5];

V_design = [0.10; 0.08; 0.06; ...
            0.15; 0.12; 0.10];

t90_design = [0.30; 0.32; 0.35; ...
              0.28; 0.32; 0.30];

Ts = 0.002;       % 500 Hz
t = 0:Ts:1.2;
N = numel(t);

%% 2. Original virtual inertia and damping

B_original = w_design ./ V_design;

M_original = B_original .* t90_design / log(10);

%% 3. Redesigned virtual parameters

% Preliminary feasible candidate
% No velocity or acceleration saturation

massScale = 2.0;
dampingScale = 1.4;

M_new = massScale * M_original;
B_new = dampingScale * B_original;

% Calculate the new performance

V_new = w_design ./ B_new;

tau_new = M_new ./ B_new;

t90_new = tau_new * log(10);

poles_new = -B_new ./ M_new;

zTustin = (2*M_new - B_new*Ts) ./ ...
          (2*M_new + B_new*Ts);

%% 4. Display redesigned parameters

parameterResults = table( ...
    axisName, M_original, B_original, ...
    M_new, B_new, V_new, t90_new, ...
    poles_new, zTustin, ...
    'VariableNames', { ...
    'Axis', ...
    'OriginalInertia','OriginalDamping', ...
    'NewInertia','NewDamping', ...
    'NewSteadyVelocity','NewRiseTime', ...
    'ContinuousPole','TustinPole'});

disp('=== REDESIGNED ADMITTANCE PARAMETERS ===');
disp(parameterResults);

assert(all(abs(zTustin) < 1));

%% 5. Existing combined-motion requirements

v_trans_max = 0.13;    % m/s
v_rot_max = 0.24;      % rad/s

a_trans_max = 1.10;    % m/s^2
a_rot_max = 1.80;      % rad/s^2

d_trans_max = 0.022;   % m
d_rot_max = 0.035;     % rad (angular displacement proxy)

%% 6. Analytical design feasibility

% Combined steady speed

vT = norm(w_design(1:3) ./ B_new(1:3));

vR = norm(w_design(4:6) ./ B_new(4:6));

% Conservative acceleration bounds for force reversal
% Assumes each input component remains within +/-w_design
% and the system starts from rest.

aT = 2*norm(w_design(1:3) ./ M_new(1:3));

aR = 2*norm(w_design(4:6) ./ M_new(4:6));

% Ideal remaining displacement after releasing from
% the maximum per-axis steady velocities.

dT = norm( ...
    w_design(1:3).*M_new(1:3)./B_new(1:3).^2);

dR = norm( ...
    w_design(4:6).*M_new(4:6)./B_new(4:6).^2);

fprintf('\n=== ANALYTICAL CONSTRAINT VERIFICATION ===\n');

fprintf('\nTRANSLATION\n');
fprintf('Steady speed: %.5f m/s | Limit: %.5f\n', ...
    vT,v_trans_max);

fprintf('Reversal acceleration bound: %.5f m/s^2 | Limit: %.5f\n', ...
    aT,a_trans_max);

fprintf('Ideal release distance: %.3f mm | Limit: %.3f mm\n', ...
    1000*dT,1000*d_trans_max);

fprintf('\nROTATION\n');
fprintf('Steady angular speed: %.5f rad/s | Limit: %.5f\n', ...
    vR,v_rot_max);

fprintf('Reversal acceleration bound: %.5f rad/s^2 | Limit: %.5f\n', ...
    aR,a_rot_max);

fprintf('Release angular-displacement proxy: %.5f rad | Limit: %.5f\n', ...
    dR,d_rot_max);

assert(vT <= v_trans_max);
assert(vR <= v_rot_max);

assert(aT <= a_trans_max);
assert(aR <= a_rot_max);

assert(dT <= d_trans_max);
assert(dR <= d_rot_max);

disp('Analytical design feasibility PASSED.');

%% 7. Same force-reversal experiment as Step 41A

W = zeros(6,N);

% Positive force
pushIdx = t >= 0.05 & t < 0.50;

W(:,pushIdx) = repmat( ...
    w_design,1,nnz(pushIdx));

% Reverse force
reverseIdx = t >= 0.50 & t < 0.75;

W(:,reverseIdx) = repmat( ...
    -w_design,1,nnz(reverseIdx));

% After 0.75 s, applied wrench is zero.

%% 8. Simulate original and redesigned controllers

% Case 1 = original
% Case 2 = redesigned

M_cases = [M_original, M_new];
B_cases = [B_original, B_new];

V = zeros(6,N,2);
A = zeros(6,N,2);

for c = 1:2

    M = M_cases(:,c);
    B = B_cases(:,c);

    for k = 2:N

        % Six-axis Tustin difference equation
        % No saturation or clipping

        V(:,k,c) = ( ...
            (2*M-B*Ts).*V(:,k-1,c) + ...
            Ts*(W(:,k)+W(:,k-1)) ...
            ) ./ (2*M+B*Ts);

    end

    % Finite-difference commanded acceleration

    A(:,:,c) = [zeros(6,1), ...
        diff(V(:,:,c),1,2)/Ts];

end

%% 9. Calculate resultant Cartesian motion

speedTrans = zeros(2,N);
speedRot = zeros(2,N);

accTrans = zeros(2,N);
accRot = zeros(2,N);

for c = 1:2

    speedTrans(c,:) = vecnorm(V(1:3,:,c),2,1);
    speedRot(c,:) = vecnorm(V(4:6,:,c),2,1);

    accTrans(c,:) = vecnorm(A(1:3,:,c),2,1);
    accRot(c,:) = vecnorm(A(4:6,:,c),2,1);

end

%% 10. Compare maximum simulated values

names = ["Original";"Redesigned"];

fprintf('\n=== FORCE REVERSAL SIMULATION ===\n');

for c = 1:2

    fprintf('\n%s Controller\n',names(c));

    fprintf('Maximum translation speed: %.5f m/s\n', ...
        max(speedTrans(c,:)));

    fprintf('Maximum translation acceleration: %.5f m/s^2\n', ...
        max(accTrans(c,:)));

    fprintf('Maximum rotation speed: %.5f rad/s\n', ...
        max(speedRot(c,:)));

    fprintf('Maximum rotation acceleration: %.5f rad/s^2\n', ...
        max(accRot(c,:)));

end

%% 11. Verify redesigned controller

% Check all six components are finite

assert(all(isfinite(V(:))));
assert(all(isfinite(A(:))));

% Verify simulation constraints for redesigned model

assert(all(speedTrans(2,:) <= v_trans_max+1e-10));

assert(all(speedRot(2,:) <= v_rot_max+1e-10));

assert(all(accTrans(2,:) <= a_trans_max+1e-10));

assert(all(accRot(2,:) <= a_rot_max+1e-10));

% Verify Tustin equation residual

residualMax = 0;

for k = 2:N

    lhs = (2*M_new+B_new*Ts).*V(:,k,2);

    rhs = (2*M_new-B_new*Ts).*V(:,k-1,2) + ...
          Ts*(W(:,k)+W(:,k-1));

    residualMax = max( ...
        residualMax,max(abs(lhs-rhs)));

end

fprintf('\nMaximum Tustin residual: %.3e\n',residualMax);

assert(residualMax < 1e-10);

disp('Redesigned controller simulation PASSED.');

%% 12. Plot original vs redesigned responses

figure;
tiledlayout(2,2);

% Translational speed

nexttile;

plot(t,speedTrans(1,:),'LineWidth',1.5);
hold on;

plot(t,speedTrans(2,:),'--','LineWidth',1.5);

yline(v_trans_max,'r:','Limit');

title('Combined Translational Speed');
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend('Original','Redesigned','Limit');
grid on;

% Rotational speed

nexttile;

plot(t,speedRot(1,:),'LineWidth',1.5);
hold on;

plot(t,speedRot(2,:),'--','LineWidth',1.5);

yline(v_rot_max,'r:','Limit');

title('Combined Rotational Speed');
xlabel('Time [s]');
ylabel('Angular speed [rad/s]');
legend('Original','Redesigned','Limit');
grid on;

% Translational acceleration

nexttile;

plot(t,accTrans(1,:),'LineWidth',1.5);
hold on;

plot(t,accTrans(2,:),'--','LineWidth',1.5);

yline(a_trans_max,'r:','Limit');

title('Combined Translational Acceleration');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
legend('Original','Redesigned','Limit');
grid on;

% Rotational acceleration

nexttile;

plot(t,accRot(1,:),'LineWidth',1.5);
hold on;

plot(t,accRot(2,:),'--','LineWidth',1.5);

yline(a_rot_max,'r:','Limit');

title('Combined Rotational Acceleration');
xlabel('Time [s]');
ylabel('Angular acceleration [rad/s^2]');
legend('Original','Redesigned','Limit');
grid on;

sgtitle('Six-Axis Admittance Parameter Redesign');

%% 13. Final verification

fprintf('\n====================================\n');
fprintf('FINAL DESIGN VERIFICATION\n');
fprintf('====================================\n');

fprintf('Mass scaling: %.2f\n',massScale);
fprintf('Damping scaling: %.2f\n',dampingScale);

fprintf('X-axis original t90: %.3f s\n',t90_design(1));
fprintf('X-axis redesigned t90: %.3f s\n',t90_new(1));

fprintf('\nNo output saturation was applied.\n');
fprintf('Redesigned controller satisfies tested constraints.\n');
fprintf('Physical robot performance remains unverified.\n');
