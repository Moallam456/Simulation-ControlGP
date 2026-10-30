
%% Step 41A - Six-Axis Force Reversal and Constraint Analysis
clearvars;
clc;
close all;

%% 1. Existing six-axis design

axisName = ["X"; "Y"; "Z"; "Rx"; "Ry"; "Rz"];

w_design = [10; 8; 6; 0.9; 0.7; 0.5];

V_design = [0.10; 0.08; 0.06; ...
            0.15; 0.12; 0.10];

t90_design = [0.30; 0.32; 0.35; ...
              0.28; 0.32; 0.30];

B = w_design ./ V_design;
M = B .* t90_design / log(10);

Ts = 0.002;
t = 0:Ts:1.2;
N = numel(t);

%% 2. Existing illustrative combined-motion limits

v_trans_max = 0.13;   % m/s
v_rot_max = 0.24;     % rad/s

a_trans_max = 1.10;   % m/s^2
a_rot_max = 1.80;     % rad/s^2

%% 3. Construct force reversal experiment

W = zeros(6,N);

% Initial push: 0.05 to 0.50 s
pushIdx = t >= 0.05 & t < 0.50;

W(:,pushIdx) = repmat( ...
    w_design,1,nnz(pushIdx));

% Reverse force: 0.50 to 0.75 s
reverseIdx = t >= 0.50 & t < 0.75;

W(:,reverseIdx) = repmat( ...
    -w_design,1,nnz(reverseIdx));

% After 0.75 s the applied wrench is zero.

%% 4. Six-axis Tustin controller

V = zeros(6,N);

for k = 2:N

    V(:,k) = ( ...
        (2*M - B*Ts).*V(:,k-1) + ...
        Ts*(W(:,k) + W(:,k-1)) ...
        ) ./ (2*M + B*Ts);

end

%% 5. Compute resultant Cartesian velocities

v_trans = vecnorm(V(1:3,:),2,1);
v_rot = vecnorm(V(4:6,:),2,1);

%% 6. Compute resultant commanded accelerations

A = [zeros(6,1), diff(V,1,2)/Ts];

a_trans = vecnorm(A(1:3,:),2,1);
a_rot = vecnorm(A(4:6,:),2,1);

%% 7. Check constraints

speedTransPass = all(v_trans <= v_trans_max + 1e-10);
speedRotPass = all(v_rot <= v_rot_max + 1e-10);

accTransPass = all(a_trans <= a_trans_max + 1e-10);
accRotPass = all(a_rot <= a_rot_max + 1e-10);

fprintf('\n=== COMBINED MOTION CONSTRAINT RESULTS ===\n');

fprintf('Translation speed: %.5f m/s | Limit: %.5f | Pass: %d\n', ...
    max(v_trans),v_trans_max,speedTransPass);

fprintf('Rotation speed: %.5f rad/s | Limit: %.5f | Pass: %d\n', ...
    max(v_rot),v_rot_max,speedRotPass);

fprintf('Translation acceleration: %.5f m/s^2 | Limit: %.5f | Pass: %d\n', ...
    max(a_trans),a_trans_max,accTransPass);

fprintf('Rotation acceleration: %.5f rad/s^2 | Limit: %.5f | Pass: %d\n', ...
    max(a_rot),a_rot_max,accRotPass);

assert(all(isfinite(V(:))));
assert(all(isfinite(A(:))));

%% 8. Visualize the experiment

figure;
tiledlayout(2,2);

nexttile;
plot(t,W(1,:),'LineWidth',1.5);
xline(0.50,'k--');
xline(0.75,'k--');
title('Applied X-Axis Force');
xlabel('Time [s]');
ylabel('Force [N]');
grid on;

nexttile;
plot(t,V(1:3,:)'*1000,'LineWidth',1.5);
hold on;
yline(0,'k:');
title('Translational Velocity Response');
xlabel('Time [s]');
ylabel('Velocity [mm/s]');
legend('Vx','Vy','Vz');
grid on;

nexttile;
plot(t,v_trans,'LineWidth',1.5);
hold on;
yline(v_trans_max,'r--','Speed limit');
title('Combined Translational Speed');
xlabel('Time [s]');
ylabel('Speed [m/s]');
grid on;

nexttile;
plot(t,a_trans,'LineWidth',1.5);
hold on;
yline(a_trans_max,'r--','Acceleration limit');
title('Combined Translational Acceleration');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
grid on;

sgtitle('Six-Axis Force Reversal and Constraints');

disp('Force reversal and constraint evaluation COMPLETED.');
