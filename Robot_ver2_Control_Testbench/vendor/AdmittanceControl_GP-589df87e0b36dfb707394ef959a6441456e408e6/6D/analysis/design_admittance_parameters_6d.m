
%% Step 39 - Six-Axis Admittance Parameter Design
clearvars;
clc;
close all;

%% 1. Axis definitions

axesName = ["X"; "Y"; "Z"; "Rx"; "Ry"; "Rz"];

% Translation: N, m/s, m/s^2, m
% Rotation: N*m, rad/s, rad/s^2, rad

%% 2. Illustrative design requirements
% NOT final robot specifications.

w_design = [10; 8; 6; 0.9; 0.7; 0.5];

V_design = [0.10; 0.08; 0.06; ...
            0.15; 0.12; 0.10];

t90_design = [0.30; 0.32; 0.35; ...
              0.28; 0.32; 0.30];

% Per-axis initial acceleration limits
a_max = [0.85; 0.65; 0.45; ...
         1.30; 0.95; 0.85];

% Per-axis ideal release-travel limits
d_max = [0.016; 0.013; 0.010; ...
         0.020; 0.019; 0.015];

%% 3. Calculate virtual damping and inertia

B = w_design ./ V_design;

M = B .* t90_design / log(10);

%% 4. Predict individual-axis performance

tau = M ./ B;

Vss = w_design ./ B;

a_initial = w_design ./ M;

release_response = Vss .* tau;

poles = -B ./ M;

%% 5. Determine per-axis feasibility

M_min = w_design ./ a_max;

M_max = B .* d_max ./ V_design;

t90_min = V_design * log(10) ./ a_max;

t90_max = d_max * log(10) ./ V_design;

axis_feasible = M_min <= M_max;

design_passes = axis_feasible & ...
    a_initial <= a_max & ...
    release_response <= d_max;

%% 6. Display separate translational/rotational results

T = table( ...
    axesName, w_design, V_design, t90_design, ...
    M, B, a_initial, release_response, ...
    M_min, M_max, axis_feasible, design_passes, ...
    'VariableNames', { ...
    'Axis','Effort','SteadySpeed','RiseTime', ...
    'VirtualInertia','Damping','InitialAcceleration', ...
    'ReleaseResponse','MinimumInertia','MaximumInertia', ...
    'Feasible','DesignPasses'});

disp('=== TRANSLATIONAL AXES: X, Y, Z ===');
disp(T(1:3,:));

disp('=== ROTATIONAL AXES: Rx, Ry, Rz ===');
disp(T(4:6,:));

%% 7. Combined-motion constraints
% These values are illustrative and need physical justification.

limits.translation_speed = 0.13;       % m/s
limits.translation_accel = 1.10;       % m/s^2
limits.translation_release = 0.022;    % m

limits.angular_speed = 0.24;           % rad/s
limits.angular_accel = 1.80;           % rad/s^2

% Preliminary integral of angular velocity [rad].
% Not generally the exact SO(3) orientation change.
limits.angular_release_proxy = 0.035;

%% 8. Combined-motion predictions

% Translation (BASE frame)
v_linear = norm(Vss(1:3));
a_linear = norm(a_initial(1:3));
d_linear = norm(release_response(1:3));

% Rotation (BASE frame)
v_angular = norm(Vss(4:6));
a_angular = norm(a_initial(4:6));

angular_release_proxy = ...
    norm(release_response(4:6));

%% 9. Combined-motion checks

linear_speed_pass = ...
    v_linear <= limits.translation_speed;

linear_accel_pass = ...
    a_linear <= limits.translation_accel;

linear_release_pass = ...
    d_linear <= limits.translation_release;

angular_speed_pass = ...
    v_angular <= limits.angular_speed;

angular_accel_pass = ...
    a_angular <= limits.angular_accel;

angular_release_proxy_pass = ...
    angular_release_proxy <= ...
    limits.angular_release_proxy;

fprintf('\n=== COMBINED TRANSLATIONAL MOTION ===\n');

fprintf('Speed: %.5f m/s | Limit: %.5f | Pass: %d\n', ...
    v_linear, limits.translation_speed, linear_speed_pass);

fprintf('Acceleration: %.5f m/s^2 | Limit: %.5f | Pass: %d\n', ...
    a_linear, limits.translation_accel, linear_accel_pass);

fprintf('Release displacement: %.5f m | Limit: %.5f | Pass: %d\n', ...
    d_linear, limits.translation_release, linear_release_pass);

fprintf('\n=== COMBINED ROTATIONAL MOTION ===\n');

fprintf('Angular speed: %.5f rad/s | Limit: %.5f | Pass: %d\n', ...
    v_angular, limits.angular_speed, angular_speed_pass);

fprintf('Angular acceleration: %.5f rad/s^2 | Limit: %.5f | Pass: %d\n', ...
    a_angular, limits.angular_accel, angular_accel_pass);

fprintf('Angular release proxy: %.5f rad | Limit: %.5f | Pass: %d\n', ...
    angular_release_proxy, ...
    limits.angular_release_proxy, ...
    angular_release_proxy_pass);

%% 10. Verification

% The illustrative per-axis design is feasible.
assert(all(axis_feasible));
assert(all(design_passes));

% The combined translation speed must reveal a violation.
assert(~linear_speed_pass);

% All other illustrative combined checks should pass.
assert(linear_accel_pass && linear_release_pass);
assert(angular_speed_pass && angular_accel_pass);
assert(angular_release_proxy_pass);

fprintf('\nSix-axis feasibility calculations VERIFIED.\n');
fprintf('Full combined-motion design NOT APPROVED.\n');

%% 11. Plot normalized constraint utilization

axisRatios = [ ...
    a_initial ./ a_max, ...
    release_response ./ d_max];

combinedRatios = [ ...
    v_linear / limits.translation_speed, ...
    a_linear / limits.translation_accel, ...
    d_linear / limits.translation_release; ...
    v_angular / limits.angular_speed, ...
    a_angular / limits.angular_accel, ...
    angular_release_proxy / limits.angular_release_proxy];

figure;
tiledlayout(2,1);

nexttile;
bar(1:6,axisRatios);
hold on;
yline(1,'k--','Constraint');
xticks(1:6);
xticklabels(axesName);
ylabel('Utilization ratio');
title('Individual Axis Constraints');
legend('Acceleration','Release response');
grid on;

nexttile;
bar(combinedRatios);
hold on;
yline(1,'k--','Constraint');
xticks([1 2]);
xticklabels({'Translation','Rotation'});
ylabel('Utilization ratio');
title('Combined Motion Constraints');
legend('Speed','Acceleration','Release response');
grid on;

sgtitle('Six-Axis Admittance Design Feasibility');
