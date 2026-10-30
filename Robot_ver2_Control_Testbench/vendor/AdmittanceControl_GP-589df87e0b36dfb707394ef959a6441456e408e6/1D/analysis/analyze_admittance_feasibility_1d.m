
%% Step 38 - Admittance Feasibility Analysis
clearvars;
clc;
close all;

%% 1. Desired operating behavior
F_design = 10;        % Operator force [N]
v_design = 0.10;      % Steady velocity [m/s]
t90_design = 0.30;    % Desired rise time [s]

%% 2. Previously designed parameters
B = F_design / v_design;
M = B*t90_design/log(10);

a_predicted = F_design/M;
d_predicted = v_design*M/B;

%% 3. Illustrative design constraints
scenarioNames = ["Scenario A"; "Scenario B"];

a_max = [0.8; 0.5];       % m/s^2
d_max = [0.015; 0.010];   % m

%% 4. Feasible mass intervals
M_min = F_design ./ a_max;
M_max = B .* d_max ./ v_design;

%% 5. Equivalent rise-time intervals
t90_min = v_design*log(10) ./ a_max;
t90_max = d_max*log(10) ./ v_design;

%% 6. Maximum compatible steady speed
v_feasible_max = sqrt(a_max .* d_max);

%% 7. Determine feasibility
feasible = M_min <= M_max;

designPasses = feasible & ...
    M >= M_min & M <= M_max;

%% 8. Display results
results = table( ...
    scenarioNames, a_max, d_max*1000, ...
    M_min, M_max, ...
    t90_min, t90_max, ...
    v_feasible_max, feasible, designPasses, ...
    'VariableNames', { ...
    'Scenario','AccelerationLimit', ...
    'ReleaseLimit_mm','MassMin_kg', ...
    'MassMax_kg','RiseTimeMin_s', ...
    'RiseTimeMax_s','MaxSpeed_mps', ...
    'Feasible','CurrentDesignPasses'});

disp(results);

fprintf('\nCurrent controller:\n');
fprintf('Virtual mass: %.4f kg\n', M);
fprintf('Virtual damping: %.4f N*s/m\n', B);
fprintf('Initial acceleration: %.4f m/s^2\n', a_predicted);
fprintf('Release distance: %.4f mm\n', d_predicted*1000);

%% 9. Verify feasibility calculations

assert(all(feasible == ...
    (v_design^2 <= a_max.*d_max)));

assert(feasible(1));
assert(~feasible(2));

assert(designPasses(1));
assert(~designPasses(2));

disp('Admittance feasibility analysis PASSED.');

%% 10. Visualize the design tradeoff
t90 = linspace(0.1,0.6,500);

a_curve = v_design*log(10)./t90;
d_curve = v_design*t90/log(10);

figure;
tiledlayout(2,1);

nexttile;
plot(t90,a_curve,'LineWidth',1.5);
hold on;
yline(a_max(1),'--','Scenario A limit');
yline(a_max(2),'--','Scenario B limit');
xline(t90_design,':','Current design');
ylabel('Initial acceleration [m/s^2]');
title('Acceleration Constraint');
grid on;

nexttile;
plot(t90,d_curve*1000,'LineWidth',1.5);
hold on;
yline(d_max(1)*1000,'--','Scenario A limit');
yline(d_max(2)*1000,'--','Scenario B limit');
xline(t90_design,':','Current design');
xlabel('90% rise time [s]');
ylabel('Release distance [mm]');
title('Release-Distance Constraint');
grid on;

sgtitle('Admittance Controller Feasibility');
