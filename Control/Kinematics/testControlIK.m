clear
clc
close all

%% Load robot configuration

robot = config.UR5();


%% Define a known reachable joint configuration

qTrue = deg2rad([
     30
    -45
     60
     20
    -30
     45
]);


%% Generate a reachable target pose using FK

TTarget = controlFK(robot, qTrue);


%% Choose an initial guess

qSeed = zeros(6,1);


%% Run your IK solver

[qSolution, info] = controlIK(robot, TTarget, qSeed);


%% Calculate the pose achieved by the IK solution

TAchieved = controlFK(robot, qSolution);


%% Calculate position error

positionError = norm( ...
    TTarget(1:3,4) - TAchieved(1:3,4));


%% Calculate orientation error

RTarget = TTarget(1:3,1:3);
RAchieved = TAchieved(1:3,1:3);

RRelative = RTarget * RAchieved';

orientationError = acos( ...
    max(-1, min(1, ...
    (trace(RRelative) - 1) / 2)));


%% Check joint limits

withinLimits = all(qSolution >= robot.limits.qMin) && ...
               all(qSolution <= robot.limits.qMax);


%% Display results

fprintf('\n');
fprintf('=====================================\n');
fprintf('        CONTROL IK TEST RESULTS\n');
fprintf('=====================================\n\n');

fprintf('Converged: %d\n', info.converged);
fprintf('Iterations: %d\n\n', info.iterations);

fprintf('Position error: %.10e m\n', positionError);
fprintf('Orientation error: %.10e rad\n\n', orientationError);

fprintf('Within joint limits: %d\n\n', withinLimits);

fprintf('Original qTrue [degrees]:\n');
disp(rad2deg(qTrue));

fprintf('IK qSolution [degrees]:\n');
disp(rad2deg(qSolution));