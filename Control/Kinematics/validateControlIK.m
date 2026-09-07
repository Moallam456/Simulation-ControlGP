clear
clc
close all

%% Load robot

robot = config.UR5();


%% Define reachable test configurations

qTests = deg2rad([

     30   -45    60    20   -30    45
    -30   -60    80   -20    40    60
     60   -30    40    90   -45   -30
    -60   -75    70    30    45   -60
     15   -90    90     0    30    90

]);


%% Solver initial seed

qSeed = zeros(robot.dof,1);


%% Number of tests

numberOfTests = size(qTests,1);


%% Store results

results(numberOfTests) = struct( ...
    'testNumber', [], ...
    'converged', [], ...
    'iterations', [], ...
    'positionError', [], ...
    'orientationError', [], ...
    'withinLimits', []);


fprintf('\n');
fprintf('============================================\n');
fprintf('       MULTIPLE CONTROL IK VALIDATION\n');
fprintf('============================================\n\n');


%% Run tests

for i = 1:numberOfTests

    fprintf('Running Test %d...\n', i);

    %% Known reachable joint configuration

    qTrue = qTests(i,:).';


    %% Generate target using FK

    TTarget = controlFK(robot, qTrue);


    %% Solve IK

    [qSolution, info] = controlIK( ...
        robot, ...
        TTarget, ...
        qSeed);


    %% Calculate achieved pose

    TAchieved = controlFK(robot, qSolution);


    %% Position error

    positionError = norm( ...
        TTarget(1:3,4) - ...
        TAchieved(1:3,4));


    %% Orientation error

    RTarget = TTarget(1:3,1:3);
    RAchieved = TAchieved(1:3,1:3);

    RRelative = RTarget * RAchieved';

    orientationError = acos( ...
        max(-1, min(1, ...
        (trace(RRelative) - 1)/2)));


    %% Check joint limits

    withinLimits = all( ...
        qSolution >= robot.limits.qMin & ...
        qSolution <= robot.limits.qMax);


    %% Store results

    results(i).testNumber = i;

    results(i).converged = info.converged;

    results(i).iterations = info.iterations;

    results(i).positionError = positionError;

    results(i).orientationError = orientationError;

    results(i).withinLimits = withinLimits;


    %% Print result

    fprintf('Converged: %d\n', info.converged);
    fprintf('Iterations: %d\n', info.iterations);
    fprintf('Position error: %.3e m\n', positionError);
    fprintf('Orientation error: %.3e rad\n', orientationError);
    fprintf('Within limits: %d\n\n', withinLimits);

end


%% Overall summary

fprintf('============================================\n');
fprintf('                 SUMMARY\n');
fprintf('============================================\n\n');

for i = 1:numberOfTests

    fprintf('Test %d: ', i);

    if results(i).converged && results(i).withinLimits
        fprintf('PASS\n');
    else
        fprintf('FAIL\n');
    end

end