function stressTestADLS()
% validation.stressTestADLS
%
% SIMULATION TEAM - ADLS MULTIPLE-SEED STRESS TEST
%
% PURPOSE
% Test whether the Control team's Adaptive Damped Least-Squares (ADLS)
% numerical IK remains valid when the same reachable TCP target is solved
% from very different initial joint guesses.
%
% The target pose is generated ONLY from the independent Simulation model.
% Every returned IK solution is also verified ONLY with the Simulation model.
%
% This test focuses on:
%   1) Sensitivity to the initial seed
%   2) Ability to find different valid IK branches
%   3) Convergence robustness
%   4) Joint-limit compliance
%   5) Independent TCP pose accuracy
%   6) Singularity avoidance
%   7) Adaptive damping activation
%
% Usage:
%       validation.stressTestADLS
%
% Required:
%   +config/UR5.m
%   ADLS_IK.m
%   controlFK.m
%   controlJacobian.m
%   singularityCheck.m
%   dhTransform.m
%   Robotics System Toolbox

clc;

fprintf('\n');
fprintf('============================================================\n');
fprintf('      SIMULATION TEAM - ADLS MULTIPLE-SEED STRESS TEST\n');
fprintf('============================================================\n\n');

%% 1. Load shared robot configuration

robot = config.UR5();

n = robot.dof;

a           = robot.kinematics.a(:);
d           = robot.kinematics.d(:);
alpha       = robot.kinematics.alpha(:);
thetaOffset = robot.kinematics.thetaOffset(:);

qMin = robot.limits.qMin(:);
qMax = robot.limits.qMax(:);

assert(n == 6, 'Expected a 6-DOF robot.');
assert(numel(qMin) == n && numel(qMax) == n, ...
    'Joint limits must contain one value per joint.');

fprintf('[1] Shared robot configuration loaded: PASS\n');

%% 2. Build independent Simulation rigidBodyTree

simRobot = rigidBodyTree( ...
    'DataFormat','column', ...
    'MaxNumBodies',n+1);

parentName = simRobot.BaseName;

for i = 1:n

    body = rigidBody(sprintf('link%d',i));
    joint = rigidBodyJoint(sprintf('joint%d',i),'revolute');

    setFixedTransform( ...
        joint, ...
        [a(i), alpha(i), d(i), 0], ...
        'dh');

    body.Joint = joint;
    addBody(simRobot,body,parentName);

    parentName = body.Name;
end

tcpBody = rigidBody('TCP');
tcpJoint = rigidBodyJoint('TCP_fixed','fixed');

setFixedTransform( ...
    tcpJoint, ...
    robot.tool.T_F_TCP);

tcpBody.Joint = tcpJoint;

addBody( ...
    simRobot, ...
    tcpBody, ...
    parentName);

fprintf('[2] Independent Simulation model built: PASS\n\n');

%% 3. Same five reachable target configurations

qTestsDeg = [
     30   -45    60    20   -30    45
    -30   -60    80   -20    40    60
     60   -30    40    90   -45   -30
    -60   -75    70    30    45   -60
     15   -90    90     0    30    90
];

qTests = deg2rad(qTestsDeg);

numberOfTargets = size(qTests,1);

%% 4. Seed-generation range
%
% If a joint is mathematically unbounded, use +/-pi only for generating
% reasonable stress-test seeds.
%
% Final solutions are still checked against the true robot limits.

seedLow = qMin;
seedHigh = qMax;

seedLow(~isfinite(seedLow)) = -pi;
seedHigh(~isfinite(seedHigh)) = pi;

% Protect against any degenerate limit definition.
badRange = seedHigh < seedLow;

seedLow(badRange) = -pi;
seedHigh(badRange) = pi;

seedMid = 0.5 * (seedLow + seedHigh);

seedQuarterLow = ...
    seedLow + 0.25 * (seedHigh - seedLow);

seedQuarterHigh = ...
    seedLow + 0.75 * (seedHigh - seedLow);

%% 5. Independent validation tolerances

positionTolerance = 1e-4;       % [m]
orientationTolerance = 1e-4;    % [rad]

%% 6. Deterministic random generator
%
% Keep this identical to stressTestIKSeeds so that both solvers receive
% exactly the same random seeds.

rng(17);

numberOfSeeds = 8;

seedNames = {
    'Zero'
    'HalfTarget'
    'OppositeTarget'
    'MidLimits'
    'QuarterLow'
    'QuarterHigh'
    'RandomA'
    'RandomB'
};

%% 7. Storage

totalRuns = numberOfTargets * numberOfSeeds;

Target = zeros(totalRuns,1);
Seed = strings(totalRuns,1);

Converged = false(totalRuns,1);
Iterations = zeros(totalRuns,1);

PositionError_m = inf(totalRuns,1);
OrientationError_rad = inf(totalRuns,1);

WithinLimits = false(totalRuns,1);
FiniteSolution = false(totalRuns,1);
Singular = false(totalRuns,1);

% -------------------------------------------------------------
% ADLS-specific diagnostics
% -------------------------------------------------------------
%
% MinSigma:
%   Smallest sigma_min encountered during the entire IK solve.
%
% MaxLambda:
%   Largest adaptive damping factor used during the entire IK solve.
%
% DampingActivated:
%   True if lambdaDLS became greater than zero at any iteration.

MinSigma = nan(totalRuns,1);
MaxLambda = nan(totalRuns,1);
DampingActivated = false(totalRuns,1);

Pass = false(totalRuns,1);

solutionStore = nan( ...
    n, ...
    numberOfSeeds, ...
    numberOfTargets);

seedStore = nan( ...
    n, ...
    numberOfSeeds, ...
    numberOfTargets);

row = 0;

%% 8. Run all target / seed combinations

for targetIndex = 1:numberOfTargets

    qTarget = qTests(targetIndex,:).';

    % ---------------------------------------------------------
    % Generate target pose using ONLY the independent
    % Robotics System Toolbox simulation model.
    % ---------------------------------------------------------

    TTarget = getTransform( ...
        simRobot, ...
        qTarget + thetaOffset, ...
        'TCP');

    % Generate two deterministic random seeds.
    randomA = ...
        seedLow + ...
        (seedHigh - seedLow) .* rand(n,1);

    randomB = ...
        seedLow + ...
        (seedHigh - seedLow) .* rand(n,1);

    % ---------------------------------------------------------
    % Build exactly the same seed set used for the fixed-DLS
    % stress test.
    % ---------------------------------------------------------

    seeds = zeros(n,numberOfSeeds);

    seeds(:,1) = ...
        min(max(zeros(n,1),qMin),qMax);

    seeds(:,2) = ...
        min(max(0.5*qTarget,qMin),qMax);

    seeds(:,3) = ...
        min(max(-qTarget,qMin),qMax);

    seeds(:,4) = ...
        min(max(seedMid,qMin),qMax);

    seeds(:,5) = ...
        min(max(seedQuarterLow,qMin),qMax);

    seeds(:,6) = ...
        min(max(seedQuarterHigh,qMin),qMax);

    seeds(:,7) = ...
        min(max(randomA,qMin),qMax);

    seeds(:,8) = ...
        min(max(randomB,qMin),qMax);

    seedStore(:,:,targetIndex) = seeds;

    fprintf('------------------------------------------------------------\n');
    fprintf('TARGET %d\n',targetIndex);

    fprintf('Generating configuration [deg]:\n');
    disp(qTestsDeg(targetIndex,:));

    for seedIndex = 1:numberOfSeeds

        row = row + 1;

        qSeed = seeds(:,seedIndex);

        Target(row) = targetIndex;
        Seed(row) = seedNames{seedIndex};

        fprintf('  Seed %-14s : ',seedNames{seedIndex});

        %% -----------------------------------------------------
        % Run ADLS inverse kinematics
        % ------------------------------------------------------

        try

            [qSolution,info] = ADLS_IK( ...
                robot, ...
                TTarget, ...
                qSeed);

        catch ME

            fprintf('ERROR - %s\n',ME.message);
            continue

        end

        qSolution = qSolution(:);

        %% -----------------------------------------------------
        % Validate returned solution size
        % ------------------------------------------------------

        if numel(qSolution) ~= n

            fprintf('FAIL - wrong solution size\n');
            continue

        end

        solutionStore(:,seedIndex,targetIndex) = qSolution;

        %% -----------------------------------------------------
        % Read basic solver information
        % ------------------------------------------------------

        Converged(row) = info.converged;
        Iterations(row) = info.iterations;

        FiniteSolution(row) = ...
            all(isfinite(qSolution));

        %% -----------------------------------------------------
        % Read ADLS diagnostic history
        % ------------------------------------------------------

        if isfield(info,'sigmaMinHistory') && ...
           ~isempty(info.sigmaMinHistory)

            validSigma = ...
                info.sigmaMinHistory( ...
                isfinite(info.sigmaMinHistory));

            if ~isempty(validSigma)

                MinSigma(row) = min(validSigma);

            end

        end

        if isfield(info,'lambdaHistory') && ...
           ~isempty(info.lambdaHistory)

            validLambda = ...
                info.lambdaHistory( ...
                isfinite(info.lambdaHistory));

            if ~isempty(validLambda)

                MaxLambda(row) = max(validLambda);

                DampingActivated(row) = ...
                    any(validLambda > 0);

            end

        end

        %% -----------------------------------------------------
        % Reject NaN / Inf solution
        % ------------------------------------------------------

        if ~FiniteSolution(row)

            fprintf('FAIL - NaN/Inf solution\n');
            continue

        end

        %% -----------------------------------------------------
        % Check exact final singularity
        % ------------------------------------------------------

        Singular(row) = ...
            singularityCheck( ...
                robot, ...
                qSolution);

        %% -----------------------------------------------------
        % Independently evaluate solution using Simulation model
        % ------------------------------------------------------

        TAchieved = getTransform( ...
            simRobot, ...
            qSolution + thetaOffset, ...
            'TCP');

        %% Position error

        PositionError_m(row) = norm( ...
            TTarget(1:3,4) - ...
            TAchieved(1:3,4));

        %% Orientation error

        OrientationError_rad(row) = ...
            rotationError( ...
                TTarget(1:3,1:3), ...
                TAchieved(1:3,1:3));

        %% -----------------------------------------------------
        % Joint-limit validation
        % ------------------------------------------------------

        WithinLimits(row) = ...
            all(qSolution >= qMin - 1e-12) && ...
            all(qSolution <= qMax + 1e-12);

        %% -----------------------------------------------------
        % Final independent acceptance test
        % ------------------------------------------------------

        Pass(row) = ...
            Converged(row) && ...
            FiniteSolution(row) && ...
            WithinLimits(row) && ...
            ~Singular(row) && ...
            PositionError_m(row) <= positionTolerance && ...
            OrientationError_rad(row) <= orientationTolerance;

        %% -----------------------------------------------------
        % Per-run console output
        % ------------------------------------------------------

        fprintf([ ...
            '%s | iter %4d | pos %.2e m | ori %.2e rad | ' ...
            'minSigma %.2e | maxLambda %.2e | damping %s | ' ...
            'singular %s\n'], ...
            passFail(Pass(row)), ...
            Iterations(row), ...
            PositionError_m(row), ...
            OrientationError_rad(row), ...
            MinSigma(row), ...
            MaxLambda(row), ...
            yesNo(DampingActivated(row)), ...
            yesNo(Singular(row)));

    end

end

%% 9. Full results table

resultsTable = table( ...
    Target, ...
    Seed, ...
    Converged, ...
    Iterations, ...
    PositionError_m, ...
    OrientationError_rad, ...
    WithinLimits, ...
    FiniteSolution, ...
    MinSigma, ...
    MaxLambda, ...
    DampingActivated, ...
    Singular, ...
    Pass);

fprintf('\n============================================================\n');
fprintf(' FULL ADLS MULTIPLE-SEED RESULTS\n');
fprintf('============================================================\n');

disp(resultsTable);

%% 10. Per-target summary

fprintf('\n============================================================\n');
fprintf(' PER-TARGET SUMMARY\n');
fprintf('============================================================\n');

for targetIndex = 1:numberOfTargets

    rows = Target == targetIndex;

    passes = sum(Pass(rows));
    conv = sum(Converged(rows));

    dampingRuns = sum(DampingActivated(rows));

    fprintf('\nTarget %d:\n',targetIndex);

    fprintf( ...
        '  Converged seeds     : %d / %d\n', ...
        conv, ...
        numberOfSeeds);

    fprintf( ...
        '  Passed seeds        : %d / %d\n', ...
        passes, ...
        numberOfSeeds);

    fprintf( ...
        '  Damping activated   : %d / %d\n', ...
        dampingRuns, ...
        numberOfSeeds);

    %% Worst finite position error

    validPos = ...
        PositionError_m(rows & isfinite(PositionError_m));

    if ~isempty(validPos)

        fprintf( ...
            '  Worst pos error     : %.3e m\n', ...
            max(validPos));

    end

    %% Worst finite orientation error

    validOri = ...
        OrientationError_rad( ...
        rows & isfinite(OrientationError_rad));

    if ~isempty(validOri)

        fprintf( ...
            '  Worst ori error     : %.3e rad\n', ...
            max(validOri));

    end

    %% Smallest sigma encountered for this target

    validTargetSigma = ...
        MinSigma(rows & isfinite(MinSigma));

    if ~isempty(validTargetSigma)

        fprintf( ...
            '  Smallest sigmaMin   : %.3e\n', ...
            min(validTargetSigma));

    end

    %% Largest damping used for this target

    validTargetLambda = ...
        MaxLambda(rows & isfinite(MaxLambda));

    if ~isempty(validTargetLambda)

        fprintf( ...
            '  Largest lambda used : %.3e\n', ...
            max(validTargetLambda));

    end

    %% Show all accepted joint solutions

    fprintf('  Returned valid joint solutions [deg]:\n');

    for seedIndex = 1:numberOfSeeds

        globalRow = ...
            (targetIndex - 1) * ...
            numberOfSeeds + ...
            seedIndex;

        if Pass(globalRow)

            fprintf( ...
                '    %-14s ', ...
                seedNames{seedIndex});

            fprintf( ...
                '%9.2f ', ...
                rad2deg( ...
                    solutionStore( ...
                    :,seedIndex,targetIndex)));

            fprintf('\n');

        else

            fprintf( ...
                '    %-14s FAILED\n', ...
                seedNames{seedIndex});

        end

    end

end

%% 11. Overall summary

fprintf('\n============================================================\n');
fprintf(' ADLS IK MULTIPLE-SEED STRESS-TEST SUMMARY\n');
fprintf('============================================================\n');

fprintf( ...
    'Total runs: %d\n', ...
    totalRuns);

fprintf( ...
    'Converged: %d / %d\n', ...
    sum(Converged), ...
    totalRuns);

fprintf( ...
    'Within limits: %d / %d\n', ...
    sum(WithinLimits), ...
    totalRuns);

fprintf( ...
    'Converged and non-singular: %d / %d\n', ...
    sum( ...
        Converged & ...
        FiniteSolution & ...
        ~Singular), ...
    totalRuns);

fprintf( ...
    'ADLS damping activated: %d / %d runs\n', ...
    sum(DampingActivated), ...
    totalRuns);

fprintf( ...
    'Overall PASS: %d / %d\n', ...
    sum(Pass), ...
    totalRuns);

%% Smallest sigma_min encountered

validSigma = ...
    MinSigma(isfinite(MinSigma));

if ~isempty(validSigma)

    fprintf( ...
        'Smallest sigmaMin encountered: %.3e\n', ...
        min(validSigma));

end

%% Largest adaptive damping used

validLambda = ...
    MaxLambda(isfinite(MaxLambda));

if ~isempty(validLambda)

    fprintf( ...
        'Largest damping used: %.3e\n', ...
        max(validLambda));

end

%% Worst finite position error

finitePos = ...
    PositionError_m( ...
    isfinite(PositionError_m));

if ~isempty(finitePos)

    fprintf( ...
        'Worst finite position error: %.3e m\n', ...
        max(finitePos));

end

%% Worst finite orientation error

finiteOri = ...
    OrientationError_rad( ...
    isfinite(OrientationError_rad));

if ~isempty(finiteOri)

    fprintf( ...
        'Worst finite orientation error: %.3e rad\n', ...
        max(finiteOri));

end

%% Final result

if all(Pass)

    fprintf('\nOVERALL RESULT: PASS\n');

    fprintf([ ...
        'ADLS IK successfully reached every independently generated ', ...
        'Simulation target from all tested initial seeds.\n']);

else

    fprintf('\nOVERALL RESULT: PARTIAL / FAIL\n');

    fprintf([ ...
        'At least one seed did not produce an accepted ADLS solution. ', ...
        'Inspect convergence, sigmaMin, adaptive damping activation, ', ...
        'joint limits, singularity status, and independent pose error.\n']);

end

fprintf('============================================================\n\n');

end


%% ========================================================================
% Local function: independent orientation error
% ========================================================================

function angle = rotationError(R1,R2)

RRelative = R1 * R2';

c = (trace(RRelative) - 1) / 2;

% Protect acos against floating-point values slightly outside [-1,1].
c = max(-1,min(1,c));

angle = acos(c);

end


%% ========================================================================
% Local function: PASS / FAIL text
% ========================================================================

function txt = passFail(tf)

if tf

    txt = 'PASS';

else

    txt = 'FAIL';

end

end


%% ========================================================================
% Local function: YES / NO text
% ========================================================================

function txt = yesNo(tf)

if tf

    txt = 'YES';

else

    txt = 'NO';

end

end