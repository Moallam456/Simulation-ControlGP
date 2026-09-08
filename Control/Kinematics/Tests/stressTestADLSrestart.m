function stressTestADLSrestart()
% validation.stressTestADLSrestart
%
% SIMULATION TEAM - ADLS + RANDOM RESTART STRESS TEST
%
% PURPOSE
% Test whether Adaptive Damped Least-Squares IK with an established
% random-restart strategy improves robustness when solving the same
% reachable TCP targets from very different initial joint guesses.
%
% The target pose is generated ONLY from the independent Simulation model.
% Every returned solution is also verified ONLY with the Simulation model.
%
% This test focuses on:
%   1) Sensitivity to the original initial seed
%   2) Convergence robustness
%   3) Ability of random restart to recover failed ADLS solves
%   4) Number of random restarts required
%   5) Joint-limit compliance
%   6) Independent TCP pose accuracy
%   7) Final singularity state
%   8) ADLS damping behavior on the final attempt
%
% Usage:
%       validation.stressTestADLSrestart
%
% Required:
%   +config/UR5.m
%   ADLS_restart_IK.m
%   ADLS_IK.m
%   controlFK.m
%   controlJacobian.m
%   singularityCheck.m
%   dhTransform.m
%   Robotics System Toolbox

clc;

fprintf('\n');
fprintf('============================================================\n');
fprintf('    SIMULATION TEAM - ADLS + RANDOM RESTART STRESS TEST\n');
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

    addBody( ...
        simRobot, ...
        body, ...
        parentName);

    parentName = body.Name;

end

tcpBody = rigidBody('TCP');

tcpJoint = rigidBodyJoint( ...
    'TCP_fixed', ...
    'fixed');

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
%
% IMPORTANT:
% These are identical to stressTestIKSeeds and stressTestADLS.
%
% This keeps the comparison fair.

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

seedLow = qMin;
seedHigh = qMax;

% If limits are mathematically infinite, use +/-pi ONLY for
% stress-test seed generation.
seedLow(~isfinite(seedLow)) = -pi;
seedHigh(~isfinite(seedHigh)) = pi;

% Protect against malformed limits.
badRange = seedHigh < seedLow;

seedLow(badRange) = -pi;
seedHigh(badRange) = pi;

seedMid = ...
    0.5 * (seedLow + seedHigh);

seedQuarterLow = ...
    seedLow + ...
    0.25 * (seedHigh - seedLow);

seedQuarterHigh = ...
    seedLow + ...
    0.75 * (seedHigh - seedLow);

%% 5. Independent validation tolerances
%
% Same values as previous stress tests.

positionTolerance = 1e-4;       % [m]
orientationTolerance = 1e-4;    % [rad]

%% 6. Random-restart settings
%
% maxRandomRestarts:
%
%   Number of extra random ADLS attempts allowed AFTER
%   the original preferred seed fails.
%
% Therefore:
%
%   maxRandomRestarts = 5
%
% means a maximum of:
%
%   1 original attempt + 5 restart attempts = 6 total attempts.
%
% restartSeed is fixed so the experiment is reproducible.

maxRandomRestarts = 5;

restartSeed = 17;

%% 7. Deterministic generator for ORIGINAL stress-test seeds
%
% This is deliberately identical to the other stress tests.

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

%% 8. Storage

totalRuns = ...
    numberOfTargets * numberOfSeeds;

Target = zeros(totalRuns,1);

Seed = strings(totalRuns,1);

% -------------------------------------------------------------
% Main convergence information
% -------------------------------------------------------------

Converged = false(totalRuns,1);

% True when the ORIGINAL seed succeeded without any restart.
InitialSeedSucceeded = false(totalRuns,1);

% True when original seed failed but a random restart recovered the solve.
RecoveredByRestart = false(totalRuns,1);

% Number of restart attempts actually performed.
RandomRestarts = zeros(totalRuns,1);

% Number of complete ADLS solves performed.
TotalAttempts = zeros(totalRuns,1);

% Total number of ADLS iterations spent across ALL attempts.
TotalIterations = zeros(totalRuns,1);

% Iterations used by the final/successful ADLS attempt.
FinalAttemptIterations = zeros(totalRuns,1);

% -------------------------------------------------------------
% Independent validation information
% -------------------------------------------------------------

PositionError_m = inf(totalRuns,1);

OrientationError_rad = inf(totalRuns,1);

WithinLimits = false(totalRuns,1);

FiniteSolution = false(totalRuns,1);

Singular = false(totalRuns,1);

Pass = false(totalRuns,1);

% -------------------------------------------------------------
% ADLS diagnostics from the FINAL attempt
% -------------------------------------------------------------
%
% ADLS_restart_IK currently exposes finalSolverInfo.
%
% Therefore these values describe the final attempt that either:
%
%   - successfully solved the target, or
%   - was the last failed attempt.
%
% They do NOT represent the minimum/maximum across every restart attempt.

FinalMinSigma = nan(totalRuns,1);

FinalMaxLambda = nan(totalRuns,1);

FinalDampingActivated = false(totalRuns,1);

% -------------------------------------------------------------
% Solution storage
% -------------------------------------------------------------

solutionStore = nan( ...
    n, ...
    numberOfSeeds, ...
    numberOfTargets);

seedStore = nan( ...
    n, ...
    numberOfSeeds, ...
    numberOfTargets);

row = 0;

%% 9. Run all target / original-seed combinations

for targetIndex = 1:numberOfTargets

    qTarget = ...
        qTests(targetIndex,:).';

    %% ---------------------------------------------------------
    % Generate target pose independently
    % ----------------------------------------------------------

    TTarget = getTransform( ...
        simRobot, ...
        qTarget + thetaOffset, ...
        'TCP');

    %% ---------------------------------------------------------
    % Generate exactly the same two deterministic random
    % ORIGINAL seeds used by the previous stress tests.
    % ----------------------------------------------------------

    randomA = ...
        seedLow + ...
        (seedHigh - seedLow) .* ...
        rand(n,1);

    randomB = ...
        seedLow + ...
        (seedHigh - seedLow) .* ...
        rand(n,1);

    %% ---------------------------------------------------------
    % Build original seed set
    % ----------------------------------------------------------

    seeds = zeros( ...
        n, ...
        numberOfSeeds);

    seeds(:,1) = ...
        min( ...
            max(zeros(n,1),qMin), ...
            qMax);

    seeds(:,2) = ...
        min( ...
            max(0.5*qTarget,qMin), ...
            qMax);

    seeds(:,3) = ...
        min( ...
            max(-qTarget,qMin), ...
            qMax);

    seeds(:,4) = ...
        min( ...
            max(seedMid,qMin), ...
            qMax);

    seeds(:,5) = ...
        min( ...
            max(seedQuarterLow,qMin), ...
            qMax);

    seeds(:,6) = ...
        min( ...
            max(seedQuarterHigh,qMin), ...
            qMax);

    seeds(:,7) = ...
        min( ...
            max(randomA,qMin), ...
            qMax);

    seeds(:,8) = ...
        min( ...
            max(randomB,qMin), ...
            qMax);

    seedStore(:,:,targetIndex) = seeds;

    %% ---------------------------------------------------------
    % Console information
    % ----------------------------------------------------------

    fprintf('------------------------------------------------------------\n');
    fprintf('TARGET %d\n',targetIndex);

    fprintf('Generating configuration [deg]:\n');

    disp(qTestsDeg(targetIndex,:));

    %% =========================================================
    % Test each original seed
    % =========================================================

    for seedIndex = 1:numberOfSeeds

        row = row + 1;

        qSeed = seeds(:,seedIndex);

        Target(row) = targetIndex;

        Seed(row) = ...
            seedNames{seedIndex};

        fprintf( ...
            '  Seed %-14s : ', ...
            seedNames{seedIndex});

        %% -----------------------------------------------------
        % Run ADLS + random restart IK
        % ------------------------------------------------------

        try

            [qSolution,info] = ...
                ADLS_restart_IK( ...
                    robot, ...
                    TTarget, ...
                    qSeed, ...
                    maxRandomRestarts, ...
                    restartSeed);

        catch ME

            fprintf( ...
                'ERROR - %s\n', ...
                ME.message);

            continue

        end

        qSolution = qSolution(:);

        %% -----------------------------------------------------
        % Validate solution dimensions
        % ------------------------------------------------------

        if numel(qSolution) ~= n

            fprintf( ...
                'FAIL - wrong solution size\n');

            continue

        end

        solutionStore( ...
            :,seedIndex,targetIndex) = ...
            qSolution;

        %% -----------------------------------------------------
        % Read restart information
        % ------------------------------------------------------

        Converged(row) = ...
            info.converged;

        InitialSeedSucceeded(row) = ...
            info.initialSeedSucceeded;

        RandomRestarts(row) = ...
            info.randomRestarts;

        TotalAttempts(row) = ...
            info.totalAttempts;

        TotalIterations(row) = ...
            info.totalIterations;

        % A solution is considered "recovered" when:
        %
        %   original seed failed
        %   AND
        %   a later restart succeeded.

        RecoveredByRestart(row) = ...
            info.converged && ...
            ~info.initialSeedSucceeded;

        %% -----------------------------------------------------
        % Read information from final ADLS attempt
        % ------------------------------------------------------

        if isfield(info,'finalSolverInfo') && ...
           ~isempty(fieldnames(info.finalSolverInfo))

            finalInfo = ...
                info.finalSolverInfo;

            if isfield(finalInfo,'iterations')

                FinalAttemptIterations(row) = ...
                    finalInfo.iterations;

            end

            %% Minimum sigma of final attempt

            if isfield(finalInfo,'sigmaMinHistory') && ...
               ~isempty(finalInfo.sigmaMinHistory)

                validSigma = ...
                    finalInfo.sigmaMinHistory( ...
                    isfinite( ...
                    finalInfo.sigmaMinHistory));

                if ~isempty(validSigma)

                    FinalMinSigma(row) = ...
                        min(validSigma);

                end

            end

            %% Maximum lambda of final attempt

            if isfield(finalInfo,'lambdaHistory') && ...
               ~isempty(finalInfo.lambdaHistory)

                validLambda = ...
                    finalInfo.lambdaHistory( ...
                    isfinite( ...
                    finalInfo.lambdaHistory));

                if ~isempty(validLambda)

                    FinalMaxLambda(row) = ...
                        max(validLambda);

                    FinalDampingActivated(row) = ...
                        any(validLambda > 0);

                end

            end

        end

        %% -----------------------------------------------------
        % Check numerical validity
        % ------------------------------------------------------

        FiniteSolution(row) = ...
            all(isfinite(qSolution));

        if ~FiniteSolution(row)

            fprintf( ...
                'FAIL - NaN/Inf solution\n');

            continue

        end

        %% -----------------------------------------------------
        % Final singularity check
        % ------------------------------------------------------

        Singular(row) = ...
            singularityCheck( ...
                robot, ...
                qSolution);

        %% -----------------------------------------------------
        % Independent Simulation evaluation
        % ------------------------------------------------------

        TAchieved = getTransform( ...
            simRobot, ...
            qSolution + thetaOffset, ...
            'TCP');

        %% Position error

        PositionError_m(row) = ...
            norm( ...
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
            all( ...
                qSolution >= ...
                qMin - 1e-12) && ...
            all( ...
                qSolution <= ...
                qMax + 1e-12);

        %% -----------------------------------------------------
        % Final independent acceptance condition
        % ------------------------------------------------------

        Pass(row) = ...
            Converged(row) && ...
            FiniteSolution(row) && ...
            WithinLimits(row) && ...
            ~Singular(row) && ...
            PositionError_m(row) <= ...
                positionTolerance && ...
            OrientationError_rad(row) <= ...
                orientationTolerance;

        %% -----------------------------------------------------
        % Console result
        % ------------------------------------------------------

        fprintf([ ...
            '%s | attempts %d | restarts %d | totalIter %4d | ' ...
            'pos %.2e m | ori %.2e rad | rescued %s | ' ...
            'singular %s\n'], ...
            passFail(Pass(row)), ...
            TotalAttempts(row), ...
            RandomRestarts(row), ...
            TotalIterations(row), ...
            PositionError_m(row), ...
            OrientationError_rad(row), ...
            yesNo(RecoveredByRestart(row)), ...
            yesNo(Singular(row)));

    end

end

%% 10. Full results table

resultsTable = table( ...
    Target, ...
    Seed, ...
    Converged, ...
    InitialSeedSucceeded, ...
    RecoveredByRestart, ...
    RandomRestarts, ...
    TotalAttempts, ...
    TotalIterations, ...
    FinalAttemptIterations, ...
    PositionError_m, ...
    OrientationError_rad, ...
    WithinLimits, ...
    FiniteSolution, ...
    FinalMinSigma, ...
    FinalMaxLambda, ...
    FinalDampingActivated, ...
    Singular, ...
    Pass);

fprintf('\n============================================================\n');
fprintf(' FULL ADLS + RANDOM RESTART RESULTS\n');
fprintf('============================================================\n');

disp(resultsTable);

%% 11. Per-target summary

fprintf('\n============================================================\n');
fprintf(' PER-TARGET SUMMARY\n');
fprintf('============================================================\n');

for targetIndex = 1:numberOfTargets

    rows = ...
        Target == targetIndex;

    passes = ...
        sum(Pass(rows));

    converged = ...
        sum(Converged(rows));

    originalSuccess = ...
        sum(InitialSeedSucceeded(rows));

    rescued = ...
        sum(RecoveredByRestart(rows));

    fprintf('\nTarget %d:\n',targetIndex);

    fprintf( ...
        '  Converged seeds       : %d / %d\n', ...
        converged, ...
        numberOfSeeds);

    fprintf( ...
        '  Passed seeds          : %d / %d\n', ...
        passes, ...
        numberOfSeeds);

    fprintf( ...
        '  Original seed success : %d / %d\n', ...
        originalSuccess, ...
        numberOfSeeds);

    fprintf( ...
        '  Recovered by restart  : %d / %d\n', ...
        rescued, ...
        numberOfSeeds);

    %% Maximum restart count

    fprintf( ...
        '  Most restarts used    : %d\n', ...
        max(RandomRestarts(rows)));

    %% Worst finite position error

    validPos = ...
        PositionError_m( ...
        rows & ...
        isfinite(PositionError_m));

    if ~isempty(validPos)

        fprintf( ...
            '  Worst pos error       : %.3e m\n', ...
            max(validPos));

    end

    %% Worst finite orientation error

    validOri = ...
        OrientationError_rad( ...
        rows & ...
        isfinite(OrientationError_rad));

    if ~isempty(validOri)

        fprintf( ...
            '  Worst ori error       : %.3e rad\n', ...
            max(validOri));

    end

    %% ---------------------------------------------------------
    % Returned valid configurations
    % ----------------------------------------------------------

    fprintf( ...
        '  Returned valid joint solutions [deg]:\n');

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
                    :, ...
                    seedIndex, ...
                    targetIndex)));

            if RecoveredByRestart(globalRow)

                fprintf( ...
                    '  [RESTARTED x%d]', ...
                    RandomRestarts(globalRow));

            end

            fprintf('\n');

        else

            fprintf( ...
                '    %-14s FAILED\n', ...
                seedNames{seedIndex});

        end

    end

end

%% 12. Overall summary

fprintf('\n============================================================\n');
fprintf(' ADLS + RANDOM RESTART STRESS-TEST SUMMARY\n');
fprintf('============================================================\n');

fprintf( ...
    'Total runs: %d\n', ...
    totalRuns);

fprintf( ...
    'Converged: %d / %d\n', ...
    sum(Converged), ...
    totalRuns);

fprintf( ...
    'Original seed succeeded: %d / %d\n', ...
    sum(InitialSeedSucceeded), ...
    totalRuns);

fprintf( ...
    'Recovered by random restart: %d / %d\n', ...
    sum(RecoveredByRestart), ...
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
    'Overall PASS: %d / %d\n', ...
    sum(Pass), ...
    totalRuns);

%% Restart statistics

successfulRestarts = ...
    RandomRestarts( ...
        Pass & ...
        RecoveredByRestart);

if ~isempty(successfulRestarts)

    fprintf( ...
        'Average restarts for recovered runs: %.2f\n', ...
        mean(successfulRestarts));

    fprintf( ...
        'Maximum restarts for recovered run: %d\n', ...
        max(successfulRestarts));

end

%% Total computational effort

fprintf( ...
    'Total ADLS attempts: %d\n', ...
    sum(TotalAttempts));

fprintf( ...
    'Total ADLS iterations: %d\n', ...
    sum(TotalIterations));

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

%% Final overall result

if all(Pass)

    fprintf('\nOVERALL RESULT: PASS\n');

    fprintf([ ...
        'ADLS with random restart successfully reached every ', ...
        'independently generated Simulation target from all ', ...
        'tested original seeds.\n']);

else

    fprintf('\nOVERALL RESULT: PARTIAL / FAIL\n');

    fprintf([ ...
        'At least one target remained unresolved after the ', ...
        'allowed random restart attempts. Inspect the failed ', ...
        'target, total attempts, joint limits, singularity ', ...
        'state, and independent pose errors.\n']);

end

fprintf('============================================================\n\n');

end


%% ========================================================================
% Local function: independent orientation error
% ========================================================================

function angle = rotationError(R1,R2)

RRelative = ...
    R1 * R2';

c = ...
    (trace(RRelative) - 1) / 2;

% Protect acos from tiny floating-point excursions outside [-1,1].
c = max( ...
    -1, ...
    min(1,c));

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