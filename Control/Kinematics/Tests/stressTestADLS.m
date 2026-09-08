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

%% 5A. Diagnostic visualization configuration
%
% The stress test still evaluates every run exactly as before.
% Visualization is generated AFTER all runs finish.
%
% Selection policy:
%   - Plot every failed run.
%   - Also plot a few representative successful runs:
%       1) fastest successful solve
%       2) slowest successful solve
%       3) successful solve with strongest damping
%
% Each figure contains up to two runs.
% Each run gets:
%   1) a 2-D IK cost-landscape slice
%   2) independent pose-cost history
%   3) sigma_min / lambda history
%
% IMPORTANT:
% The contour is a 2-joint slice of the full 6-D IK objective.
% It is useful for visualization, but it is NOT proof by itself that a
% point is a true local minimum in the complete 6-D joint space.

viz.enabled = true;

viz.numberOfSuccessExamples = 3;
viz.runsPerFigure = 2;

% Grid resolution for each contour plot.
% 45 is a good compromise between smoothness and runtime.
viz.contourGridSize = 45;

% Default joints used for the 2-D cost slice.
% Use [] instead to automatically choose the two joints that moved most
% during that particular IK run.
viz.contourJoints = [2 3];

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

% Store target poses so diagnostic plots can be generated after all runs.
TTargetStore = nan(4,4,numberOfTargets);

% Per-run solver histories used only for diagnostics / visualization.
qHistoryStore = cell(totalRuns,1);
hasIterationHistory = false(totalRuns,1);
sigmaHistoryStore = cell(totalRuns,1);
lambdaHistoryStore = cell(totalRuns,1);

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

    TTargetStore(:,:,targetIndex) = TTarget;

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

        % ------------------------------------------------------
        % Capture full joint-iteration history if ADLS_IK exposes it.
        %
        % If qHistory is unavailable, the diagnostic plots gracefully
        % fall back to showing only the seed and final solution.
        % ------------------------------------------------------

        [qHistory,hasRealHistory] = ...
            extractQHistory(info,n,qSeed,qSolution);

        qHistoryStore{row} = qHistory;
        hasIterationHistory(row) = hasRealHistory;

        if isfield(info,'sigmaMinHistory') && ...
           ~isempty(info.sigmaMinHistory)

            sigmaHistoryStore{row} = info.sigmaMinHistory(:);

        end

        if isfield(info,'lambdaHistory') && ...
           ~isempty(info.lambdaHistory)

            lambdaHistoryStore{row} = info.lambdaHistory(:);

        end

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

%% 12. Failure / success diagnostic visualization

if viz.enabled

    plotADLSStressDiagnostics( ...
        simRobot, ...
        thetaOffset, ...
        qMin, ...
        qMax, ...
        seedLow, ...
        seedHigh, ...
        TTargetStore, ...
        Target, ...
        Seed, ...
        Pass, ...
        Iterations, ...
        MaxLambda, ...
        solutionStore, ...
        seedStore, ...
        qHistoryStore, ...
        hasIterationHistory, ...
        sigmaHistoryStore, ...
        lambdaHistoryStore, ...
        positionTolerance, ...
        orientationTolerance, ...
        numberOfSeeds, ...
        viz);

end

fprintf('============================================================\n\n');

end


%% ========================================================================
% Local function: extract / normalize q history from ADLS_IK
% ========================================================================

function [qHistory,hasRealHistory] = ...
    extractQHistory(info,n,qSeed,qSolution)

hasRealHistory = false;
qHistory = [];

if isfield(info,'qHistory') && ~isempty(info.qHistory)

    raw = info.qHistory;

    if size(raw,1) == n

        qHistory = raw;
        hasRealHistory = true;

    elseif size(raw,2) == n

        qHistory = raw.';
        hasRealHistory = true;

    end

end

% Remove columns containing NaN / Inf so plotting remains robust.
if ~isempty(qHistory)

    finiteColumns = all(isfinite(qHistory),1);
    qHistory = qHistory(:,finiteColumns);

end

% Ensure the first point is the actual seed.
if isempty(qHistory)

    qHistory = qSeed(:);

elseif norm(qHistory(:,1) - qSeed(:)) > 1e-12

    qHistory = [qSeed(:), qHistory];

end

% Ensure the final returned configuration appears in the history.
if numel(qSolution) == n && all(isfinite(qSolution))

    if isempty(qHistory) || ...
       norm(qHistory(:,end) - qSolution(:)) > 1e-12

        qHistory = [qHistory, qSolution(:)];

    end

end

end


%% ========================================================================
% Local function: choose runs and create diagnostic figures
% ========================================================================

function plotADLSStressDiagnostics( ...
    simRobot, ...
    thetaOffset, ...
    qMin, ...
    qMax, ...
    seedLow, ...
    seedHigh, ...
    TTargetStore, ...
    Target, ...
    Seed, ...
    Pass, ...
    Iterations, ...
    MaxLambda, ...
    solutionStore, ...
    seedStore, ...
    qHistoryStore, ...
    hasIterationHistory, ...
    sigmaHistoryStore, ...
    lambdaHistoryStore, ...
    positionTolerance, ...
    orientationTolerance, ...
    numberOfSeeds, ...
    viz)

failureRows = find(~Pass);
successRows = find(Pass);

representativeSuccessRows = ...
    selectRepresentativeSuccesses( ...
        successRows, ...
        Iterations, ...
        MaxLambda, ...
        viz.numberOfSuccessExamples);

selectedRows = ...
    interleaveFailureSuccessRows( ...
        failureRows, ...
        representativeSuccessRows);

if isempty(selectedRows)

    fprintf('\n[Visualization] No runs available to plot.\n');
    return

end

fprintf('\n============================================================\n');
fprintf(' ADLS IK DIAGNOSTIC VISUALIZATION\n');
fprintf('============================================================\n');
fprintf('Failed runs plotted          : %d\n',numel(failureRows));
fprintf('Successful examples plotted  : %d\n', ...
    numel(representativeSuccessRows));
fprintf('Runs per figure              : %d\n',viz.runsPerFigure);

if any(~hasIterationHistory(selectedRows))

    fprintf([ ...
        'NOTE: At least one selected run has no info.qHistory. ' ...
        'For those runs, only seed/final points can be shown.\n']);

end

fprintf([ ...
    'Contour meaning: 2-joint slice of the 6-D diagnostic IK cost; ' ...
    'other joints are fixed at the final/last configuration.\n']);
fprintf([ ...
    'Cost meaning: 0.5*((positionError/positionTolerance)^2 + ' ...
    '(orientationError/orientationTolerance)^2).\n\n']);

numberSelected = numel(selectedRows);
numberFigures = ceil(numberSelected / viz.runsPerFigure);

for figureIndex = 1:numberFigures

    firstIndex = ...
        (figureIndex - 1) * viz.runsPerFigure + 1;

    lastIndex = min( ...
        figureIndex * viz.runsPerFigure, ...
        numberSelected);

    rowsThisFigure = selectedRows(firstIndex:lastIndex);
    numberRowsThisFigure = numel(rowsThisFigure);

    figure( ...
        'Name',sprintf( ...
            'ADLS IK Diagnostics %d/%d', ...
            figureIndex, ...
            numberFigures), ...
        'NumberTitle','off');

    tiledlayout( ...
        numberRowsThisFigure, ...
        3, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    for localIndex = 1:numberRowsThisFigure

        runRow = rowsThisFigure(localIndex);

        targetIndex = Target(runRow);
        seedIndex = mod(runRow - 1,numberOfSeeds) + 1;

        qSeed = seedStore(:,seedIndex,targetIndex);
        qSolution = solutionStore(:,seedIndex,targetIndex);
        qHistory = qHistoryStore{runRow};

        TTarget = TTargetStore(:,:,targetIndex);

        sigmaHistory = sigmaHistoryStore{runRow};
        lambdaHistory = lambdaHistoryStore{runRow};

        % ------------------------------------------------------
        % Panel 1: 2-D cost landscape
        % ------------------------------------------------------

        nexttile;

        plotCostLandscape( ...
            simRobot, ...
            thetaOffset, ...
            qMin, ...
            qMax, ...
            seedLow, ...
            seedHigh, ...
            TTarget, ...
            qSeed, ...
            qSolution, ...
            qHistory, ...
            Pass(runRow), ...
            targetIndex, ...
            Seed(runRow), ...
            positionTolerance, ...
            orientationTolerance, ...
            viz);

        % ------------------------------------------------------
        % Panel 2: cost vs iteration
        % ------------------------------------------------------

        nexttile;

        plotCostHistory( ...
            simRobot, ...
            thetaOffset, ...
            TTarget, ...
            qHistory, ...
            hasIterationHistory(runRow), ...
            positionTolerance, ...
            orientationTolerance);

        % ------------------------------------------------------
        % Panel 3: sigma_min and lambda
        % ------------------------------------------------------

        nexttile;

        plotDampingDiagnostics( ...
            sigmaHistory, ...
            lambdaHistory);

    end

    sgtitle(sprintf( ...
        'ADLS IK selected-run diagnostics - figure %d of %d', ...
        figureIndex, ...
        numberFigures));

end

end


%% ========================================================================
% Local function: representative successful runs
% ========================================================================

function selected = selectRepresentativeSuccesses( ...
    successRows,Iterations,MaxLambda,requestedCount)

selected = [];

if isempty(successRows) || requestedCount <= 0
    return
end

% 1) Fastest successful run.
[~,idx] = min(Iterations(successRows));
selected(end+1) = successRows(idx); %#ok<AGROW>

% 2) Slowest successful run.
[~,idx] = max(Iterations(successRows));
candidate = successRows(idx);

if ~ismember(candidate,selected)
    selected(end+1) = candidate; %#ok<AGROW>
end

% 3) Successful run that experienced the strongest damping.
lambdaValues = MaxLambda(successRows);
lambdaValues(~isfinite(lambdaValues)) = -inf;

if any(isfinite(MaxLambda(successRows)))

    [~,idx] = max(lambdaValues);
    candidate = successRows(idx);

    if ~ismember(candidate,selected)
        selected(end+1) = candidate; %#ok<AGROW>
    end

end

% Fill deterministically if the diagnostic categories overlapped.
for k = 1:numel(successRows)

    if numel(selected) >= requestedCount
        break
    end

    candidate = successRows(k);

    if ~ismember(candidate,selected)
        selected(end+1) = candidate; %#ok<AGROW>
    end

end

selected = selected(1:min(requestedCount,numel(selected)));

end


%% ========================================================================
% Local function: pair failures with successes where possible
% ========================================================================

function selected = interleaveFailureSuccessRows( ...
    failureRows,successRows)

selected = [];

f = 1;
s = 1;

while f <= numel(failureRows) || s <= numel(successRows)

    if f <= numel(failureRows)
        selected(end+1) = failureRows(f); %#ok<AGROW>
        f = f + 1;
    end

    if s <= numel(successRows)
        selected(end+1) = successRows(s); %#ok<AGROW>
        s = s + 1;
    end

end

end


%% ========================================================================
% Local function: 2-D cost-landscape slice
% ========================================================================

function plotCostLandscape( ...
    simRobot, ...
    thetaOffset, ...
    qMin, ...
    qMax, ...
    seedLow, ...
    seedHigh, ...
    TTarget, ...
    qSeed, ...
    qSolution, ...
    qHistory, ...
    runPassed, ...
    targetIndex, ...
    seedName, ...
    positionTolerance, ...
    orientationTolerance, ...
    viz)

n = numel(qSeed);

if isempty(qHistory)
    qHistory = qSeed(:);
end

% Choose which two joints define the plotted slice.
if isempty(viz.contourJoints)

    jointMotion = max(qHistory,[],2) - min(qHistory,[],2);
    [~,order] = sort(jointMotion,'descend');
    plotJoints = order(1:min(2,n)).';

    if numel(plotJoints) < 2
        plotJoints = [2 3];
    end

else

    plotJoints = viz.contourJoints(:).';

end

assert( ...
    numel(plotJoints) == 2 && ...
    all(plotJoints >= 1) && ...
    all(plotJoints <= n) && ...
    plotJoints(1) ~= plotJoints(2), ...
    'viz.contourJoints must contain two distinct valid joint indices.');

j1 = plotJoints(1);
j2 = plotJoints(2);

% The other four joints are fixed at the final valid solution.
% If the solver did not return a finite final solution, use the last finite
% point available in the history instead.
if numel(qSolution) == n && all(isfinite(qSolution))

    qReference = qSolution(:);

elseif ~isempty(qHistory) && all(isfinite(qHistory(:,end)))

    qReference = qHistory(:,end);

else

    qReference = qSeed(:);

end

plotLow = qMin(:);
plotHigh = qMax(:);

lowInfinite = ~isfinite(plotLow);
highInfinite = ~isfinite(plotHigh);

plotLow(lowInfinite) = seedLow(lowInfinite);
plotHigh(highInfinite) = seedHigh(highInfinite);

[j1Low,j1High] = choosePlotRange( ...
    qHistory(j1,:), ...
    qSeed(j1), ...
    qSolution, ...
    j1, ...
    plotLow(j1), ...
    plotHigh(j1));

[j2Low,j2High] = choosePlotRange( ...
    qHistory(j2,:), ...
    qSeed(j2), ...
    qSolution, ...
    j2, ...
    plotLow(j2), ...
    plotHigh(j2));

q1Values = linspace(j1Low,j1High,viz.contourGridSize);
q2Values = linspace(j2Low,j2High,viz.contourGridSize);

[Q1,Q2] = meshgrid(q1Values,q2Values);
C = nan(size(Q1));

for r = 1:size(Q1,1)

    for c = 1:size(Q1,2)

        q = qReference;
        q(j1) = Q1(r,c);
        q(j2) = Q2(r,c);

        if configurationWithinLimits(q,qMin,qMax)

            C(r,c) = diagnosticPoseCost( ...
                simRobot, ...
                thetaOffset, ...
                TTarget, ...
                q, ...
                positionTolerance, ...
                orientationTolerance);

        end

    end

end

finiteC = C(isfinite(C));

if isempty(finiteC)

    axis off
    text(0.5,0.5,'No finite contour costs available.', ...
        'HorizontalAlignment','center');
    return

end

logC = log10(C + 1e-12);

contourf( ...
    rad2deg(Q1), ...
    rad2deg(Q2), ...
    logC, ...
    28, ...
    'LineStyle','none');

hold on

% Overlay the actual joint-history projection onto the selected two joints.
% Other joints changed during the real IK run, so this path is a projection
% onto the 2-D slice rather than the exact 6-D objective trajectory.
plot( ...
    rad2deg(qHistory(j1,:)), ...
    rad2deg(qHistory(j2,:)), ...
    'k.-', ...
    'LineWidth',1.4, ...
    'MarkerSize',9, ...
    'DisplayName','IK iterations');

plot( ...
    rad2deg(qSeed(j1)), ...
    rad2deg(qSeed(j2)), ...
    'ko', ...
    'MarkerFaceColor','w', ...
    'MarkerSize',7, ...
    'DisplayName','Seed');

if numel(qSolution) == n && all(isfinite(qSolution))

    plot( ...
        rad2deg(qSolution(j1)), ...
        rad2deg(qSolution(j2)), ...
        'kp', ...
        'MarkerFaceColor','w', ...
        'MarkerSize',10, ...
        'DisplayName','Returned solution');

end

hold off

cb = colorbar;
cb.Label.String = 'log_{10}(normalized diagnostic cost)';

xlabel(sprintf('q_%d [deg]',j1));
ylabel(sprintf('q_%d [deg]',j2));
grid on

if runPassed
    statusText = 'PASS';
else
    statusText = 'FAIL';
end

title(sprintf( ...
    'T%d / %s / %s - q_%d,q_%d slice', ...
    targetIndex, ...
    char(seedName), ...
    statusText, ...
    j1, ...
    j2), ...
    'Interpreter','none');

legend('Location','best');

end


%% ========================================================================
% Local function: choose contour range for one joint
% ========================================================================

function [low,high] = choosePlotRange( ...
    qHistoryJoint,qSeedJoint,qSolution,jointIndex,limitLow,limitHigh)

values = qHistoryJoint(isfinite(qHistoryJoint));

if isfinite(qSeedJoint)
    values(end+1) = qSeedJoint; %#ok<AGROW>
end

if numel(qSolution) >= jointIndex && isfinite(qSolution(jointIndex))
    values(end+1) = qSolution(jointIndex); %#ok<AGROW>
end

if isempty(values)

    center = 0.5 * (limitLow + limitHigh);
    low = max(limitLow,center - deg2rad(30));
    high = min(limitHigh,center + deg2rad(30));
    return

end

valueMin = min(values);
valueMax = max(values);
span = valueMax - valueMin;

margin = max(0.20 * span,deg2rad(15));

low = max(limitLow,valueMin - margin);
high = min(limitHigh,valueMax + margin);

if high - low < deg2rad(10)

    center = 0.5 * (high + low);
    low = max(limitLow,center - deg2rad(15));
    high = min(limitHigh,center + deg2rad(15));

end

if high <= low

    low = limitLow;
    high = limitHigh;

end

end


%% ========================================================================
% Local function: independent diagnostic pose-cost history
% ========================================================================

function plotCostHistory( ...
    simRobot, ...
    thetaOffset, ...
    TTarget, ...
    qHistory, ...
    hasRealHistory, ...
    positionTolerance, ...
    orientationTolerance)

if isempty(qHistory)

    axis off
    text(0.5,0.5,'No joint history available.', ...
        'HorizontalAlignment','center');
    return

end

numberOfPoints = size(qHistory,2);
costHistory = nan(numberOfPoints,1);

for k = 1:numberOfPoints

    q = qHistory(:,k);

    if all(isfinite(q))

        costHistory(k) = diagnosticPoseCost( ...
            simRobot, ...
            thetaOffset, ...
            TTarget, ...
            q, ...
            positionTolerance, ...
            orientationTolerance);

    end

end

iteration = 0:(numberOfPoints - 1);
plotValues = costHistory;

finitePositive = isfinite(plotValues) & plotValues > 0;
plotValues(finitePositive) = max(plotValues(finitePositive),1e-16);
plotValues(plotValues == 0) = 1e-16;

semilogy( ...
    iteration, ...
    plotValues, ...
    '.-', ...
    'LineWidth',1.4, ...
    'MarkerSize',10);

grid on
xlabel('Iteration');
ylabel('Normalized diagnostic cost');

finiteCost = costHistory(isfinite(costHistory));

if isempty(finiteCost)

    title('Independent pose-cost history');

else

    title(sprintf( ...
        'Independent pose cost - final %.2e', ...
        finiteCost(end)));

end

if ~hasRealHistory

    text( ...
        0.02, ...
        0.06, ...
        'qHistory unavailable: seed/final only', ...
        'Units','normalized', ...
        'FontWeight','bold');

end

end


%% ========================================================================
% Local function: sigma_min and lambda history
% ========================================================================

function plotDampingDiagnostics(sigmaHistory,lambdaHistory)

hasSigma = ~isempty(sigmaHistory) && any(isfinite(sigmaHistory));
hasLambda = ~isempty(lambdaHistory) && any(isfinite(lambdaHistory));

if ~hasSigma && ~hasLambda

    axis off
    text( ...
        0.5, ...
        0.5, ...
        'No sigmaMinHistory / lambdaHistory available.', ...
        'HorizontalAlignment','center');
    return

end

if hasSigma

    yyaxis left

    sigmaPlot = abs(sigmaHistory(:));
    sigmaPlot(~isfinite(sigmaPlot)) = nan;
    sigmaPlot(sigmaPlot == 0) = 1e-16;

    semilogy( ...
        0:(numel(sigmaPlot)-1), ...
        sigmaPlot, ...
        '.-', ...
        'LineWidth',1.3);

    ylabel('sigma_{min}');

end

if hasLambda

    yyaxis right

    lambdaPlot = abs(lambdaHistory(:));
    lambdaPlot(~isfinite(lambdaPlot)) = nan;
    lambdaPlot(lambdaPlot == 0) = 1e-16;

    semilogy( ...
        0:(numel(lambdaPlot)-1), ...
        lambdaPlot, ...
        '.-', ...
        'LineWidth',1.3);

    ylabel('lambda');

end

grid on
xlabel('Iteration');
title('ADLS conditioning / damping');

end


%% ========================================================================
% Local function: normalized independent IK diagnostic cost
% ========================================================================

function cost = diagnosticPoseCost( ...
    simRobot, ...
    thetaOffset, ...
    TTarget, ...
    q, ...
    positionTolerance, ...
    orientationTolerance)

try

    TAchieved = getTransform( ...
        simRobot, ...
        q(:) + thetaOffset, ...
        'TCP');

catch

    cost = nan;
    return

end

positionError = norm( ...
    TTarget(1:3,4) - TAchieved(1:3,4));

orientationError = rotationError( ...
    TTarget(1:3,1:3), ...
    TAchieved(1:3,1:3));

normalizedPosition = positionError / positionTolerance;
normalizedOrientation = orientationError / orientationTolerance;

cost = 0.5 * ( ...
    normalizedPosition^2 + ...
    normalizedOrientation^2);

end


%% ========================================================================
% Local function: joint-limit check for contour points
% ========================================================================

function tf = configurationWithinLimits(q,qMin,qMax)

tf = true;

finiteLower = isfinite(qMin);
finiteUpper = isfinite(qMax);

if any(q(finiteLower) < qMin(finiteLower))
    tf = false;
    return
end

if any(q(finiteUpper) > qMax(finiteUpper))
    tf = false;
end

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