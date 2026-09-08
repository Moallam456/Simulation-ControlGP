function [qSolution, info] = ADLS_restart_IK( ...
    robot, ...
    T_B_TCP_target, ...
    qSeed, ...
    maxRandomRestarts, ...
    restartSeed)
%ADLS_RESTART_IK Adaptive Damped Least-Squares IK with random restarts.
%
% This function applies an established random-restart strategy around the
% ADLS_IK numerical inverse kinematics solver.
%
% Strategy:
%
%   1. First attempt:
%        Solve IK using the user-provided preferred seed qSeed.
%
%   2. If ADLS_IK converges:
%        Return the solution immediately.
%
%   3. If ADLS_IK fails:
%        Generate a random valid joint configuration inside the robot
%        joint limits and restart ADLS_IK from that configuration.
%
%   4. Continue until:
%        - a valid solution is found, OR
%        - the allowed number of random restarts is exhausted.
%
% This is the same general numerical-IK recovery strategy used by
% established robotics software such as MATLAB inverseKinematics and
% MoveIt numerical IK solvers.
%
%
% Inputs:
%   robot               - Shared robot configuration
%
%   T_B_TCP_target      - Desired 4x4 TCP pose expressed in Base frame
%
%   qSeed               - Preferred initial joint configuration [rad]
%
%   maxRandomRestarts   - Maximum number of RANDOM retries after the
%                         original qSeed attempt.
%
%                         Optional.
%                         Default = 5
%
%   restartSeed         - Seed of the random-number generator used to
%                         generate restart configurations.
%
%                         Optional.
%                         Default = 17
%
%                         Using a fixed seed makes the restart sequence
%                         deterministic and reproducible.
%
%
% Outputs:
%   qSolution           - IK solution [rad]
%
%   info                - Structure containing:
%
%       converged
%       totalAttempts
%       randomRestarts
%       successfulAttempt
%       totalIterations
%       initialSeedSucceeded
%       finalSolverInfo
%       attemptConverged
%       attemptIterations
%       attemptSeeds
%
%
% Example:
%
%   [qSolution,info] = ADLS_restart_IK( ...
%       robot, ...
%       TTarget, ...
%       qCurrent);
%
%
% With explicit restart settings:
%
%   [qSolution,info] = ADLS_restart_IK( ...
%       robot, ...
%       TTarget, ...
%       qCurrent, ...
%       5, ...
%       17);


%% -------------------------------------------------------------
% Optional parameters
% -------------------------------------------------------------

if nargin < 4 || isempty(maxRandomRestarts)

    % Number of random retries allowed AFTER the preferred seed fails.
    %
    % This is a computational-budget parameter, not part of Yang's
    % ADLS damping equation.

    maxRandomRestarts = 5;

end


if nargin < 5 || isempty(restartSeed)

    % Fixed seed for repeatable random-restart experiments.

    restartSeed = 17;

end


%% -------------------------------------------------------------
% Validate restart parameters
% -------------------------------------------------------------

if maxRandomRestarts < 0 || ...
   fix(maxRandomRestarts) ~= maxRandomRestarts

    error("maxRandomRestarts must be a nonnegative integer.");

end


%% -------------------------------------------------------------
% Read robot properties
% -------------------------------------------------------------

n = robot.dof;

qMin = robot.limits.qMin(:);
qMax = robot.limits.qMax(:);

qSeed = qSeed(:);


if numel(qSeed) ~= n

    error("qSeed must contain one value for each robot joint.");

end


if ~isequal(size(T_B_TCP_target), [4 4])

    error("T_B_TCP_target must be a 4x4 homogeneous transformation.");

end


%% -------------------------------------------------------------
% Define range used for RANDOM restart generation
% -------------------------------------------------------------
%
% Normally every physical robot joint should have finite mechanical
% limits.
%
% If a model contains an infinite limit, +/-pi is used ONLY for random
% restart generation so that a finite random configuration can still
% be produced.

restartLow = qMin;
restartHigh = qMax;

restartLow(~isfinite(restartLow)) = -pi;
restartHigh(~isfinite(restartHigh)) = pi;


% Protect against an incorrectly defined joint range.

badRange = restartHigh < restartLow;

restartLow(badRange) = -pi;
restartHigh(badRange) = pi;


%% -------------------------------------------------------------
% Create an independent deterministic random stream
% -------------------------------------------------------------
%
% Using a dedicated random stream means this function does not have to
% alter MATLAB's global RNG state.
%
% The same restartSeed produces the same sequence of restart
% configurations, which is useful for benchmarking and debugging.

restartStream = RandStream( ...
    'mt19937ar', ...
    'Seed',restartSeed);


%% -------------------------------------------------------------
% Number of complete ADLS attempts
% -------------------------------------------------------------
%
% Example:
%
%   maxRandomRestarts = 5
%
% gives:
%
%       attempt 1 = preferred qSeed
%       attempt 2 = random restart 1
%       attempt 3 = random restart 2
%       attempt 4 = random restart 3
%       attempt 5 = random restart 4
%       attempt 6 = random restart 5

maxAttempts = 1 + maxRandomRestarts;


%% -------------------------------------------------------------
% Preallocate diagnostic storage
% -------------------------------------------------------------

attemptSeeds = nan(n,maxAttempts);

attemptConverged = false(maxAttempts,1);

attemptIterations = zeros(maxAttempts,1);


%% -------------------------------------------------------------
% Initialize output information
% -------------------------------------------------------------

info.converged = false;

info.totalAttempts = 0;

info.randomRestarts = 0;

info.successfulAttempt = 0;

info.totalIterations = 0;

info.initialSeedSucceeded = false;

info.finalSolverInfo = struct();


%% -------------------------------------------------------------
% Store last returned candidate
% -------------------------------------------------------------
%
% If every attempt fails, returning the final ADLS candidate still allows
% debugging of the failed numerical search.

qLast = qSeed;

lastSolverInfo = struct();


%% =============================================================
% Random-restart IK search
% =============================================================

for attempt = 1:maxAttempts


    %% ---------------------------------------------------------
    % 1. Select initial configuration for this ADLS solve
    % ----------------------------------------------------------

    if attempt == 1

        % Always try the preferred/user-provided seed FIRST.
        %
        % On the actual robot this would normally be qCurrent or the
        % previous trajectory waypoint solution.

        qAttemptSeed = qSeed;

    else

        % -----------------------------------------------------
        % Random restart
        % ------------------------------------------------------
        %
        % Generate a new joint configuration uniformly inside
        % each joint's allowed restart range:
        %
        % q_j =
        %
        % qMin_j +
        % rand() * (qMax_j - qMin_j)
        %
        % This gives the numerical solver a completely different
        % starting location in joint space.

        qAttemptSeed = ...
            restartLow + ...
            (restartHigh - restartLow) .* ...
            rand(restartStream,n,1);

    end


    %% ---------------------------------------------------------
    % Store seed used for this attempt
    % ----------------------------------------------------------

    attemptSeeds(:,attempt) = qAttemptSeed;


    %% ---------------------------------------------------------
    % 2. Run ONE complete ADLS numerical IK solve
    % ----------------------------------------------------------

    [qCandidate, solverInfo] = ADLS_IK( ...
        robot, ...
        T_B_TCP_target, ...
        qAttemptSeed);


    qCandidate = qCandidate(:);


    %% ---------------------------------------------------------
    % Store diagnostic results
    % ----------------------------------------------------------

    attemptConverged(attempt) = ...
        solverInfo.converged;

    attemptIterations(attempt) = ...
        solverInfo.iterations;


    info.totalAttempts = attempt;

    info.totalIterations = ...
        info.totalIterations + solverInfo.iterations;


    % Keep the latest candidate for debugging in case every
    % restart eventually fails.

    qLast = qCandidate;

    lastSolverInfo = solverInfo;


    %% ---------------------------------------------------------
    % 3. Successful numerical IK solve
    % ----------------------------------------------------------

    if solverInfo.converged

        qSolution = qCandidate;


        %% -----------------------------------------------------
        % Final output information
        % ------------------------------------------------------

        info.converged = true;

        info.successfulAttempt = attempt;

        % Number of random restarts actually performed.
        %
        % attempt 1 -> 0 restarts
        % attempt 2 -> 1 restart
        % attempt 3 -> 2 restarts
        % etc.

        info.randomRestarts = attempt - 1;


        % Tells us whether ADLS succeeded without needing
        % the restart mechanism at all.

        info.initialSeedSucceeded = ...
            (attempt == 1);


        info.finalSolverInfo = solverInfo;


        %% -----------------------------------------------------
        % Trim diagnostic arrays to attempts actually performed
        % ------------------------------------------------------

        info.attemptSeeds = ...
            attemptSeeds(:,1:attempt);

        info.attemptConverged = ...
            attemptConverged(1:attempt);

        info.attemptIterations = ...
            attemptIterations(1:attempt);


        return

    end

end


%% =============================================================
% Every allowed attempt failed
% =============================================================

qSolution = qLast;


info.converged = false;

info.randomRestarts = maxRandomRestarts;

info.successfulAttempt = 0;

info.initialSeedSucceeded = false;

info.finalSolverInfo = lastSolverInfo;


info.attemptSeeds = ...
    attemptSeeds(:,1:maxAttempts);

info.attemptConverged = ...
    attemptConverged(1:maxAttempts);

info.attemptIterations = ...
    attemptIterations(1:maxAttempts);

end