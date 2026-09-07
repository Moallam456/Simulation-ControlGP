function result = reachability(robotModel, robot, workspaceReq)
%REACHABILITY Evaluate POSITION-ONLY Cartesian workspace reachability.
%
%   result = analysis.reachability(robotModel, robot, workspaceReq)
%
%   Each required Cartesian point is tested using generalized inverse
%   kinematics with a position-only TCP constraint.
%
%   No TCP orientation is imposed at this stage.
%
%   OUTPUT:
%       result.coveragePercent
%       result.numRequiredPoints
%       result.numReachablePoints
%       result.requiredPoints
%       result.reachableMask
%       result.solutions
%
%   NOTE:
%       This evaluates geometric position reachability only.
%       Welding orientation requirements will be added later.

%% ========================================================================
% 1. VALIDATE WORKSPACE REQUIREMENT
% =========================================================================

fields = ["xMin","xMax","yMin","yMax","zMin","zMax"];

for i = 1:numel(fields)

    if ~isfield(workspaceReq, fields(i))
        error("analysis:reachability:MissingRequirement", ...
            "Missing workspace field: %s", fields(i));
    end

    if ~isfinite(workspaceReq.(fields(i)))
        error("analysis:reachability:InvalidRequirement", ...
            "Workspace field %s must be finite.", fields(i));
    end

end

if workspaceReq.xMin >= workspaceReq.xMax || ...
   workspaceReq.yMin >= workspaceReq.yMax || ...
   workspaceReq.zMin >= workspaceReq.zMax

    error("analysis:reachability:InvalidBounds", ...
        "Workspace minimum must be smaller than maximum.");

end


%% ========================================================================
% 2. CREATE REQUIRED CARTESIAN GRID
% =========================================================================

nx = 10;
ny = 10;
nz = 10;

x = linspace(workspaceReq.xMin, workspaceReq.xMax, nx);
y = linspace(workspaceReq.yMin, workspaceReq.yMax, ny);
z = linspace(workspaceReq.zMin, workspaceReq.zMax, nz);

[X,Y,Z] = ndgrid(x,y,z);

requiredPoints = [ ...
    X(:), ...
    Y(:), ...
    Z(:) ...
];

numRequiredPoints = size(requiredPoints,1);


%% ========================================================================
% 3. CREATE POSITION-ONLY IK SOLVER
% =========================================================================

gik = generalizedInverseKinematics( ...
    "RigidBodyTree", robotModel, ...
    "ConstraintInputs", {"position"});

positionConstraint = ...
    constraintPositionTarget(char(robot.frames.tcp));

% Position tolerance of solver.
%
% This is now a TRUE Cartesian IK tolerance,
% not a random-workspace sampling tolerance.

positionConstraint.PositionTolerance = 1e-3;   % 1 mm


%% ========================================================================
% 4. PREALLOCATE RESULTS
% =========================================================================

reachableMask = false(numRequiredPoints,1);

solutions = NaN(robot.dof, numRequiredPoints);

positionError = NaN(numRequiredPoints,1);


%% ========================================================================
% 5. INITIAL GUESS
% =========================================================================

qSeed = zeros(robot.dof,1);


%% ========================================================================
% 6. TEST EVERY REQUIRED CARTESIAN POINT
% =========================================================================

for i = 1:numRequiredPoints

    targetPoint = requiredPoints(i,:);

    positionConstraint.TargetPosition = targetPoint;

    % Solve position-only IK
    [qSolution, solutionInfo] = gik( ...
        qSeed, ...
        positionConstraint);

    % --------------------------------------------------------------
    % Compute the ACTUAL achieved TCP position
    % --------------------------------------------------------------

    T_B_TCP = getTransform( ...
        robotModel, ...
        qSolution, ...
        char(robot.frames.tcp));

    achievedPoint = T_B_TCP(1:3,4).';

    errorDistance = norm(achievedPoint - targetPoint);

    positionError(i) = errorDistance;


    % --------------------------------------------------------------
    % Check solution
    % --------------------------------------------------------------

    withinJointLimits = ...
        all(qSolution >= robot.limits.qMin) && ...
        all(qSolution <= robot.limits.qMax);

    if errorDistance <= positionConstraint.PositionTolerance && ...
       withinJointLimits

        reachableMask(i) = true;

        solutions(:,i) = qSolution;

        % Use this solution as next seed.
        % Neighboring Cartesian points often have neighboring
        % joint configurations.
        qSeed = qSolution;

    else

        % Reset seed if solver fails badly
        qSeed = zeros(robot.dof,1);

    end

end


%% ========================================================================
% 7. CALCULATE COVERAGE
% =========================================================================

numReachablePoints = sum(reachableMask);

coveragePercent = ...
    100 * numReachablePoints / numRequiredPoints;


%% ========================================================================
% 8. STORE RESULTS
% =========================================================================

result.coveragePercent = coveragePercent;

result.numRequiredPoints = numRequiredPoints;

result.numReachablePoints = numReachablePoints;

result.numUnreachablePoints = ...
    numRequiredPoints - numReachablePoints;

result.requiredPoints = requiredPoints;

result.reachableMask = reachableMask;

result.positionError = positionError;

result.solutions = solutions;

end