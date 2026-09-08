function [qSolution, info] = ADLS_IK( ...
    robot, ...
    T_B_TCP_target, ...
    qSeed, ...
    lambdaMax, ...
    sigmaThreshold)
%ADLS_IK Numerical inverse kinematics using Adaptive Damped Least Squares.
%
% This implementation uses the adaptive damping law presented by
% Yang et al. (2026).
%
% The solver keeps the same kinematic architecture used by controlIK:
%
%   - controlFK()       -> current TCP pose
%   - controlJacobian() -> geometric Jacobian
%   - [v; omega] = J*q_dot
%
% The main difference from controlIK is that the damping factor is NOT
% constant. Instead, it changes according to the smallest singular value
% of the Jacobian.
%
% Inputs:
%   robot              - Shared robot configuration
%   T_B_TCP_target     - Desired 4x4 TCP pose expressed in Base frame
%   qSeed              - Initial joint configuration, DOFx1 [rad]
%
% Outputs:
%   qSolution          - Joint solution [rad]
%   info               - Solver diagnostic information


%% -------------------------------------------------------------
% Solver parameters
% --------------------------------------------------------------

% Keep these equal to controlIK during the first benchmark.
%
% This ensures that when we compare:
%
%       fixed DLS vs ADLS
%
% the damping strategy is the main variable being changed.

maxIterations = 1000;

positionTolerance = 1e-4;       % [m]
orientationTolerance = 1e-4;    % [rad]

stepSize = 0.5;


%% -------------------------------------------------------------
% Yang et al. ADLS parameters
% --------------------------------------------------------------

% Yang uses lambda = 0.1 as the benchmark / maximum damping value.
%
% When sigmaMin -> 0:
%
%       lambdaDLS -> lambdaMax
%


% Singularity-region threshold.
%
% Yang's singularity-avoidance simulation uses:
%
%       c = 0.06
%
% When:
%
%       sigmaMin > c
%
% the Jacobian is considered sufficiently far from the singular region
% and no damping is applied.
%
% IMPORTANT:
% This is the value used by Yang's robot experiment/simulation.
% It is NOT yet tuned specifically for our robot.

%% Optional ADLS parameters

if nargin < 4 || isempty(lambdaMax)
    lambdaMax = 0.1;
end

if nargin < 5 || isempty(sigmaThreshold)
    sigmaThreshold = 1e-4;
end


%% -------------------------------------------------------------
% Read robot limits
% --------------------------------------------------------------

qMin = robot.limits.qMin(:);
qMax = robot.limits.qMax(:);


%% -------------------------------------------------------------
% Validate inputs
% --------------------------------------------------------------

q = qSeed(:);

if numel(q) ~= robot.dof
    error("qSeed must contain one value for each robot joint.");
end

if ~isequal(size(T_B_TCP_target), [4 4])
    error("T_B_TCP_target must be a 4x4 homogeneous transformation.");
end


%% -------------------------------------------------------------
% Ensure initial configuration satisfies joint limits
% --------------------------------------------------------------

q = min(max(q, qMin), qMax);


%% -------------------------------------------------------------
% Extract desired TCP pose
% --------------------------------------------------------------

pTarget = T_B_TCP_target(1:3,4);
RTarget = T_B_TCP_target(1:3,1:3);


%% -------------------------------------------------------------
% Initialize output information
% --------------------------------------------------------------

info.converged = false;
info.iterations = 0;

info.positionError = inf;
info.orientationError = inf;

% ADLS-specific information
info.sigmaMin = NaN;
info.lambda = NaN;
info.rmsJointUpdate = NaN;


%% -------------------------------------------------------------
% History for studying / plotting the algorithm
% --------------------------------------------------------------
%
% These are not required by ADLS mathematically.
% They are stored so we can later plot:
%
%       sigmaMin vs iteration
%       lambda    vs iteration
%
% and actually observe the adaptive behavior.

sigmaMinHistory = nan(maxIterations,1);
lambdaHistory = nan(maxIterations,1);
rmsJointUpdateHistory = nan(maxIterations,1);

% Store the complete joint configuration at every IK iteration.
% Column 1 is the initial seed.
qHistory = nan(robot.dof, maxIterations + 1);
qHistory(:,1) = q;

%% =============================================================
% Iterative IK loop
% =============================================================

for iteration = 1:maxIterations


    %% ---------------------------------------------------------
    % 1. Calculate current TCP pose
    % ----------------------------------------------------------

    TCurrent = controlFK(robot, q);

    pCurrent = TCurrent(1:3,4);
    RCurrent = TCurrent(1:3,1:3);


    %% ---------------------------------------------------------
    % 2. Position error
    % ----------------------------------------------------------
    %
    % Our convention:
    %
    %       error = target - current
    %
    % Yang writes:
    %
    %       F(q) = current - target
    %
    % Therefore our joint update will use a PLUS sign rather
    % than the MINUS sign appearing in Yang's Eq. (23).

    positionErrorVector = pTarget - pCurrent;

    positionError = norm(positionErrorVector);


    %% ---------------------------------------------------------
    % 3. Orientation error
    % ----------------------------------------------------------
    %
    % Keep the same orientation-error formulation as controlIK
    % during the first ADLS benchmark.
    %
    % This is important scientifically:
    %
    %       We want to compare damping strategies,
    %       not change several algorithms at once.

%% ---------------------------------------------------------
% 3. Orientation error using SO(3) logarithm
% ----------------------------------------------------------
%
% Relative rotation from the current TCP orientation to the
% desired TCP orientation:
%
%       RError = RTarget * RCurrent'
%
% Because our geometric Jacobian is expressed in the Base frame:
%
%       [v_B; omega_B] = J * q_dot
%
% we construct the rotational error in the Base frame as well.
%
% The SO(3) matrix logarithm gives:
%
%       log(RError) = [u_hat] * theta
%
% where:
%
%       u_hat = unit rotation axis
%       theta = required rotation angle [rad]
%
% Converting the skew-symmetric matrix to a vector gives:
%
%       orientationErrorVector = theta * u_hat
%
% Therefore:
%
%       norm(orientationErrorVector) = theta
%
% unlike the previous formulation, whose magnitude was sin(theta).

RError = RTarget * RCurrent';

orientationErrorVector = rotationLogVector(RError);

orientationError = norm(orientationErrorVector);


    %% ---------------------------------------------------------
    % 4. Check Cartesian convergence
    % ----------------------------------------------------------

    if positionError <= positionTolerance && ...
       orientationError <= orientationTolerance

        info.converged = true;
        info.iterations = iteration;

        info.positionError = positionError;
        info.orientationError = orientationError;

        qSolution = q;

        % Remove unused preallocated history entries.
        info.sigmaMinHistory = ...
            sigmaMinHistory(1:max(iteration-1,0));

        info.lambdaHistory = ...
            lambdaHistory(1:max(iteration-1,0));

        info.rmsJointUpdateHistory = ...
             rmsJointUpdateHistory(1:max(iteration-1,0));
         % Complete joint path from seed to converged solution.
             info.qHistory = ...
              qHistory(:,1:iteration);
        return

    end


    %% ---------------------------------------------------------
    % 5. Calculate geometric Jacobian
    % ----------------------------------------------------------

    J = controlJacobian(robot, q);


    %% ---------------------------------------------------------
    % 6. Singular Value Decomposition
    % ----------------------------------------------------------
    %
    % Yang Eq. (24):
    %
    %       J = U * diag(sigma_i) * V'
    %
    % MATLAB's:
    %
    %       svd(J)
    %
    % returns the singular values directly.
    %
    % They are normally returned from largest to smallest.

    singularValues = svd(J);

    % Smallest singular value:
    %
    %       sigmaMin = min(sigma_i)
    %
    sigmaMin = min(singularValues);


    %% ---------------------------------------------------------
    % 7. Calculate Yang adaptive damping factor
    % ----------------------------------------------------------
    %
    % Yang Eq. (25):
    %
    %                       0                     sigmaMin > c
    %
    % lambdaDLS =
    %
    %                       lambdaMax *
    %                       sqrt(1-(sigmaMin/c)^2) otherwise
    %
    %
    % Therefore:
    %
    % Far from singularity:
    %
    %       sigmaMin > c
    %       lambdaDLS = 0
    %
    %
    % Approaching singularity:
    %
    %       sigmaMin decreases
    %       lambdaDLS increases
    %
    %
    % At singularity:
    %
    %       sigmaMin = 0
    %       lambdaDLS = lambdaMax

    if sigmaMin > sigmaThreshold

        lambdaDLS = 0;

    else

        % Ratio lies nominally between 0 and 1.
        ratio = sigmaMin / sigmaThreshold;

        % max(...,0) protects against tiny floating-point roundoff
        % producing a negative number inside sqrt().
        lambdaDLS = lambdaMax * ...
            sqrt(max(0, 1 - ratio^2));

    end


    %% ---------------------------------------------------------
    % 8. Construct Cartesian pose-error vector
    % ----------------------------------------------------------
    %
    % Geometric Jacobian convention:
    %
    %       [v; omega] = J*q_dot
    %
    % Therefore error ordering must match:
    %
    %       [translation error;
    %        orientation error]

    poseError = [
        positionErrorVector
        orientationErrorVector
    ];


    %% ---------------------------------------------------------
    % 9. Adaptive Damped Least-Squares update
    % ----------------------------------------------------------
    %
    % Yang Eq. (23):
    %
    % q(i+1) =
    % q(i) - (J'*J + lambdaDLS^2*I)^(-1) * J' * F(q)
    %
    %
    % Yang defines:
    %
    %       F = current - target
    %
    % We define:
    %
    %       poseError = target - current = -F
    %
    % so our update becomes:
    %
    %       dq =
    %       (J'*J + lambdaDLS^2*I)^(-1)
    %       * J' * poseError
    %
    %
    % DO NOT explicitly calculate inv().
    %
    % MATLAB "\" solves the linear system directly.

    A = J' * J + ...
        lambdaDLS^2 * eye(robot.dof);

    b = J' * poseError;

    dq = A \ b;


    %% ---------------------------------------------------------
    % 10. Apply joint-update scaling
    % ----------------------------------------------------------
    %
    % stepSize is retained from our fixed-DLS implementation so
    % the benchmark remains fair.
    %
    % Yang's published Eq. (23) itself does not contain this
    % extra scaling factor.

    dqApplied = stepSize * dq;


    %% ---------------------------------------------------------
    % 11. Calculate RMS joint update
    % ----------------------------------------------------------
    %
    % Yang's numerical-method flowchart uses:
    %
    % RMS(dq) = sqrt( (1/6) * sum(dq_i^2) )
    %
    % We store it for analysis.
    %
    % We are NOT yet using it as the main stopping condition,
    % because our fixed-DLS baseline uses Cartesian pose error
    % tolerances.

    rmsJointUpdate = sqrt(mean(dqApplied.^2));


    %% ---------------------------------------------------------
    % 12. Update joint configuration
    % ----------------------------------------------------------

    q = q + dqApplied;


    %% ---------------------------------------------------------
    % 13. Enforce joint limits
    % ----------------------------------------------------------

    q = min(max(q, qMin), qMax);


    % Record the new joint configuration after the update and
    % joint-limit enforcement.
    qHistory(:,iteration + 1) = q;
    %% ---------------------------------------------------------
    % 14. Store ADLS diagnostics
    % ----------------------------------------------------------

    sigmaMinHistory(iteration) = sigmaMin;
    lambdaHistory(iteration) = lambdaDLS;
    rmsJointUpdateHistory(iteration) = rmsJointUpdate;

    info.iterations = iteration;

    info.positionError = positionError;
    info.orientationError = orientationError;

    info.sigmaMin = sigmaMin;
    info.lambda = lambdaDLS;
    info.rmsJointUpdate = rmsJointUpdate;

end


%% =============================================================
% Maximum iterations reached without convergence
% =============================================================

qSolution = q;

info.converged = false;

info.sigmaMinHistory = ...
    sigmaMinHistory(1:maxIterations);

info.lambdaHistory = ...
    lambdaHistory(1:maxIterations);

info.rmsJointUpdateHistory = ...
    rmsJointUpdateHistory(1:maxIterations);

% Initial seed + all 1000 attempted updates.
info.qHistory = ...
    qHistory(:,1:maxIterations + 1);

end

function rotationVector = rotationLogVector(R)
%ROTATIONLOGVECTOR SO(3) logarithm expressed as a 3x1 rotation vector.
%
% Input:
%   R              - 3x3 rotation matrix
%
% Output:
%   rotationVector - theta * axis, 3x1 [rad]
%
% The returned vector is the vector form of:
%
%       log(R) = [axis] * theta
%
% Therefore:
%
%       norm(rotationVector) = theta
%
% where theta is the shortest rotation angle in [0, pi].
%
% This implementation follows the SO(3) matrix-logarithm formulation
% described in Modern Robotics by Lynch and Park.
%
% Special handling is required near theta = pi because:
%
%       sin(pi) = 0
%
% so the ordinary expression
%
%       theta/(2*sin(theta)) * (R - R')
%
% becomes numerically singular.


%% -------------------------------------------------------------
% Calculate rotation angle
% --------------------------------------------------------------

cosTheta = (trace(R) - 1) / 2;

% Floating-point roundoff may make cosTheta slightly larger than 1
% or slightly smaller than -1.
cosTheta = max(-1, min(1, cosTheta));

theta = acos(cosTheta);


%% -------------------------------------------------------------
% Case 1: Rotation is essentially zero
% -------------------------------------------------------------
%
% If theta ~= 0:
%
%       log(I) = 0
%
% so there is no rotational correction required.

smallAngleTolerance = 1e-8;

if theta < smallAngleTolerance

    rotationVector = zeros(3,1);

    return

end


%% -------------------------------------------------------------
% Case 2: Rotation is close to 180 degrees
% -------------------------------------------------------------
%
% The usual formula contains:
%
%       1 / sin(theta)
%
% which is numerically problematic near:
%
%       theta = pi
%
% We therefore extract the rotation axis directly from the
% rotation matrix, following the special pi-case used in the
% Modern Robotics MatrixLog3 algorithm.

piTolerance = 1e-6;

if abs(pi - theta) < piTolerance

    if (1 + R(3,3)) > smallAngleTolerance

        axis = ...
            1 / sqrt(2 * (1 + R(3,3))) * [
                R(1,3)
                R(2,3)
                1 + R(3,3)
            ];

    elseif (1 + R(2,2)) > smallAngleTolerance

        axis = ...
            1 / sqrt(2 * (1 + R(2,2))) * [
                R(1,2)
                1 + R(2,2)
                R(3,2)
            ];

    else

        axis = ...
            1 / sqrt(2 * (1 + R(1,1))) * [
                1 + R(1,1)
                R(2,1)
                R(3,1)
            ];

    end

    % Numerical protection.
    axis = axis / norm(axis);

    rotationVector = theta * axis;

    return

end


%% -------------------------------------------------------------
% Case 3: General rotation
% -------------------------------------------------------------
%
% For:
%
%       0 < theta < pi
%
% the SO(3) logarithm is:
%
%       log(R) =
%
%       theta
%       ---------------- * (R - R')
%       2*sin(theta)
%
% This produces:
%
%       [axis] * theta

so3Matrix = ...
    theta / (2 * sin(theta)) * ...
    (R - R');


%% -------------------------------------------------------------
% Convert skew-symmetric matrix to vector
% -------------------------------------------------------------
%
% If:
%
%       [w] =
%
%       [  0   -w3   w2
%         w3    0   -w1
%        -w2   w1    0 ]
%
% then:
%
%       w = [ [w](3,2)
%             [w](1,3)
%             [w](2,1) ]

rotationVector = [
    so3Matrix(3,2)
    so3Matrix(1,3)
    so3Matrix(2,1)
];

end

