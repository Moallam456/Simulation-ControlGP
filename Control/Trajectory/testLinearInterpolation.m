%% Straight-Line Cartesian Path + IK + Robot Animation Test
%
% PURPOSE
% -------
% This script tests the complete geometric straight-line pipeline:
%
%   starting joint configuration
%           ↓
%          FK
%           ↓
%   starting TCP pose
%           ↓
%   linearInterpolation()
%           ↓
%   Cartesian path positions
%           ↓
%   construct full TCP poses
%           ↓
%          IK
%           ↓
%   joint configurations
%           ↓
%          FK
%           ↓
%   validate achieved TCP path
%           ↓
%   animate MATLAB robot
%
%
% IMPORTANT
% ---------
% This is NOT yet a timed trajectory.
%
% We currently have:
%
%       p = p(s)
%
% and after IK:
%
%       q = q(s)
%
% We have NOT yet introduced:
%
%       s = s(t)
%
% Therefore the animation speed below is only for visualization.
% It is not a physical robot velocity profile.


clear
clc
close all


%% ========================================================================
%  1. LOAD ROBOT CONFIGURATION
% =========================================================================

% Load the shared UR5 configuration.
%
% This structure contains the robot-dependent information used by the
% Control code:
%
%   - degrees of freedom
%   - DH parameters
%   - joint limits
%   - TCP definition
%   - frame definitions

robot = config.UR5();


%% ========================================================================
%  2. BUILD MATLAB REFERENCE ROBOT
% =========================================================================

% buildRobot() converts the shared robot configuration into MATLAB's
% rigidBodyTree representation.
%
% IMPORTANT:
%
% We are NOT using this robot object to calculate our trajectory or IK.
%
% Our Control code still performs:
%
%   FK  -> controlFK()
%   IK  -> controlIK()
%
% The rigidBodyTree is mainly being used here so that MATLAB can visually
% draw and animate the robot.

refRobot = robotmodel.buildRobot(robot);


%% ========================================================================
%  3. CHOOSE STARTING JOINT CONFIGURATION
% =========================================================================

% Define a known robot configuration for waypoint A.
%
% Each value corresponds to one UR5 joint.
%
% We write the values in degrees because they are easier to read,
% then convert them to radians because our Control implementation uses rad.

qStart = deg2rad([
     30
    -45
     60
     20
    -30
     45
]);


%% ========================================================================
%  4. CALCULATE THE STARTING TCP POSE USING FK
% =========================================================================

% Forward kinematics converts:
%
%       joint configuration qStart
%
% into:
%
%       TCP pose TStart
%
% expressed in the robot Base frame.

TStart = controlFK(robot, qStart);


% A homogeneous transformation has the form:
%
%          [ R   p ]
%      T = [       ]
%          [ 0   1 ]
%
% R = 3x3 TCP orientation
% p = 3x1 TCP position


% Extract Cartesian starting position.
pStart = TStart(1:3,4);


% Extract starting orientation.
%
% We will keep this orientation CONSTANT for the whole first test.
%
% This means:
%
%   the TCP translates along the line,
%   but the tool does not rotate.

RStart = TStart(1:3,1:3);


%% ========================================================================
%  5. DEFINE CARTESIAN ENDPOINT B
% =========================================================================

% Define waypoint B as a simple displacement from A.
%
% Here:
%
%       +0.10 m in Base X
%        0.00 m in Base Y
%        0.00 m in Base Z
%
% Therefore this is a 10 cm straight Cartesian movement.

pEnd = pStart + [
    0.10
    0
    0
];


%% ========================================================================
%  6. DEFINE GEOMETRIC PATH PARAMETER s
% =========================================================================

% s is NOT time.
%
% It only tells us how far along the geometric path we are:
%
%       s = 0       -> exactly at A
%       s = 0.5     -> halfway between A and B
%       s = 1       -> exactly at B
%
% 21 points gives:
%
%       0, 0.05, 0.10, ..., 0.95, 1

numSamples = 21;

s = linspace(0,1,numSamples);


%% ========================================================================
%  7. GENERATE STRAIGHT CARTESIAN PATH
% =========================================================================

% Use OUR path-generation function:
%
%       p(s) = pStart + s (pEnd - pStart)
%
% Output dimensions:
%
%       pPath = 3 x N
%
% Every COLUMN is one Cartesian TCP position:
%
%       pPath(:,1)   -> first point
%       pPath(:,2)   -> second point
%       ...
%       pPath(:,end) -> final point

pPath = linearInterpolation(pStart, pEnd, s);


%% ========================================================================
%  8. BASIC GEOMETRIC VALIDATION
% =========================================================================

% Before involving IK, verify that the path generator satisfies its
% fundamental endpoint conditions:
%
%       p(0) = pStart
%       p(1) = pEnd

startPathError = norm(pPath(:,1)   - pStart);
endPathError   = norm(pPath(:,end) - pEnd);

fprintf("\n");
fprintf("========================================\n");
fprintf("LINEAR INTERPOLATION CHECK\n");
fprintf("========================================\n");
fprintf("Start-point error: %.3e m\n", startPathError);
fprintf("End-point error:   %.3e m\n", endPathError);
fprintf("========================================\n");


%% ========================================================================
%  9. BUILD A FULL TARGET POSE FOR EVERY PATH POINT
% =========================================================================

% linearInterpolation() gives us POSITION only:
%
%       p1, p2, p3, ..., pN
%
% But inverse kinematics requires a complete TCP POSE:
%
%             [ R_i   p_i ]
%       T_i = [           ]
%             [  0     1  ]
%
%
% For this first experiment:
%
%       R_i = RStart
%
% for every path point.
%
% So POSITION changes,
% but ORIENTATION remains constant.


% Preallocate a 4x4xN array.
%
% TPath(:,:,1) is pose 1
% TPath(:,:,2) is pose 2
% etc.

TPath = zeros(4,4,numSamples);


for i = 1:numSamples

    % Start with an identity transformation.
    TTarget = eye(4);

    % Keep the original TCP orientation.
    TTarget(1:3,1:3) = RStart;

    % Insert the current interpolated Cartesian position.
    TTarget(1:3,4) = pPath(:,i);

    % Store complete target pose.
    TPath(:,:,i) = TTarget;

end


%% ========================================================================
%  10. SOLVE IK ALONG THE COMPLETE CARTESIAN PATH
% =========================================================================

% For every target pose:
%
%       TPath(:,:,i)
%
% solve:
%
%       IK -> q_i
%
% The resulting qPath will have dimensions:
%
%       6 x N
%
% Every COLUMN represents one complete robot configuration.


qPath = zeros(robot.dof,numSamples);


% Store useful solver information for later analysis.
positionErrors    = zeros(1,numSamples);
orientationErrors = zeros(1,numSamples);
iterations        = zeros(1,numSamples);
converged         = false(1,numSamples);


% The first IK solution should start from the configuration we already know
% produces TStart.
%
% Therefore our initial seed is qStart.

qSeed = qStart;


for i = 1:numSamples

    % Current desired TCP pose.
    TTarget = TPath(:,:,i);


    % --------------------------------------------------------------------
    % Solve inverse kinematics
    % --------------------------------------------------------------------
    %
    % controlIK tries to find:
    %
    %       qSolution
    %
    % such that:
    %
    %       controlFK(robot,qSolution) ≈ TTarget

    [qSolution,info] = controlIK( ...
        robot, ...
        TTarget, ...
        qSeed);


    % Store solver information.
    positionErrors(i)    = info.positionError;
    orientationErrors(i) = info.orientationError;
    iterations(i)        = info.iterations;
    converged(i)         = info.converged;


    % Print what happened at this waypoint.
    fprintf( ...
        "Point %2d/%2d | Converged: %d | Pos error: %.3e m | Ori error: %.3e rad | Iter: %d\n", ...
        i, ...
        numSamples, ...
        info.converged, ...
        info.positionError, ...
        info.orientationError, ...
        info.iterations);


    % If one point cannot be solved, the path cannot currently be followed
    % continuously by this IK solver.
    %
    % Stop immediately rather than pretending the remaining test is valid.

    if ~info.converged
        error("IK failed at path sample %d.",i);
    end


    % Store joint configuration.
    qPath(:,i) = qSolution;


    % --------------------------------------------------------------------
    % VERY IMPORTANT: NEXT IK SEED
    % --------------------------------------------------------------------
    %
    % Do NOT restart every IK solve from qStart.
    %
    % Instead:
    %
    %       q1 becomes seed for q2
    %       q2 becomes seed for q3
    %       q3 becomes seed for q4
    %       ...
    %
    % Nearby Cartesian points normally correspond to nearby joint
    % configurations.
    %
    % This helps the numerical IK stay on the same solution branch.

    qSeed = qSolution;

end


%% ========================================================================
%  11. VERIFY IK SOLUTIONS USING FORWARD KINEMATICS
% =========================================================================

% IK claimed that each qPath(:,i) reaches TPath(:,:,i).
%
% We should not simply trust that.
%
% Send every solved configuration back through OUR FK:
%
%       desired pose
%           ↓
%          IK
%           ↓
%           q
%           ↓
%          FK
%           ↓
%       achieved pose
%
% Then compare desired vs achieved Cartesian positions.


pAchieved = zeros(3,numSamples);


for i = 1:numSamples

    % Calculate TCP pose actually produced by the solved joints.
    TAchieved = controlFK(robot,qPath(:,i));

    % Store its Cartesian position.
    pAchieved(:,i) = TAchieved(1:3,4);

end


% Cartesian error for every path point.
%
% vecnorm(...,2,1) calculates the Euclidean norm of every COLUMN.

cartesianError = vecnorm(pAchieved - pPath,2,1);


fprintf("\n");
fprintf("========================================\n");
fprintf("IK + FK PATH VALIDATION\n");
fprintf("========================================\n");
fprintf("Successful IK points: %d / %d\n", ...
    sum(converged),numSamples);

fprintf("Maximum FK path error: %.3e m\n", ...
    max(cartesianError));

fprintf("Maximum IK position error: %.3e m\n", ...
    max(positionErrors));

fprintf("Maximum IK orientation error: %.3e rad\n", ...
    max(orientationErrors));

fprintf("========================================\n");


%% ========================================================================
%  12. JOINT-CONTINUITY CHECK
% =========================================================================

% A smooth Cartesian path does NOT automatically guarantee smooth joint
% solutions.
%
% Numerical IK could theoretically jump from one valid IK branch to another.
%
% Example:
%
%       q1 = 20 deg
%       q2 = 21 deg
%       q3 = 22 deg
%       q4 = 150 deg    <- suspicious jump
%
%
% We therefore inspect:
%
%       Delta q_i = q_i - q_(i-1)


deltaQ = diff(qPath,1,2);


% deltaQ has N-1 columns because differences exist BETWEEN samples.
%
% Convert to degrees only for easier human interpretation in the plots.

qPathDeg  = rad2deg(qPath);
deltaQDeg = rad2deg(deltaQ);


% Find largest absolute change between any two consecutive solutions.

maxJointStep = max(abs(deltaQDeg),[],"all");


fprintf("\n");
fprintf("========================================\n");
fprintf("JOINT CONTINUITY CHECK\n");
fprintf("========================================\n");
fprintf("Largest consecutive joint change: %.3f deg\n", ...
    maxJointStep);
fprintf("========================================\n");


%% ========================================================================
%  13. STATIC PATH VALIDATION PLOT
% =========================================================================

% This plot answers:
%
% "Did the TCP positions produced by IK + FK actually follow the path that
%  linearInterpolation requested?"


figure

% Desired Cartesian path.
plot3( ...
    pPath(1,:), ...
    pPath(2,:), ...
    pPath(3,:), ...
    'o-', ...
    'LineWidth',1.5);

hold on


% TCP positions obtained after:
%
%       desired path -> IK -> FK
%
% Ideally this line lies almost exactly on top of the desired path.

plot3( ...
    pAchieved(1,:), ...
    pAchieved(2,:), ...
    pAchieved(3,:), ...
    'x--', ...
    'LineWidth',1.5);


% Highlight A and B.

scatter3( ...
    pStart(1), ...
    pStart(2), ...
    pStart(3), ...
    80, ...
    'filled');

scatter3( ...
    pEnd(1), ...
    pEnd(2), ...
    pEnd(3), ...
    80, ...
    'filled');


grid on
axis equal

xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')

legend( ...
    'Desired linear path', ...
    'IK + FK achieved path', ...
    'Waypoint A', ...
    'Waypoint B');

title('Linear Cartesian Path Validation')


%% ========================================================================
%  14. PLOT JOINT CONFIGURATIONS ALONG THE PATH
% =========================================================================

% NOTE:
%
% The horizontal axis is PATH SAMPLE NUMBER, not time.
%
% We are checking geometric continuity of the IK solution.
%
% Later, once s(t) exists, we will plot q(t) instead.


figure

plot(1:numSamples,qPathDeg.','LineWidth',1.2)

grid on

xlabel('Path sample')
ylabel('Joint angle [deg]')

title('Joint Configurations Along Cartesian Path')

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% ========================================================================
%  15. PLOT CHANGE BETWEEN CONSECUTIVE JOINT SOLUTIONS
% =========================================================================

% This plot makes IK branch jumps easier to notice.
%
% Smooth small changes are expected.
%
% A large isolated spike deserves investigation.


figure

plot(2:numSamples,deltaQDeg.','LineWidth',1.2)

grid on

xlabel('Path sample')
ylabel('\Delta q [deg/sample]')

title('Change Between Consecutive IK Solutions')

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% ========================================================================
%  16. ANIMATE THE ROBOT FOLLOWING THE PATH
% =========================================================================

% THIS is the major extension beyond the old static test.
%
% We now have:
%
%       qPath(:,1)
%       qPath(:,2)
%       ...
%       qPath(:,N)
%
% so MATLAB finally knows how the robot should be configured at each
% Cartesian sample.


figure


% Create axes explicitly so that both the robot and path are drawn in
% the same coordinate system.

ax = axes;


% Show the robot at the FIRST solved configuration.
%
% FastUpdate = true:
%       allows repeated robot updates more efficiently.
%
% PreservePlot = false:
%       prevents MATLAB from leaving a copy of every previous robot pose.

show( ...
    refRobot, ...
    qPath(:,1), ...
    'Parent',ax, ...
    'FastUpdate',true, ...
    'PreservePlot',false);


hold(ax,'on')
grid(ax,'on')
axis(ax,'equal')

xlabel(ax,'X [m]')
ylabel(ax,'Y [m]')
zlabel(ax,'Z [m]')

title(ax,'UR5 Following Linear Cartesian Path')


% Draw desired line so that we can visually compare the moving TCP against
% the requested Cartesian path.

plot3( ...
    ax, ...
    pPath(1,:), ...
    pPath(2,:), ...
    pPath(3,:), ...
    'LineWidth',2);


% Highlight path endpoints.

scatter3( ...
    ax, ...
    pStart(1), ...
    pStart(2), ...
    pStart(3), ...
    80, ...
    'filled');

scatter3( ...
    ax, ...
    pEnd(1), ...
    pEnd(2), ...
    pEnd(3), ...
    80, ...
    'filled');


text( ...
    ax, ...
    pStart(1), ...
    pStart(2), ...
    pStart(3), ...
    '  A');

text( ...
    ax, ...
    pEnd(1), ...
    pEnd(2), ...
    pEnd(3), ...
    '  B');


% ------------------------------------------------------------------------
% Animation delay
% ------------------------------------------------------------------------
%
% This value ONLY changes how fast the visualization plays.
%
% It is NOT a physical trajectory timestep.
%
% We will replace this concept later when we implement:
%
%       s(t)
%
% and an actual trajectory sampling period.

animationPause = 0.08;


for i = 1:numSamples

    % Update the robot to the next solved joint configuration.

    show( ...
        refRobot, ...
        qPath(:,i), ...
        'Parent',ax, ...
        'FastUpdate',true, ...
        'PreservePlot',false);


    % Force MATLAB to update the figure immediately.
    drawnow


    % Slow the playback enough that the motion can be seen.
    pause(animationPause)

end