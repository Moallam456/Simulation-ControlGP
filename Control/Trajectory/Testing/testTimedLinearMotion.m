%% Timed Straight-Line Cartesian Motion Test
%
% PURPOSE
% -------
% This script connects the time-scaling methods to the Cartesian
% straight-line path and then to the robot.
%
% Complete pipeline:
%
%       Time-scaling method
%               ↓
%             s(t)
%               ↓
%       linearInterpolation()
%               ↓
%             p(t)
%               ↓
%       Construct TCP poses
%               ↓
%              IK
%               ↓
%             q(t)
%               ↓
%              FK
%               ↓
%       Validate + analyze
%               ↓
%          Animate robot
%
%
% Unlike the previous test, time now has physical meaning.
%
% Previously:
%
%       s = linspace(0,1,N)
%
% was only geometric sampling.
%
% Here:
%
%       s = s(t)
%
% comes from cubic, quintic, trapezoidal, or S-curve time scaling.
%
%
% IMPORTANT
% ---------
% The rigidBodyTree is used only for visualization.
%
% Path generation:
%       linearInterpolation()
%
% Kinematics:
%       controlFK()
%       controlIK()
%
% are still our own Control-team implementations.


clear
clc
close all


%% ========================================================================
%  1. SELECT TIME-SCALING METHOD
% =========================================================================

% Change ONLY this variable to test another profile:
%
%   "cubic"
%   "quintic"
%   "trapezoidal"
%   "scurve"

profileType = "cubic";


%% ========================================================================
%  2. PROFILE PARAMETERS
% =========================================================================

% Sampling period.
%
% This determines the time spacing between generated trajectory samples.
%
% Example:
%
%   dt = 0.05 s
%
% means:
%
%   20 trajectory samples per second.
%
% This is suitable for MATLAB testing.
%
% It is NOT yet the final physical controller sampling period.

dt = 0.05;


% ------------------------------------------------------------------------
% Cubic / Quintic parameters
% ------------------------------------------------------------------------
%
% These profiles are specified primarily by total trajectory duration T.

T_polynomial = 4.0;     % [s]


% ------------------------------------------------------------------------
% Trapezoidal / S-curve normalized path limits
% ------------------------------------------------------------------------
%
% IMPORTANT:
%
% These limits belong to s(t), NOT directly to the TCP.
%
% Units:
%
%   sDotMax   -> [1/s]
%   sDDotMax  -> [1/s^2]
%   sDDDotMax -> [1/s^3]
%
% Later in this script we convert them into physical TCP velocity,
% acceleration, and jerk using the actual Cartesian line length.

sDotMax   = 0.4;        % [1/s]
sDDotMax  = 0.4;        % [1/s^2]
sDDDotMax = 1.0;        % [1/s^3]


%% ========================================================================
%  3. LOAD ROBOT CONFIGURATION
% =========================================================================

% Load canonical UR5 data.
%
% This includes:
%
%   - DH parameters
%   - joint limits
%   - maximum joint velocity
%   - TCP definition

robot = config.UR5();


%% ========================================================================
%  4. BUILD MATLAB VISUALIZATION MODEL
% =========================================================================

% Convert our canonical RobotConfig into MATLAB's rigidBodyTree.
%
% Again:
%
% This object is NOT being used to calculate IK.
%
% It is only being used later to DRAW the robot.

refRobot = robotmodel.buildRobot(robot);


%% ========================================================================
%  5. DEFINE STARTING ROBOT CONFIGURATION
% =========================================================================

% Choose a known reachable starting configuration.
%
% Values are written in degrees for readability and converted to radians.

qStart = deg2rad([
     30
    -45
     60
     20
    -30
     45
]);


%% ========================================================================
%  6. CALCULATE STARTING TCP POSE
% =========================================================================

% Use our FK:
%
%       qStart
%          ↓
%      controlFK
%          ↓
%       TStart

TStart = controlFK(robot,qStart);


% Extract TCP position.

pStart = TStart(1:3,4);


% Extract TCP orientation.
%
% For this experiment, orientation remains constant for the whole line.

RStart = TStart(1:3,1:3);


%% ========================================================================
%  7. DEFINE ENDPOINT B
% =========================================================================

% Move the TCP 10 cm along Base +X.
%
% Because only X changes:
%
%       pEnd - pStart = [0.10; 0; 0]
%
% The desired geometric path is therefore a straight horizontal line
% in Base-frame X.

pEnd = pStart + [
    0.10
    0
    0
];


%% ========================================================================
%  8. CALCULATE CARTESIAN PATH LENGTH
% =========================================================================

% Vector from A to B.

deltaP = pEnd - pStart;


% Euclidean line length:
%
%       L = ||pEnd - pStart||
%
% For this test it should be 0.10 m.

pathLength = norm(deltaP);


fprintf("\n");
fprintf("========================================\n");
fprintf("CARTESIAN PATH\n");
fprintf("========================================\n");
fprintf("Path length: %.4f m\n",pathLength);
fprintf("========================================\n");


%% ========================================================================
%  9. GENERATE TIME SCALING
% =========================================================================

% This is the ONLY part of the main pipeline that changes when comparing
% the different time-scaling methods.
%
% Every method must produce:
%
%       t
%       s
%       sDot
%       sDDot
%       sDDDot
%
% Everything after this section is identical.


switch lower(profileType)

    case "cubic"

        [t, s, sDot, sDDot, sDDDot, profileInfo] = ...
            cubicTimeScaling( ...
                T_polynomial, ...
                dt);


    case "quintic"

        [t, s, sDot, sDDot, sDDDot, profileInfo] = ...
            quinticTimeScaling( ...
                T_polynomial, ...
                dt);


    case "trapezoidal"

        [t, s, sDot, sDDot, sDDDot, profileInfo] = ...
            trapezoidalTimeScaling( ...
                sDotMax, ...
                sDDotMax, ...
                dt);


    case "scurve"

        [t, s, sDot, sDDot, sDDDot, profileInfo] = ...
            sCurveTimeScaling( ...
                sDotMax, ...
                sDDotMax, ...
                sDDDotMax, ...
                dt);


    otherwise

        error( ...
            "Unknown profileType. Use cubic, quintic, trapezoidal, or scurve.");

end


% Number of timed trajectory samples.

numSamples = length(t);


fprintf("\n");
fprintf("========================================\n");
fprintf("TIME PROFILE\n");
fprintf("========================================\n");
fprintf("Profile:      %s\n",profileInfo.type);
fprintf("Total time:   %.3f s\n",profileInfo.T);
fprintf("Samples:      %d\n",numSamples);
fprintf("Nominal dt:   %.3f s\n",dt);
fprintf("========================================\n");


%% ========================================================================
%  10. GENERATE TIMED CARTESIAN PATH
% =========================================================================

% Previously we used:
%
%       s = linspace(0,1,N)
%
% Now s came from:
%
%       s = s(t)
%
% Therefore this equation:
%
%       p = pStart + s(pEnd - pStart)
%
% now produces:
%
%       p(t)
%
% instead of merely p(s).

pPath = linearInterpolation( ...
    pStart, ...
    pEnd, ...
    s);


%% ========================================================================
%  11. CALCULATE CARTESIAN VELOCITY, ACCELERATION, AND JERK
% =========================================================================

% For a straight path:
%
%       p(t) = pStart + deltaP * s(t)
%
% Therefore:
%
%       pDot(t)   = deltaP * sDot(t)
%
%       pDDot(t)  = deltaP * sDDot(t)
%
%       pDDDot(t) = deltaP * sDDDot(t)
%
%
% deltaP is 3x1.
%
% sDot is 1xN.
%
% Therefore the product gives a 3xN Cartesian velocity trajectory.


pDot = deltaP * sDot;

pDDot = deltaP * sDDot;

pDDDot = deltaP * sDDDot;


% Calculate the MAGNITUDE of each Cartesian vector.
%
% Example:
%
%       tcpSpeed(i)
%
% is the physical TCP speed at time sample i in m/s.

tcpSpeed = vecnorm(pDot,2,1);

tcpAcceleration = vecnorm(pDDot,2,1);

tcpJerk = vecnorm(pDDDot,2,1);


% Ignore NaN jerk entries when finding peaks.
%
% Trapezoidal profiles intentionally use NaN at acceleration
% discontinuities because ideal jerk is undefined/infinite there.

finiteTcpJerk = tcpJerk(isfinite(tcpJerk));


fprintf("\n");
fprintf("========================================\n");
fprintf("TCP MOTION FROM TIME SCALING\n");
fprintf("========================================\n");

fprintf("Peak TCP speed:        %.4f m/s\n", ...
    max(tcpSpeed));

fprintf("Peak TCP acceleration: %.4f m/s^2\n", ...
    max(tcpAcceleration));

if ~isempty(finiteTcpJerk)

    fprintf("Peak finite TCP jerk:  %.4f m/s^3\n", ...
        max(finiteTcpJerk));

else

    fprintf("Peak finite TCP jerk:  undefined\n");

end

fprintf("========================================\n");


%% ========================================================================
%  12. CONSTRUCT COMPLETE TCP POSES
% =========================================================================

% linearInterpolation gives only position:
%
%       p(t)
%
% But IK requires:
%
%              [ R   p ]
%       T(t) = [       ]
%              [ 0   1 ]
%
%
% For this first timed experiment:
%
%       R(t) = RStart
%
% so the tool keeps the same orientation throughout the line.


TPath = zeros(4,4,numSamples);


for i = 1:numSamples

    % Start with identity matrix.

    TTarget = eye(4);


    % Keep orientation constant.

    TTarget(1:3,1:3) = RStart;


    % Insert timed Cartesian position.

    TTarget(1:3,4) = pPath(:,i);


    % Store complete target pose.

    TPath(:,:,i) = TTarget;

end


%% ========================================================================
%  13. SOLVE IK ALONG TIMED PATH
% =========================================================================

% qPath will become:
%
%       q(t)
%
% with dimensions:
%
%       6 x N
%
% Each column is the complete six-joint configuration at one time sample.


qPath = zeros(robot.dof,numSamples);


% Diagnostic data from IK.

positionErrors    = zeros(1,numSamples);
orientationErrors = zeros(1,numSamples);
iterations        = zeros(1,numSamples);
converged         = false(1,numSamples);


% First IK solve starts from the known initial configuration.

qSeed = qStart;


for i = 1:numSamples

    % Current desired TCP pose.

    TTarget = TPath(:,:,i);


    % --------------------------------------------------------------
    % Solve IK
    % --------------------------------------------------------------

    [qSolution,info] = controlIK( ...
        robot, ...
        TTarget, ...
        qSeed);


    % --------------------------------------------------------------
    % Store diagnostics
    % --------------------------------------------------------------

    positionErrors(i) = info.positionError;

    orientationErrors(i) = info.orientationError;

    iterations(i) = info.iterations;

    converged(i) = info.converged;


    % --------------------------------------------------------------
    % Stop immediately if IK fails
    % --------------------------------------------------------------

    if ~info.converged

        error( ...
            "IK failed at sample %d, time %.3f s.", ...
            i, ...
            t(i));

    end


    % Store solved joint configuration.

    qPath(:,i) = qSolution;


    % --------------------------------------------------------------
    % Sequential IK seed
    % --------------------------------------------------------------
    %
    % Use the current solution as the starting guess for the next one.
    %
    % This encourages:
    %
    %       q_i ≈ q_(i+1)
    %
    % and reduces the chance of jumping between IK branches.

    qSeed = qSolution;

end


%% ========================================================================
%  14. VERIFY IK USING FORWARD KINEMATICS
% =========================================================================

% Check:
%
%       desired p(t)
%
% against:
%
%       FK(IK(desired T(t)))
%
%
% This validates the entire geometric + IK pipeline.


pAchieved = zeros(3,numSamples);


for i = 1:numSamples

    TAchieved = controlFK( ...
        robot, ...
        qPath(:,i));

    pAchieved(:,i) = TAchieved(1:3,4);

end


% Cartesian position error at every timed sample.

cartesianError = vecnorm( ...
    pAchieved - pPath, ...
    2, ...
    1);


fprintf("\n");
fprintf("========================================\n");
fprintf("IK / FK VALIDATION\n");
fprintf("========================================\n");

fprintf("Successful samples:        %d / %d\n", ...
    sum(converged), ...
    numSamples);

fprintf("Maximum Cartesian error:  %.3e m\n", ...
    max(cartesianError));

fprintf("Maximum IK position error: %.3e m\n", ...
    max(positionErrors));

fprintf("Maximum orientation error: %.3e rad\n", ...
    max(orientationErrors));

fprintf("========================================\n");


%% ========================================================================
%  15. CALCULATE JOINT VELOCITY qDot(t)
% =========================================================================

% This is an important new step.
%
% Because qPath now has actual corresponding TIMES, we can estimate:
%
%       dq/dt
%
% rather than only:
%
%       change in q per sample.
%
%
% We use MATLAB's gradient() instead of simple diff().
%
% gradient() is useful here because:
%
%   - it keeps the output the same length as qPath
%   - it handles the actual time vector t
%   - it uses central differences for interior samples


qDot = zeros(size(qPath));


for joint = 1:robot.dof

    qDot(joint,:) = gradient( ...
        qPath(joint,:), ...
        t);

end


%% ========================================================================
%  16. CALCULATE JOINT ACCELERATION qDDot(t)
% =========================================================================

% Differentiate joint velocity:
%
%       qDDot = d(qDot)/dt

qDDot = zeros(size(qPath));


for joint = 1:robot.dof

    qDDot(joint,:) = gradient( ...
        qDot(joint,:), ...
        t);

end


%% ========================================================================
%  17. CALCULATE JOINT JERK qDDDot(t)
% =========================================================================

% Differentiate acceleration:
%
%       qDDDot = d(qDDot)/dt
%
%
% IMPORTANT:
%
% Numerical differentiation amplifies noise.
%
% Therefore qDDDot is mainly useful here as an exploratory metric.
%
% Do not interpret tiny spikes as exact physical robot jerk.

qDDDot = zeros(size(qPath));


for joint = 1:robot.dof

    qDDDot(joint,:) = gradient( ...
        qDDot(joint,:), ...
        t);

end


%% ========================================================================
%  18. CHECK JOINT POSITION LIMITS
% =========================================================================

% Verify that every solved configuration remains within the UR5
% position limits.


qMin = robot.limits.qMin;

qMax = robot.limits.qMax;


jointPositionViolation = false(robot.dof,1);


for joint = 1:robot.dof

    jointPositionViolation(joint) = ...
        any(qPath(joint,:) < qMin(joint)) || ...
        any(qPath(joint,:) > qMax(joint));

end


%% ========================================================================
%  19. CHECK JOINT VELOCITY LIMITS
% =========================================================================

% UR5 configuration currently contains:
%
%       robot.limits.qdMax
%
% so we CAN perform a real velocity-limit check.


peakJointVelocity = max(abs(qDot),[],2);


jointVelocityViolation = ...
    peakJointVelocity > robot.limits.qdMax;


fprintf("\n");
fprintf("========================================\n");
fprintf("JOINT MOTION SUMMARY\n");
fprintf("========================================\n");


for joint = 1:robot.dof

    fprintf( ...
        "J%d | peak qDot = %.4f rad/s | limit = %.4f rad/s | violation = %d\n", ...
        joint, ...
        peakJointVelocity(joint), ...
        robot.limits.qdMax(joint), ...
        jointVelocityViolation(joint));

end


fprintf("\n");

fprintf("Peak joint accelerations [rad/s^2]:\n");

disp(max(abs(qDDot),[],2));


fprintf("Peak numerical joint jerk [rad/s^3]:\n");

disp(max(abs(qDDDot),[],2));


% The current UR5 config deliberately has no trusted acceleration limits.
%
% Therefore we REPORT acceleration and jerk, but do NOT classify them as:
%
%       PASS / FAIL
%
% yet.

if isempty(robot.limits.qddMax)

    fprintf( ...
        "Joint acceleration limits are not currently defined in RobotConfig.\n");

end


fprintf("========================================\n");


%% ========================================================================
%  20. PLOT TIME-SCALING PROFILE
% =========================================================================

figure

tiledlayout(4,1)


nexttile

plot(t,s,'LineWidth',1.5)

grid on

ylabel('s')

title("Time Scaling - " + profileInfo.type)


nexttile

plot(t,sDot,'LineWidth',1.5)

grid on

ylabel('ds/dt [1/s]')


nexttile

plot(t,sDDot,'LineWidth',1.5)

grid on

ylabel('d^2s/dt^2 [1/s^2]')


nexttile

plot(t,sDDDot,'LineWidth',1.5)

grid on

ylabel('d^3s/dt^3 [1/s^3]')

xlabel('Time [s]')


%% ========================================================================
%  21. PLOT PHYSICAL TCP MOTION
% =========================================================================

% Now we display quantities in physical Cartesian units:
%
%       m/s
%       m/s^2
%       m/s^3


figure

tiledlayout(3,1)


nexttile

plot(t,tcpSpeed,'LineWidth',1.5)

grid on

ylabel('Speed [m/s]')

title("TCP Motion - " + profileInfo.type)


nexttile

plot(t,tcpAcceleration,'LineWidth',1.5)

grid on

ylabel('Acceleration [m/s^2]')


nexttile

plot(t,tcpJerk,'LineWidth',1.5)

grid on

ylabel('Jerk [m/s^3]')

xlabel('Time [s]')


%% ========================================================================
%  22. PLOT JOINT POSITIONS
% =========================================================================

figure

plot( ...
    t, ...
    rad2deg(qPath.'), ...
    'LineWidth',1.2)

grid on

xlabel('Time [s]')

ylabel('Joint angle [deg]')

title("Joint Positions - " + profileInfo.type)

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% ========================================================================
%  23. PLOT JOINT VELOCITIES
% =========================================================================

figure

plot( ...
    t, ...
    qDot.', ...
    'LineWidth',1.2)

grid on

xlabel('Time [s]')

ylabel('Joint velocity [rad/s]')

title("Joint Velocities - " + profileInfo.type)

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% ========================================================================
%  24. PLOT JOINT ACCELERATIONS
% =========================================================================

figure

plot( ...
    t, ...
    qDDot.', ...
    'LineWidth',1.2)

grid on

xlabel('Time [s]')

ylabel('Joint acceleration [rad/s^2]')

title("Joint Accelerations - " + profileInfo.type)

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% ========================================================================
%  25. PLOT JOINT JERK
% =========================================================================

figure

plot( ...
    t, ...
    qDDDot.', ...
    'LineWidth',1.2)

grid on

xlabel('Time [s]')

ylabel('Joint jerk [rad/s^3]')

title("Numerical Joint Jerk - " + profileInfo.type)

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% ========================================================================
%  26. PLOT DESIRED VS ACHIEVED CARTESIAN PATH
% =========================================================================

figure


plot3( ...
    pPath(1,:), ...
    pPath(2,:), ...
    pPath(3,:), ...
    'o-', ...
    'LineWidth',1.5);

hold on


plot3( ...
    pAchieved(1,:), ...
    pAchieved(2,:), ...
    pAchieved(3,:), ...
    'x--', ...
    'LineWidth',1.5);


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

title("Timed Cartesian Path - " + profileInfo.type)

legend( ...
    'Desired path', ...
    'IK + FK achieved path', ...
    'A', ...
    'B');


%% ========================================================================
%  27. ANIMATE TIMED ROBOT MOTION
% =========================================================================

% We now animate q(t).
%
% Because we have an actual time vector, the pause between frames is based
% on the trajectory time difference.
%
% IMPORTANT:
%
% MATLAB rendering itself takes some time.
%
% Therefore this is an APPROXIMATE visual playback of the requested timing,
% not a real-time robot controller or dynamic simulation.


figure


ax = axes;


% Display first robot configuration.

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

title(ax, ...
    "UR5 Timed Linear Motion - " + profileInfo.type);


% Draw complete desired Cartesian path.

plot3( ...
    ax, ...
    pPath(1,:), ...
    pPath(2,:), ...
    pPath(3,:), ...
    'LineWidth',2);


% Draw A and B.

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


% Playback multiplier:
%
%   1.0 -> nominal trajectory timing
%   2.0 -> twice as slow
%   0.5 -> twice as fast
%
% This only affects visualization.

playbackScale = 1.0;


for i = 1:numSamples

    % Draw robot configuration q(t_i).

    show( ...
        refRobot, ...
        qPath(:,i), ...
        'Parent',ax, ...
        'FastUpdate',true, ...
        'PreservePlot',false);


    drawnow


    % Pause until approximately the next trajectory sample.
    %
    % The final sample has no following interval.

    if i < numSamples

        frameDuration = ...
            t(i+1) - t(i);

        pause( ...
            playbackScale * frameDuration);

    end

end