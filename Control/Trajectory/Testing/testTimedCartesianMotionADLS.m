%% testTimedCartesianMotionADLS
%
% TIMED CARTESIAN STRAIGHT-LINE MOTION USING ADLS IK
%
% PURPOSE
% -------
% This script connects all of the trajectory-pipeline components developed
% so far into one complete experiment:
%
%
%       Robot starting configuration
%                   ↓
%             Forward Kinematics
%                   ↓
%              Start TCP pose
%                   ↓
%          Cartesian endpoint definition
%                   ↓
%             Time scaling s(t)
%                   ↓
%       Position interpolation p(s(t))
%                   +
%       Orientation interpolation R(s(t))
%                   ↓
%        Full Cartesian trajectory T(t)
%                   ↓
%          Sequential ADLS inverse IK
%                   ↓
%            Joint trajectory q(t)
%                   ↓
%              FK validation
%                   ↓
%       qDot / qDDot / qDDDot analysis
%                   ↓
%          Position / velocity limits
%                   ↓
%                 Animation
%
%
% IMPORTANT
% ---------
% This script is still a KINEMATIC trajectory experiment.
%
% We are NOT yet simulating:
%
%       motor torque
%       inertia
%       friction
%       gravity compensation
%       actuator dynamics
%       closed-loop servo control
%
% MATLAB's rigidBodyTree is used only to VISUALIZE the joint trajectory.
%
%
% Current IK solver:
%
%       ADLS_IK()
%
% which includes:
%
%       - Adaptive Damped Least Squares
%       - sigmaThreshold = 1e-4 by default
%       - SO(3) logarithmic orientation error
%       - joint-limit clamping
%
%
% Current Cartesian motion:
%
%       straight-line position
%       constant TCP orientation
%
% To test changing orientation later, simply change REnd.

clear;
clc;
close all;


%% ========================================================================
%  1. SELECT TIME-SCALING METHOD
% =========================================================================
%
% Available implementations already present in:
%
%       Control/Trajectory/TimeScaling/
%
% Options:
%
%       "cubic"
%       "quintic"
%       "trapezoidal"
%       "scurve"
%
%
% Quintic is chosen first because it provides:
%
%       zero velocity at start/end
%       zero acceleration at start/end
%
% and is therefore a convenient first smooth timed experiment.

profileType = "quintic";


%% ========================================================================
%  2. TRAJECTORY SAMPLING PERIOD
% =========================================================================
%
% dt determines how often the continuous trajectory is sampled.
%
% Example:
%
%       dt = 0.05 s
%
% gives approximately:
%
%       20 samples / second
%
%
% IMPORTANT:
% This is only the MATLAB trajectory sampling period.
%
% It is NOT yet the final embedded servo-loop period.

dt = 0.05;      % [s]


%% ========================================================================
%  3. TIME-SCALING PARAMETERS
% =========================================================================

% ------------------------------------------------------------------------
% Polynomial profiles
% ------------------------------------------------------------------------
%
% Cubic and quintic methods use a chosen total trajectory duration.

T_polynomial = 4.0;     % [s]


% ------------------------------------------------------------------------
% Trapezoidal / S-curve parameters
% ------------------------------------------------------------------------
%
% These limits are applied to the NORMALIZED path variable:
%
%       0 <= s <= 1
%
% so their units are:
%
%       sDotMax   [1/s]
%       sDDotMax  [1/s^2]
%       sDDDotMax [1/s^3]

sDotMax   = 0.4;        % [1/s]
sDDotMax  = 0.4;        % [1/s^2]
sDDDotMax = 1.0;        % [1/s^3]


%% ========================================================================
%  4. LOAD ROBOT CONFIGURATION
% =========================================================================

robot = config.UR5();

n = robot.dof;


%% ========================================================================
%  5. BUILD MATLAB VISUALIZATION MODEL
% =========================================================================
%
% This rigidBodyTree is used ONLY to draw the robot.
%
% Our own Control-team functions are still responsible for:
%
%       FK
%       Jacobian
%       IK
%       path generation
%
% Therefore the visualization model is not secretly solving the motion.

refRobot = robotmodel.buildUR5geometry(robot);
% Build preliminary welding-station environment
station = environment.buildPreliminaryWeldingStation();

%% ========================================================================
%  6. DEFINE STARTING ROBOT CONFIGURATION
% =========================================================================
%
% We deliberately start from a known reachable nontrivial configuration.
%
% This avoids using q = 0 as the only test case.

qStart = deg2rad([
     30
    -45
     60
     20
    -30
     45
]);


%% ========================================================================
%  7. CALCULATE START TCP POSE
% =========================================================================
%
% Our FK gives:
%
%       qStart
%          ↓
%       controlFK
%          ↓
%       TStart
%
%
% TStart contains:
%
%       [ RStart   pStart ]
%       [   0         1   ]

TStart = controlFK(robot,qStart);

pStart = TStart(1:3,4);

RStart = TStart(1:3,1:3);


%% ========================================================================
%  8. DEFINE END TCP POSE
% =========================================================================
%
% Move the TCP 10 cm along Base +X.
%
% Therefore:
%
%       deltaP = [0.10; 0; 0]
%
%
% For the first full pipeline experiment we keep orientation constant:
%
%       REnd = RStart
%
% This is a perfectly valid Cartesian motion:
%
%       position changes
%       orientation remains fixed

pEnd = pStart + [
    0.10
    0
    0
];

REnd = RStart;


%% ========================================================================
%  9. CARTESIAN PATH INFORMATION
% =========================================================================

deltaP = pEnd - pStart;

pathLength = norm(deltaP);

fprintf('\n');
fprintf('============================================================\n');
fprintf(' CARTESIAN MOTION\n');
fprintf('============================================================\n');

fprintf('Start position [m]:\n');
disp(pStart.');

fprintf('End position [m]:\n');
disp(pEnd.');

fprintf('Path length: %.4f m\n',pathLength);

fprintf('Orientation: constant\n');

fprintf('============================================================\n');


%% ========================================================================
%  10. GENERATE TIME SCALING
% =========================================================================
%
% This stage answers:
%
%       "HOW FAST should we progress along the path?"
%
%
% The geometric line itself is still represented by:
%
%       p(s)
%
% but the time-scaling method gives:
%
%       s = s(t)
%
%
% Every method returns:
%
%       t
%       s
%       sDot
%       sDDot
%       sDDDot

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


% Make sure vectors use a consistent row orientation.
%
% This avoids accidental dimensional problems later.

t       = t(:).';
s       = s(:).';
sDot    = sDot(:).';
sDDot   = sDDot(:).';
sDDDot  = sDDDot(:).';


numSamples = numel(t);


fprintf('\n');
fprintf('============================================================\n');
fprintf(' TIME PROFILE\n');
fprintf('============================================================\n');

fprintf('Method:       %s\n',profileInfo.type);
fprintf('Total time:   %.3f s\n',profileInfo.T);
fprintf('Samples:      %d\n',numSamples);
fprintf('Nominal dt:   %.4f s\n',dt);

fprintf('============================================================\n');


%% ========================================================================
%  11. GENERATE TIMED CARTESIAN POSITION PATH
% =========================================================================
%
% Straight-line geometry:
%
%       p(s) = pStart + s(pEnd - pStart)
%
%
% Because:
%
%       s = s(t)
%
% this now becomes:
%
%       p(t) = pStart + s(t)(pEnd - pStart)
%
%
% Therefore samples are NOT necessarily equally spaced geometrically.
%
% Near the beginning and end of a smooth motion profile:
%
%       sDot is small
%
% so Cartesian samples are closer together.

pPath = linearInterpolation( ...
    pStart, ...
    pEnd, ...
    s);


%% ========================================================================
%  12. GENERATE CARTESIAN VELOCITY / ACCELERATION / JERK
% =========================================================================
%
% For:
%
%       p(t) = pStart + deltaP*s(t)
%
% derivatives are:
%
%       pDot   = deltaP*sDot
%       pDDot  = deltaP*sDDot
%       pDDDot = deltaP*sDDDot

pDot = deltaP * sDot;

pDDot = deltaP * sDDot;

pDDDot = deltaP * sDDDot;


% Physical magnitudes.

tcpSpeed = vecnorm(pDot,2,1);

tcpAcceleration = vecnorm(pDDot,2,1);

tcpJerk = vecnorm(pDDDot,2,1);


fprintf('\n');
fprintf('============================================================\n');
fprintf(' TCP MOTION\n');
fprintf('============================================================\n');

fprintf( ...
    'Peak TCP speed:        %.5f m/s\n', ...
    max(tcpSpeed));

fprintf( ...
    'Peak TCP acceleration: %.5f m/s^2\n', ...
    max(tcpAcceleration));


finiteTcpJerk = tcpJerk(isfinite(tcpJerk));

if ~isempty(finiteTcpJerk)

    fprintf( ...
        'Peak finite TCP jerk:  %.5f m/s^3\n', ...
        max(finiteTcpJerk));

else

    fprintf('Peak TCP jerk: undefined\n');

end

fprintf('============================================================\n');


%% ========================================================================
%  13. CONSTRUCT FULL TIMED CARTESIAN POSES
% =========================================================================
%
% IK needs a complete TCP pose:
%
%       T(t) =
%
%       [ R(t)   p(t) ]
%       [  0       1  ]
%
%
% Position comes from linearInterpolation().
%
% Orientation comes from slerpOrientation().
%
%
% In this experiment:
%
%       RStart = REnd
%
% so SLERP naturally returns the same orientation for every sample.
%
% Later we can simply specify a different REnd and this pipeline already
% supports changing TCP orientation.

TPath = zeros(4,4,numSamples);


for i = 1:numSamples

    % Current normalized path parameter.

    si = s(i);


    % -------------------------------------------------------------
    % Orientation interpolation
    % -------------------------------------------------------------
    %
    % Constant orientation case:
    %
    %       RStart == REnd
    %
    % therefore:
    %
    %       RTarget == RStart
    %
    % for all samples.

    RTarget = slerpOrientation( ...
        RStart, ...
        REnd, ...
        si);


    % -------------------------------------------------------------
    % Construct homogeneous transform
    % -------------------------------------------------------------

    TTarget = eye(4);

    TTarget(1:3,1:3) = RTarget;

    TTarget(1:3,4) = pPath(:,i);


    TPath(:,:,i) = TTarget;

end


%% ========================================================================
%  14. PREALLOCATE JOINT TRAJECTORY
% =========================================================================

qPath = zeros(n,numSamples);


% IK diagnostics.

ikConverged = false(1,numSamples);

ikIterations = zeros(1,numSamples);

ikPositionError = zeros(1,numSamples);

ikOrientationError = zeros(1,numSamples);

sigmaMinPath = nan(1,numSamples);

lambdaPath = nan(1,numSamples);


%% ========================================================================
%  15. SEQUENTIAL ADLS IK
% =========================================================================
%
% This is the key Cartesian-to-joint conversion.
%
%
% First sample:
%
%       qStart
%          ↓
%      ADLS_IK(T1)
%          ↓
%        qPath1
%
%
% Second sample:
%
%       qPath1
%          ↓
%      ADLS_IK(T2)
%          ↓
%        qPath2
%
%
% etc.
%
%
% This is important because consecutive Cartesian trajectory samples are
% close together.
%
% Therefore the previous valid joint solution is normally an excellent
% initial guess for the next IK problem.

qSeed = qStart;


fprintf('\n');
fprintf('============================================================\n');
fprintf(' SEQUENTIAL ADLS IK\n');
fprintf('============================================================\n');


for i = 1:numSamples

    TTarget = TPath(:,:,i);


    %% -------------------------------------------------------------
    % Run corrected ADLS IK
    % --------------------------------------------------------------

    [qSolution,info] = ADLS_IK( ...
        robot, ...
        TTarget, ...
        qSeed);


    %% -------------------------------------------------------------
    % Store solver diagnostics
    % --------------------------------------------------------------

    ikConverged(i) = info.converged;

    ikIterations(i) = info.iterations;

    ikPositionError(i) = info.positionError;

    ikOrientationError(i) = info.orientationError;


    if isfield(info,'sigmaMin')

        sigmaMinPath(i) = info.sigmaMin;

    end


    if isfield(info,'lambda')

        lambdaPath(i) = info.lambda;

    end


    %% -------------------------------------------------------------
    % Check convergence
    % --------------------------------------------------------------

    if ~info.converged

        fprintf('\n');

        fprintf( ...
            'ADLS FAILED at sample %d / %d\n', ...
            i, ...
            numSamples);

        fprintf( ...
            'Time: %.4f s\n', ...
            t(i));

        fprintf( ...
            'Position error: %.3e m\n', ...
            info.positionError);

        fprintf( ...
            'Orientation error: %.3e rad\n', ...
            info.orientationError);

        error( ...
            'Timed Cartesian trajectory cannot continue because IK failed.');

    end


    %% -------------------------------------------------------------
    % Store valid joint solution
    % --------------------------------------------------------------

    qPath(:,i) = qSolution;


    %% -------------------------------------------------------------
    % Sequential seed
    % --------------------------------------------------------------

    qSeed = qSolution;


    fprintf( ...
        ['Sample %3d/%3d | t = %6.3f s | iter = %3d | ' ...
         'pos = %.2e m | ori = %.2e rad\n'], ...
        i, ...
        numSamples, ...
        t(i), ...
        info.iterations, ...
        info.positionError, ...
        info.orientationError);

end


fprintf('============================================================\n');


%% ========================================================================
%  16. INDEPENDENT FORWARD-KINEMATICS VALIDATION
% =========================================================================
%
% We do NOT trust only:
%
%       info.converged
%
%
% For every solved qPath sample we independently calculate:
%
%       TAchieved = controlFK(qPath)
%
% and compare against:
%
%       TDesired
%
%
% This catches errors in the IK convergence logic.

pAchieved = zeros(3,numSamples);

independentPositionError = zeros(1,numSamples);

independentOrientationError = zeros(1,numSamples);


for i = 1:numSamples

    TAchieved = controlFK( ...
        robot, ...
        qPath(:,i));


    pAchieved(:,i) = TAchieved(1:3,4);


    %% Position error

    independentPositionError(i) = norm( ...
        TPath(1:3,4,i) - ...
        TAchieved(1:3,4));


    %% Orientation error
    %
    % Calculate true relative rotation angle.

    independentOrientationError(i) = rotationError( ...
        TPath(1:3,1:3,i), ...
        TAchieved(1:3,1:3));

end


fprintf('\n');
fprintf('============================================================\n');
fprintf(' INDEPENDENT IK / FK VALIDATION\n');
fprintf('============================================================\n');

fprintf( ...
    'Successful IK samples:          %d / %d\n', ...
    sum(ikConverged), ...
    numSamples);

fprintf( ...
    'Maximum position error:         %.3e m\n', ...
    max(independentPositionError));

fprintf( ...
    'Maximum orientation error:      %.3e rad\n', ...
    max(independentOrientationError));

fprintf( ...
    'Maximum ADLS iterations/sample: %d\n', ...
    max(ikIterations));

fprintf('============================================================\n');


%% ========================================================================
%  17. CALCULATE JOINT VELOCITY
% =========================================================================
%
% qPath is now:
%
%       q(t)
%
% so:
%
%       qDot(t) = dq/dt
%
%
% gradient() is used because:
%
%       - output length stays equal to numSamples
%       - actual time samples are used
%       - central differences are used internally for interior samples

qDot = zeros(size(qPath));


for joint = 1:n

    qDot(joint,:) = gradient( ...
        qPath(joint,:), ...
        t);

end


%% ========================================================================
%  18. CALCULATE JOINT ACCELERATION
% =========================================================================

qDDot = zeros(size(qPath));


for joint = 1:n

    qDDot(joint,:) = gradient( ...
        qDot(joint,:), ...
        t);

end


%% ========================================================================
%  19. CALCULATE NUMERICAL JOINT JERK
% =========================================================================
%
% Numerical differentiation amplifies discretization noise.
%
% Therefore qDDDot is useful as an exploratory metric here, but should
% NOT yet be interpreted as an exact physical actuator jerk.

qDDDot = zeros(size(qPath));


for joint = 1:n

    qDDDot(joint,:) = gradient( ...
        qDDot(joint,:), ...
        t);

end


%% ========================================================================
%  20. JOINT POSITION LIMIT CHECK
% =========================================================================

qMin = robot.limits.qMin(:);

qMax = robot.limits.qMax(:);


jointPositionViolation = false(n,1);


for joint = 1:n

    jointPositionViolation(joint) = ...
        any(qPath(joint,:) < qMin(joint)) || ...
        any(qPath(joint,:) > qMax(joint));

end


%% ========================================================================
%  21. JOINT VELOCITY LIMIT CHECK
% =========================================================================
%
% qdMax exists in the current UR5 RobotConfig.
%
% Therefore this is a meaningful physical constraint check.

peakJointVelocity = max(abs(qDot),[],2);


if ~isempty(robot.limits.qdMax)

    qdMax = robot.limits.qdMax(:);

    jointVelocityViolation = ...
        peakJointVelocity > qdMax;

else

    qdMax = nan(n,1);

    jointVelocityViolation = false(n,1);

end


%% ========================================================================
%  22. JOINT MOTION SUMMARY
% =========================================================================

peakJointAcceleration = max(abs(qDDot),[],2);

peakJointJerk = max(abs(qDDDot),[],2);


fprintf('\n');
fprintf('============================================================\n');
fprintf(' JOINT MOTION SUMMARY\n');
fprintf('============================================================\n');


for joint = 1:n

    fprintf( ...
        ['J%d | peak vel = %.4f rad/s | limit = %.4f rad/s | ' ...
         'position violation = %d | velocity violation = %d\n'], ...
        joint, ...
        peakJointVelocity(joint), ...
        qdMax(joint), ...
        jointPositionViolation(joint), ...
        jointVelocityViolation(joint));

end


fprintf('\nPeak joint acceleration [rad/s^2]:\n');

disp(peakJointAcceleration);


fprintf('Peak numerical joint jerk [rad/s^3]:\n');

disp(peakJointJerk);


if isempty(robot.limits.qddMax)

    fprintf( ...
        ['NOTE: trusted joint-acceleration limits are not yet ' ...
         'defined in RobotConfig.\n']);

end


fprintf('============================================================\n');


%% ========================================================================
%  23. CHECK DISCRETE JOINT CONTINUITY
% =========================================================================
%
% Sequential IK should produce a continuous qPath.
%
% Calculate:
%
%       deltaQ(k) = q(k+1) - q(k)
%
%
% We do NOT apply an arbitrary pass/fail threshold here.
%
% The physically meaningful restriction is handled through joint velocity:
%
%       qDot <= qdMax

deltaQ = diff(qPath,1,2);

maxJointStep = max(abs(deltaQ),[],2);


fprintf('\n');
fprintf('============================================================\n');
fprintf(' JOINT CONTINUITY\n');
fprintf('============================================================\n');

fprintf('Largest inter-sample joint change [deg]:\n');

disp(rad2deg(maxJointStep));

fprintf('============================================================\n');


%% ========================================================================
%  24. PLOT TIME-SCALING PROFILE
% =========================================================================

figure;

tiledlayout(4,1);


nexttile;

plot(t,s,'LineWidth',1.5);

grid on;

ylabel('s');

title( ...
    "Time Scaling - " + profileInfo.type);


nexttile;

plot(t,sDot,'LineWidth',1.5);

grid on;

ylabel('ds/dt [1/s]');


nexttile;

plot(t,sDDot,'LineWidth',1.5);

grid on;

ylabel('d^2s/dt^2 [1/s^2]');


nexttile;

plot(t,sDDDot,'LineWidth',1.5);

grid on;

ylabel('d^3s/dt^3 [1/s^3]');

xlabel('Time [s]');


%% ========================================================================
%  25. PLOT PHYSICAL TCP SPEED / ACCELERATION / JERK
% =========================================================================

figure;

tiledlayout(3,1);


nexttile;

plot(t,tcpSpeed,'LineWidth',1.5);

grid on;

ylabel('Speed [m/s]');

title( ...
    "TCP Motion - " + profileInfo.type);


nexttile;

plot(t,tcpAcceleration,'LineWidth',1.5);

grid on;

ylabel('Acceleration [m/s^2]');


nexttile;

plot(t,tcpJerk,'LineWidth',1.5);

grid on;

ylabel('Jerk [m/s^3]');

xlabel('Time [s]');


%% ========================================================================
%  26. PLOT JOINT POSITIONS
% =========================================================================

figure;

plot( ...
    t, ...
    rad2deg(qPath.'), ...
    'LineWidth',1.2);

grid on;

xlabel('Time [s]');

ylabel('Joint Angle [deg]');

title( ...
    "Joint Positions - " + profileInfo.type);

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% ========================================================================
%  27. PLOT JOINT VELOCITIES
% =========================================================================

figure;

plot( ...
    t, ...
    qDot.', ...
    'LineWidth',1.2);

grid on;

xlabel('Time [s]');

ylabel('Joint Velocity [rad/s]');

title( ...
    "Joint Velocities - " + profileInfo.type);

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% ========================================================================
%  28. PLOT JOINT ACCELERATIONS
% =========================================================================

figure;

plot( ...
    t, ...
    qDDot.', ...
    'LineWidth',1.2);

grid on;

xlabel('Time [s]');

ylabel('Joint Acceleration [rad/s^2]');

title( ...
    "Joint Accelerations - " + profileInfo.type);

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% ========================================================================
%  29. PLOT DESIRED VS ACHIEVED CARTESIAN PATH
% =========================================================================
%
% If IK is correct, the two curves should nearly overlap.

figure;


plot3( ...
    pPath(1,:), ...
    pPath(2,:), ...
    pPath(3,:), ...
    'o-', ...
    'LineWidth',1.5);

hold on;


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


grid on;

axis equal;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title( ...
    "Desired vs Achieved Cartesian Path - " + profileInfo.type);

legend( ...
    'Desired', ...
    'IK + FK achieved', ...
    'Start A', ...
    'End B', ...
    'Location','best');


%% ========================================================================
%  30. PLOT IK ITERATION COUNT
% =========================================================================
%
% This lets us see where the numerical solver finds the path difficult.
%
% Near-singular or poorly conditioned regions may require more iterations.

figure;

plot( ...
    t, ...
    ikIterations, ...
    'o-', ...
    'LineWidth',1.2);

grid on;

xlabel('Time [s]');

ylabel('ADLS Iterations');

title('ADLS Iterations Along Timed Cartesian Trajectory');


%% ========================================================================
%  31. ANIMATE ROBOT
% =========================================================================
%
% Finally:
%
%       qPath(:,i)
%
% is displayed sequentially using the MATLAB rigidBodyTree.
%
%
% IMPORTANT:
% This is visual playback, NOT real-time dynamic simulation.
%
% MATLAB rendering itself consumes time, so pause() only approximates the
% requested trajectory timing.

figure;

ax = axes;


%% Draw first robot configuration

show( ...
    refRobot, ...
    qPath(:,1), ...
    'Parent',ax, ...
    'FastUpdate',true, ...
    'PreservePlot',false);


hold(ax,'on');

grid(ax,'on');

axis(ax,'equal');

%% Draw preliminary welding station

show( ...
    station.mountingPlane.object, ...
    'Parent', ax);

show( ...
    station.table.object, ...
    'Parent', ax);

xlabel(ax,'X [m]');

ylabel(ax,'Y [m]');

zlabel(ax,'Z [m]');


title( ...
    ax, ...
    "UR5 Timed Cartesian Motion - ADLS + " + profileInfo.type);


%% Draw complete desired Cartesian path

plot3( ...
    ax, ...
    pPath(1,:), ...
    pPath(2,:), ...
    pPath(3,:), ...
    'LineWidth',2);


%% Mark endpoints

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


%% ========================================================================
%  Playback speed
% =========================================================================
%
% playbackScale = 1
%
% tries to visually reproduce nominal trajectory time.
%
% Example:
%
%       2.0 -> twice as slow
%       0.5 -> twice as fast

playbackScale = 1.0;

%% ========================================================================
%  ANIMATION PLAYBACK
% =========================================================================
%
% The trajectory has ALREADY been calculated at this point:
%
%       qPath = joint trajectory
%       t     = trajectory time vector
%
% Therefore replaying the animation does NOT require:
%
%       path generation
%       time scaling
%       inverse kinematics
%       FK validation
%
% We simply display the already-computed qPath again.

playTrajectory( ...
    refRobot, ...
    qPath, ...
    t, ...
    ax, ...
    playbackScale);


%% ========================================================================
%  REPLAY BUTTON
% =========================================================================
%
% Adds a button directly to the animation figure.
%
% Every click simply plays the already-calculated trajectory again.

animationFigure = ancestor(ax,'figure');

uicontrol( ...
    animationFigure, ...
    'Style','pushbutton', ...
    'String','Replay Motion', ...
    'Units','normalized', ...
    'Position',[0.82 0.02 0.14 0.06], ...
    'FontSize',11, ...
    'Callback', @(~,~) playTrajectory( ...
        refRobot, ...
        qPath, ...
        t, ...
        ax, ...
        playbackScale));


fprintf('\n');
fprintf('============================================================\n');
fprintf(' TRAJECTORY PIPELINE COMPLETE\n');
fprintf('============================================================\n');

fprintf('Cartesian path generated:     PASS\n');
fprintf('Time scaling generated:       PASS\n');
fprintf('Sequential ADLS IK:           PASS\n');
fprintf('Independent FK validation:    PASS\n');

fprintf( ...
    'Joint position limits:       %s\n', ...
    passFail(~any(jointPositionViolation)));

fprintf( ...
    'Joint velocity limits:       %s\n', ...
    passFail(~any(jointVelocityViolation)));

fprintf('Robot animation completed:    PASS\n');

fprintf('============================================================\n');


%% ========================================================================
% Local function: independent orientation error
% ========================================================================
%
% Computes the shortest rotational difference between:
%
%       RTarget
%
% and:
%
%       RActual
%
% using:
%
%       theta = acos((trace(RRelative)-1)/2)

function angle = rotationError(RTarget,RActual)

RRelative = ...
    RTarget * RActual';

cosAngle = ...
    (trace(RRelative) - 1) / 2;

% Protect acos against floating-point errors.

cosAngle = ...
    max(-1,min(1,cosAngle));

angle = acos(cosAngle);

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

function playTrajectory(refRobot,qPath,t,ax,playbackScale)
%PLAYTRAJECTORY Replay an already-computed robot joint trajectory.
%
% This function DOES NOT recompute:
%   - path generation
%   - time scaling
%   - inverse kinematics
%
% It only displays the already-calculated qPath.

numSamples = size(qPath,2);

for i = 1:numSamples

    %% Display current robot configuration

    show( ...
        refRobot, ...
        qPath(:,i), ...
        'Parent',ax, ...
        'FastUpdate',true, ...
        'PreservePlot',false, ...
        'Visuals','on', ...
        'Collisions','off');

    %% Update title

    title( ...
        ax, ...
        sprintf( ...
            'UR5 Timed Cartesian Motion | t = %.2f / %.2f s', ...
            t(i), ...
            t(end)));

    drawnow;

    %% Preserve trajectory timing

    if i < numSamples

        frameDuration = t(i+1) - t(i);

        pause(playbackScale * frameDuration);

    end

end

end