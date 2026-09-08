%% testMultiSegmentCartesianADLS
%
% MULTI-SEGMENT CARTESIAN TRAJECTORY STRESS TEST
%
% PURPOSE
% -------
% Stress-test the current Cartesian trajectory pipeline using several
% connected straight-line segments:
%
%       A -> B -> C -> D -> E -> F
%
% The test intentionally contains:
%
%       - Different segment lengths
%       - Motion in X
%       - Motion in Y
%       - Motion in Z
%       - Sharp Cartesian direction changes
%       - Sequential ADLS IK
%       - Constant TCP orientation
%
%
% IMPORTANT
% ---------
% Each segment uses its own trapezoidal/triangular time scaling.
%
% Therefore:
%
%       velocity = 0
%
% at every corner.
%
% This is intentional.
%
% A sharp corner cannot physically be crossed with nonzero Cartesian
% velocity without:
%
%       - infinite acceleration, or
%       - modifying/blending the geometric path.
%
% Corner blending will be tested later.
%
%
% CURRENT PIPELINE
%
%       Cartesian waypoints
%              ↓
%       straight segments
%              ↓
%       quintic s(t) for each segment
%              ↓
%       timed Cartesian poses
%              ↓
%       sequential ADLS IK
%              ↓
%             q(t)
%              ↓
%       independent FK check
%              ↓
%       joint velocity / limits
%              ↓
%           animation

clear;
clc;
close all;


%% ========================================================================
% 1. ROBOT
% =========================================================================

robot = config.UR5();

n = robot.dof;


%% ========================================================================
% 2. VISUALIZATION MODEL
% =========================================================================

visualRobot = robotmodel.buildUR5geometry(robot);

station = environment.buildPreliminaryWeldingStation();


%% ========================================================================
% 3. STARTING CONFIGURATION
% =========================================================================
%
% Same well-behaved configuration used in our previous timed test.

qStart = deg2rad([
     30
    -45
     60
     20
    -30
     45
]);


%% ========================================================================
% 4. GET INITIAL TCP POSE
% =========================================================================

TStart = controlFK(robot,qStart);

pA = TStart(1:3,4);

RConstant = TStart(1:3,1:3);


%% ========================================================================
% 5. DEFINE HARSH MULTI-SEGMENT CARTESIAN PATH
% =========================================================================
%
% Different lengths and directions are deliberately used.
%
% Approximate segment lengths:
%
%   A -> B : 0.50 m
%   B -> C : 0.22 m
%   C -> D : 0.39 m
%   D -> E : 0.39 m
%   E -> F : 0.30 m
%
% The path contains:
%
%   - Long 50 cm move
%   - X motion
%   - Y motion
%   - Z motion
%   - Diagonal 3-D motion
%   - Direction reversals
%
% This is intentionally much harsher than the previous test.

pB = pA + [
     0.50
     0
     0
];

pC = pB + [
     0
     0.20
     0.10
];

pD = pC + [
    -0.35
     0.10
     0.15
];

pE = pD + [
     0.15
    -0.35
    -0.10
];

pF = pE + [
    -0.20
     0.10
    -0.20
];

%% Store all Cartesian waypoints

waypoints = [
    pA ...
    pB ...
    pC ...
    pD ...
    pE ...
    pF
];

numWaypoints = size(waypoints,2);

numSegments = numWaypoints - 1;


%% Names used in console output

waypointNames = [
    "A"
    "B"
    "C"
    "D"
    "E"
    "F"
];

%% ========================================================================
% 6. TRAJECTORY SETTINGS
% =========================================================================
%
% We now specify PHYSICAL Cartesian motion requirements rather than giving
% every segment an arbitrary duration.
%
% desiredTCPSpeed:
%
%       desired constant TCP cruise speed along each straight segment
%
% desiredTCPAccel:
%
%       desired TCP acceleration magnitude during acceleration/deceleration
%
%
% The trapezoidal time-scaling function works with the NORMALIZED path
% parameter s, not directly with meters.
%
% For a straight Cartesian path of length L:
%
%       TCP speed        = L * sDot
%       TCP acceleration = L * sDDot
%
% Therefore:
%
%       sDotMax  = desiredTCPSpeed / L
%       sDDotMax = desiredTCPAccel / L
%
%
% The duration of each segment is NOT chosen manually anymore.
% trapezoidalTimeScaling() calculates it automatically.

desiredTCPSpeed = 0.10;      % [m/s]   = 10 cm/s
desiredTCPAccel = 0.25;      % [m/s^2]

dt = 0.05;                   % [s]


%% ========================================================================
% 7. GLOBAL TRAJECTORY STORAGE
% =========================================================================
%
% All separately-generated Cartesian segments will be joined into one
% complete trajectory:
%
%       A -> B -> C -> D -> E -> F
%
% Endpoint duplication is removed.
%
% Example:
%
%       Segment AB ends at B
%       Segment BC begins at B
%
% B should appear only once in the final trajectory.

TPath = zeros(4,4,0);

pPath = zeros(3,0);

tGlobal = [];

segmentIndexPath = [];


% Store physical TCP speed so we can later verify that the cruise region
% really reaches the requested constant welding/travel speed.

tcpSpeedPath = [];


%% ========================================================================
% 8. GENERATE EVERY CARTESIAN SEGMENT
% =========================================================================

currentGlobalTime = 0;


fprintf('\n');
fprintf('============================================================\n');
fprintf(' MULTI-SEGMENT CARTESIAN PATH\n');
fprintf('============================================================\n');


for segment = 1:numSegments

    %% --------------------------------------------------------------------
    % Start and end positions of this Cartesian line
    % ---------------------------------------------------------------------

    pStartSegment = waypoints(:,segment);

    pEndSegment = waypoints(:,segment+1);


    %% --------------------------------------------------------------------
    % Segment geometry
    % ---------------------------------------------------------------------

    segmentVector = ...
        pEndSegment - pStartSegment;

    segmentLength = ...
        norm(segmentVector);


    fprintf( ...
        'Segment %s -> %s | Length = %.4f m\n', ...
        waypointNames(segment), ...
        waypointNames(segment+1), ...
        segmentLength);


    %% --------------------------------------------------------------------
    % Convert physical TCP requirements into normalized path limits
    % ---------------------------------------------------------------------
    %
    % Straight Cartesian interpolation:
    %
    %       p(s) = pStart + s(pEnd-pStart)
    %
    % where:
    %
    %       0 <= s <= 1
    %
    %
    % Differentiating:
    %
    %       |pDot| = segmentLength * sDot
    %
    % Therefore:
    %
    %       sDotMax = desiredTCPSpeed / segmentLength
    %
    %
    % Similarly:
    %
    %       |pDDot| = segmentLength * sDDot
    %
    % therefore:
    %
    %       sDDotMax = desiredTCPAccel / segmentLength

    sDotMax = ...
        desiredTCPSpeed / segmentLength;

    sDDotMax = ...
        desiredTCPAccel / segmentLength;


    %% --------------------------------------------------------------------
    % Generate trapezoidal / triangular time scaling
    % ---------------------------------------------------------------------
    %
    % If the segment is long enough:
    %
    %       accelerate
    %           ↓
    %       constant velocity
    %           ↓
    %       decelerate
    %
    %
    % If the segment is too short to reach desiredTCPSpeed:
    %
    %       accelerate
    %           ↓
    %       immediately decelerate
    %
    % and trapezoidalTimeScaling() automatically returns a TRIANGULAR
    % profile instead.
    %
    %
    % Important:
    %
    % The function calculates the required segment duration automatically.

    [ ...
        tSegment, ...
        sSegment, ...
        sDotSegment, ...
        sDDotSegment, ...
        ~, ...
        profileInfo ...
    ] = trapezoidalTimeScaling( ...
            sDotMax, ...
            sDDotMax, ...
            dt);


    % Force consistent row-vector format.

    tSegment = tSegment(:).';

    sSegment = sSegment(:).';

    sDotSegment = sDotSegment(:).';

    sDDotSegment = sDDotSegment(:).';


    %% --------------------------------------------------------------------
    % Physical TCP speed for this segment
    % ---------------------------------------------------------------------
    %
    % This should show:
    %
    %       acceleration
    %       flat constant-speed region
    %       deceleration
    %
    % for a true trapezoidal profile.

    tcpSpeedSegment = ...
        segmentLength * sDotSegment;


    fprintf( ...
        ['                 Profile = %-11s | Duration = %.3f s | ' ...
         'Cruise/Peak TCP speed = %.4f m/s\n'], ...
        profileInfo.type, ...
        profileInfo.T, ...
        max(tcpSpeedSegment));


    %% --------------------------------------------------------------------
    % Generate straight Cartesian position path
    % ---------------------------------------------------------------------

    pSegment = linearInterpolation( ...
        pStartSegment, ...
        pEndSegment, ...
        sSegment);


    %% --------------------------------------------------------------------
    % Build complete Cartesian poses
    % ---------------------------------------------------------------------
    %
    % Position changes along the straight line.
    %
    % Orientation remains constant for this stress test.

    numberOfSegmentSamples = ...
        numel(tSegment);

    TSegment = ...
        zeros(4,4,numberOfSegmentSamples);


    for k = 1:numberOfSegmentSamples

        T = eye(4);

        % Constant TCP orientation.
        T(1:3,1:3) = RConstant;

        % Current Cartesian position.
        T(1:3,4) = pSegment(:,k);

        TSegment(:,:,k) = T;

    end


    %% --------------------------------------------------------------------
    % Remove duplicated waypoint sample
    % ---------------------------------------------------------------------
    %
    % Segment AB already contains B.
    %
    % Therefore Segment BC's first sample, which is also B, is removed.
    %
    % This avoids duplicate times and duplicate poses in the final path.

    if segment > 1

        tSegment = ...
            tSegment(2:end);

        pSegment = ...
            pSegment(:,2:end);

        TSegment = ...
            TSegment(:,:,2:end);

        tcpSpeedSegment = ...
            tcpSpeedSegment(2:end);

    end


    %% --------------------------------------------------------------------
    % Convert local segment time into global trajectory time
    % ---------------------------------------------------------------------
    %
    % Example:
    %
    % Segment AB:
    %
    %       local  0 -> 5 s
    %       global 0 -> 5 s
    %
    % Segment BC:
    %
    %       local  0 -> 3 s
    %       global 5 -> 8 s

    tSegmentGlobal = ...
        currentGlobalTime + tSegment;


    %% --------------------------------------------------------------------
    % Append segment to complete Cartesian trajectory
    % ---------------------------------------------------------------------

    TPath = cat( ...
        3, ...
        TPath, ...
        TSegment);


    pPath = [
        pPath ...
        pSegment
    ];


    tGlobal = [
        tGlobal ...
        tSegmentGlobal
    ];


    tcpSpeedPath = [
        tcpSpeedPath ...
        tcpSpeedSegment
    ];


    segmentIndexPath = [
        segmentIndexPath ...
        segment * ones(1,numel(tSegment))
    ];


    %% --------------------------------------------------------------------
    % Advance global time
    % ---------------------------------------------------------------------
    %
    % IMPORTANT:
    %
    % Each segment can now have a DIFFERENT duration.
    %
    % Therefore we use the duration calculated by
    % trapezoidalTimeScaling(), rather than:
    %
    %       segment * segmentDuration

    currentGlobalTime = ...
        currentGlobalTime + profileInfo.T;

end


%% ========================================================================
% COMPLETE PATH INFORMATION
% =========================================================================

numSamples = ...
    numel(tGlobal);


fprintf('\n');

fprintf('Segments:       %d\n', ...
    numSegments);

fprintf('Total samples:  %d\n', ...
    numSamples);

fprintf('Total duration: %.3f s\n', ...
    tGlobal(end));

fprintf('Requested TCP cruise speed: %.4f m/s\n', ...
    desiredTCPSpeed);

fprintf('============================================================\n');


%% ========================================================================
% PLOT TCP SPEED
% =========================================================================
%
% This is the important new plot.
%
% For every trapezoidal segment we expect:
%
%
% TCP
% speed
%
%       ____________
%      /            \
% ____/              \____
%
%
% The flat region should be approximately:
%
%       desiredTCPSpeed
%
% Each waypoint still has zero velocity because we have NOT implemented
% corner blending yet.

figure;

plot( ...
    tGlobal, ...
    tcpSpeedPath, ...
    'LineWidth',1.5);

hold on;


yline( ...
    desiredTCPSpeed, ...
    '--', ...
    'Desired TCP cruise speed');


grid on;

xlabel('Time [s]');

ylabel('TCP Speed [m/s]');

title('TCP Speed Through Multi-Segment Trapezoidal Path');


%% ========================================================================
% 9. SEQUENTIAL ADLS IK
% =========================================================================
%
% This is intentionally ONE continuous sequence.
%
% We do NOT reset the seed at each Cartesian segment.
%
% Therefore:
%
%       previous q
%
% always seeds:
%
%       next Cartesian pose
%
% even when crossing B, C, D, etc.

qPath = zeros(n,numSamples);

ikConverged = false(1,numSamples);

ikIterations = zeros(1,numSamples);


qSeed = qStart;


fprintf('\n');
fprintf('============================================================\n');
fprintf(' SEQUENTIAL ADLS IK\n');
fprintf('============================================================\n');


for k = 1:numSamples

    [qSolution,info] = ADLS_IK( ...
        robot, ...
        TPath(:,:,k), ...
        qSeed);


    ikConverged(k) = info.converged;

    ikIterations(k) = info.iterations;


    if ~info.converged

        fprintf('\n');
        fprintf('IK FAILURE\n');
        fprintf('----------\n');

        fprintf('Global sample: %d / %d\n',k,numSamples);

        fprintf( ...
            'Segment: %s -> %s\n', ...
            waypointNames(segmentIndexPath(k)), ...
            waypointNames(segmentIndexPath(k)+1));

        fprintf( ...
            'Position error: %.3e m\n', ...
            info.positionError);

        fprintf( ...
            'Orientation error: %.3e rad\n', ...
            info.orientationError);

        error( ...
            'Multi-segment trajectory failed during sequential ADLS IK.');

    end


    qPath(:,k) = qSolution;

    qSeed = qSolution;

end


fprintf( ...
    'Successful IK samples: %d / %d\n', ...
    sum(ikConverged), ...
    numSamples);

fprintf('============================================================\n');


%% ========================================================================
% 10. INDEPENDENT FK VALIDATION
% =========================================================================

positionError = zeros(1,numSamples);

orientationError = zeros(1,numSamples);

pAchieved = zeros(3,numSamples);


for k = 1:numSamples

    TAchieved = controlFK( ...
        robot, ...
        qPath(:,k));


    pAchieved(:,k) = ...
        TAchieved(1:3,4);


    positionError(k) = norm( ...
        TPath(1:3,4,k) - ...
        TAchieved(1:3,4));


    orientationError(k) = rotationError( ...
        TPath(1:3,1:3,k), ...
        TAchieved(1:3,1:3));

end


fprintf('\n');
fprintf('============================================================\n');
fprintf(' INDEPENDENT FK VALIDATION\n');
fprintf('============================================================\n');

fprintf( ...
    'Maximum position error:    %.3e m\n', ...
    max(positionError));

fprintf( ...
    'Maximum orientation error: %.3e rad\n', ...
    max(orientationError));

fprintf( ...
    'Maximum IK iterations:     %d\n', ...
    max(ikIterations));

fprintf('============================================================\n');


%% ========================================================================
% 11. JOINT VELOCITY
% =========================================================================

qDot = zeros(size(qPath));


for joint = 1:n

    qDot(joint,:) = gradient( ...
        qPath(joint,:), ...
        tGlobal);

end


%% ========================================================================
% 12. JOINT ACCELERATION
% =========================================================================

qDDot = zeros(size(qPath));


for joint = 1:n

    qDDot(joint,:) = gradient( ...
        qDot(joint,:), ...
        tGlobal);

end


%% ========================================================================
% 13. JOINT LIMIT CHECKS
% =========================================================================

qMin = robot.limits.qMin(:);

qMax = robot.limits.qMax(:);


positionViolation = false(n,1);


for joint = 1:n

    positionViolation(joint) = ...
        any(qPath(joint,:) < qMin(joint)) || ...
        any(qPath(joint,:) > qMax(joint));

end


peakJointVelocity = ...
    max(abs(qDot),[],2);


if ~isempty(robot.limits.qdMax)

    qdMax = robot.limits.qdMax(:);

    velocityViolation = ...
        peakJointVelocity > qdMax;

else

    qdMax = nan(n,1);

    velocityViolation = false(n,1);

end

%% ========================================================================
% EXACT JOINT VELOCITY VIOLATION DIAGNOSTICS
% =========================================================================
%
% A Cartesian path can be:
%
%       reachable by IK
%
% but still NOT physically executable at the requested timing.
%
% Here we identify exactly where:
%
%       |qDot| > qdMax
%
% occurs.
%
% For every violation we report:
%
%       joint
%       sample
%       time
%       Cartesian segment
%       actual joint velocity
%       allowed joint velocity
%       percentage over limit

fprintf('\n');
fprintf('============================================================\n');
fprintf(' JOINT VELOCITY VIOLATION DIAGNOSTICS\n');
fprintf('============================================================\n');


anyVelocityViolation = false;


for joint = 1:n

    %% Samples where this joint exceeds its allowed velocity

    violatingSamples = find( ...
        abs(qDot(joint,:)) > qdMax(joint));


    if ~isempty(violatingSamples)

        anyVelocityViolation = true;


        fprintf('\n');
        fprintf('JOINT %d VIOLATION\n',joint);
        fprintf('-----------------\n');


        for idx = 1:numel(violatingSamples)

            sample = violatingSamples(idx);

            segment = segmentIndexPath(sample);


            actualVelocity = ...
                abs(qDot(joint,sample));

            velocityLimit = ...
                qdMax(joint);


            percentOver = ...
                100 * ...
                (actualVelocity / velocityLimit - 1);


            fprintf( ...
                ['Sample %3d | t = %.3f s | Segment %s -> %s | ' ...
                 '|qDot| = %.4f rad/s | limit = %.4f rad/s | ' ...
                 '%.2f %% over limit\n'], ...
                sample, ...
                tGlobal(sample), ...
                waypointNames(segment), ...
                waypointNames(segment+1), ...
                actualVelocity, ...
                velocityLimit, ...
                percentOver);

        end

    end

end


if ~anyVelocityViolation

    fprintf('No joint velocity violations detected.\n');

end


fprintf('============================================================\n');
%% ========================================================================
% 14. JOINT CONTINUITY CHECK
% =========================================================================
%
% Large jumps indicate:
%
%       branch switching
%
% or:
%
%       unstable sequential IK.

deltaQ = diff(qPath,1,2);

maxJointStep = ...
    max(abs(deltaQ),[],2);


fprintf('\n');
fprintf('============================================================\n');
fprintf(' JOINT TRAJECTORY SUMMARY\n');
fprintf('============================================================\n');


for joint = 1:n

    fprintf( ...
        ['J%d | peak vel %.4f rad/s | ' ...
         'largest step %.4f deg | ' ...
         'position violation %d | velocity violation %d\n'], ...
        joint, ...
        peakJointVelocity(joint), ...
        rad2deg(maxJointStep(joint)), ...
        positionViolation(joint), ...
        velocityViolation(joint));

end


fprintf('============================================================\n');


%% ========================================================================
% 15. PLOT CARTESIAN POLYLINE
% =========================================================================

figure;

plot3( ...
    pPath(1,:), ...
    pPath(2,:), ...
    pPath(3,:), ...
    'LineWidth',1.5);

hold on;


plot3( ...
    waypoints(1,:), ...
    waypoints(2,:), ...
    waypoints(3,:), ...
    'o', ...
    'MarkerSize',8, ...
    'LineWidth',1.5);


for i = 1:numWaypoints

    text( ...
        waypoints(1,i), ...
        waypoints(2,i), ...
        waypoints(3,i), ...
        "  " + waypointNames(i));

end


grid on;

axis equal;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title('Multi-Segment Cartesian Stress-Test Path');


%% ========================================================================
% 16. DESIRED VS ACHIEVED PATH
% =========================================================================

figure;

plot3( ...
    pPath(1,:), ...
    pPath(2,:), ...
    pPath(3,:), ...
    'LineWidth',1.5);

hold on;

plot3( ...
    pAchieved(1,:), ...
    pAchieved(2,:), ...
    pAchieved(3,:), ...
    '--', ...
    'LineWidth',1.5);

grid on;

axis equal;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

legend( ...
    'Desired', ...
    'IK + FK achieved');

title('Desired vs Achieved Multi-Segment Path');


%% ========================================================================
% 17. JOINT POSITION PLOT
% =========================================================================

figure;

plot( ...
    tGlobal, ...
    rad2deg(qPath.'), ...
    'LineWidth',1.2);

grid on;

xlabel('Time [s]');
ylabel('Joint angle [deg]');

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');

title('Joint Motion Through Multi-Segment Cartesian Path');


%% ========================================================================
% 18. JOINT VELOCITY PLOT
% =========================================================================

figure;

plot( ...
    tGlobal, ...
    qDot.', ...
    'LineWidth',1.2);

grid on;

xlabel('Time [s]');
ylabel('Joint velocity [rad/s]');

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');

title('Joint Velocity Through Multi-Segment Path');


%% ========================================================================
% 19. IK ITERATION PLOT
% =========================================================================

figure;

plot( ...
    tGlobal, ...
    ikIterations, ...
    'LineWidth',1.2);

grid on;

xlabel('Time [s]');
ylabel('ADLS iterations');

title('ADLS Difficulty Through Multi-Segment Path');


%% ========================================================================
% 20. ANIMATION
% =========================================================================

figure;

ax = axes;


%% Draw robot

show( ...
    visualRobot, ...
    qPath(:,1), ...
    'Parent',ax, ...
    'FastUpdate',true, ...
    'PreservePlot',false, ...
    'Visuals','on', ...
    'Collisions','off');


hold(ax,'on');

grid(ax,'on');

axis(ax,'equal');


%% Draw environment

show( ...
    station.mountingPlane.object, ...
    'Parent',ax);

show( ...
    station.table.object, ...
    'Parent',ax);


%% Draw complete trajectory

plot3( ...
    ax, ...
    pPath(1,:), ...
    pPath(2,:), ...
    pPath(3,:), ...
    'LineWidth',2);


%% Draw waypoints

plot3( ...
    ax, ...
    waypoints(1,:), ...
    waypoints(2,:), ...
    waypoints(3,:), ...
    'o', ...
    'MarkerSize',8);


for i = 1:numWaypoints

    text( ...
        ax, ...
        waypoints(1,i), ...
        waypoints(2,i), ...
        waypoints(3,i), ...
        "  " + waypointNames(i));

end


xlabel(ax,'X [m]');
ylabel(ax,'Y [m]');
zlabel(ax,'Z [m]');


playbackScale = 1;


%% Play trajectory

playTrajectory( ...
    visualRobot, ...
    qPath, ...
    tGlobal, ...
    ax, ...
    playbackScale);


%% Replay button

animationFigure = ancestor(ax,'figure');


uicontrol( ...
    animationFigure, ...
    'Style','pushbutton', ...
    'String','Replay Motion', ...
    'Units','normalized', ...
    'Position',[0.82 0.02 0.14 0.06], ...
    'FontSize',11, ...
    'Callback', @(~,~) playTrajectory( ...
        visualRobot, ...
        qPath, ...
        tGlobal, ...
        ax, ...
        playbackScale));


%% ========================================================================
% 21. FINAL RESULT
% =========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' MULTI-SEGMENT TRAJECTORY STRESS TEST\n');
fprintf('============================================================\n');

fprintf( ...
    'Cartesian segments:      %d\n', ...
    numSegments);

fprintf( ...
    'IK samples passed:       %d / %d\n', ...
    sum(ikConverged), ...
    numSamples);

fprintf( ...
    'Max position error:      %.3e m\n', ...
    max(positionError));

fprintf( ...
    'Max orientation error:   %.3e rad\n', ...
    max(orientationError));

fprintf( ...
    'Joint position limits:   %s\n', ...
    passFail(~any(positionViolation)));

fprintf( ...
    'Joint velocity limits:   %s\n', ...
    passFail(~any(velocityViolation)));

fprintf('============================================================\n');


%% ========================================================================
% LOCAL FUNCTION — ORIENTATION ERROR
% =========================================================================

function angle = rotationError(RTarget,RActual)

RRelative = ...
    RTarget * RActual';

c = ...
    (trace(RRelative)-1)/2;

c = max(-1,min(1,c));

angle = acos(c);

end


%% ========================================================================
% LOCAL FUNCTION — PASS / FAIL
% =========================================================================

function txt = passFail(tf)

if tf

    txt = 'PASS';

else

    txt = 'FAIL';

end

end


%% ========================================================================
% LOCAL FUNCTION — ANIMATION PLAYBACK
% =========================================================================

function playTrajectory(robotModel,qPath,t,ax,playbackScale)

numSamples = size(qPath,2);


for i = 1:numSamples

    show( ...
        robotModel, ...
        qPath(:,i), ...
        'Parent',ax, ...
        'FastUpdate',true, ...
        'PreservePlot',false, ...
        'Visuals','on', ...
        'Collisions','off');


    title( ...
        ax, ...
        sprintf( ...
            'UR5 Multi-Segment Motion | t = %.2f / %.2f s', ...
            t(i), ...
            t(end)));


    drawnow;


    if i < numSamples

        pause( ...
            playbackScale * ...
            (t(i+1)-t(i)));

    end

end

end