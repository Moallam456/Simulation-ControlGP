%% trajectoryPipelineArcLength
%
% MULTI-SEGMENT CARTESIAN TRAJECTORY PIPELINE USING ARC-LENGTH PARAMETERIZATION
%
% PURPOSE
% -------
% This script keeps the established multi-segment trajectory pipeline:
%
%   Cartesian waypoints
%          ↓
%   straight geometric segments
%          ↓
%   segment-wise time scaling
%          ↓
%   timed Cartesian poses
%          ↓
%   sequential ADLS IK
%          ↓
%        q(t)
%          ↓
%   independent FK validation
%          ↓
%   joint velocity / limits
%          ↓
%      animation
%
% but inserts an explicit ARC-LENGTH REPRESENTATION between geometry and
% time scaling:
%
%   raw geometric line samples
%          ↓
%   arcLengthParameterize()
%          ↓
%        p(l)
%          ↓
%   normalized time scaling s(t), 0 <= s <= 1
%          ↓
%        l(t) = L*s(t)
%          ↓
%        p(l(t))
%
% IMPORTANT
% ---------
% Each sharp Cartesian segment is still time-scaled SEPARATELY.
% Therefore TCP velocity is zero at B, C, D, E, etc.
%
% This is intentional. A sharp corner cannot be crossed with nonzero
% Cartesian velocity without blending/modifying the geometric path.
%
% For straight lines this arc-length stage is mathematically redundant,
% because normalized linear interpolation is already proportional to arc
% length. This script exists to establish the architecture needed later for
% splines, recorded paths, and arbitrary curves.

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
% 5. DEFINE MULTI-SEGMENT CARTESIAN PATH
% =========================================================================

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

waypoints = [
    pA ...
    pB ...
    pC ...
    pD ...
    pE ...
    pF
];

waypointNames = [
    "A"
    "B"
    "C"
    "D"
    "E"
    "F"
];

numWaypoints = size(waypoints,2);
numSegments = numWaypoints - 1;


%% ========================================================================
% 6. PATH + TRAJECTORY SETTINGS
% =========================================================================

% Dense geometric samples are generated first.
numGeometryPointsPerSegment = 100;

% arcLengthParameterize() then resamples the geometric path approximately
% every 5 mm.
arcLengthSpacing = 0.005;      % [m]

% Physical Cartesian motion requirements.
desiredTCPSpeed = 0.10;        % [m/s]
desiredTCPAccel = 0.25;        % [m/s^2]

% Time sampling.
dt = 0.05;                     % [s]


%% ========================================================================
% 7. GLOBAL STORAGE
% =========================================================================

TPath = zeros(4,4,0);
pPath = zeros(3,0);

tGlobal = [];
tcpSpeedPath = [];
arcPositionPath = [];
segmentIndexPath = [];

% Geometry-only storage for visualization.
rawGeometryPath = zeros(3,0);
arcGeometryPath = zeros(3,0);

currentGlobalTime = 0;
currentGlobalArcLength = 0;


%% ========================================================================
% 8. GENERATE + ARC-LENGTH PARAMETERIZE + TIME-SCALE EACH SEGMENT
% =========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' ARC-LENGTH CARTESIAN TRAJECTORY PIPELINE\n');
fprintf('============================================================\n');

for segment = 1:numSegments

    %% --------------------------------------------------------------------
    % Segment endpoints
    % ---------------------------------------------------------------------

    pStartSegment = waypoints(:,segment);
    pEndSegment = waypoints(:,segment+1);


    %% --------------------------------------------------------------------
    % A. Generate raw geometric line
    % ---------------------------------------------------------------------

    rawSegment = generateLineWaypoints( ...
        pStartSegment, ...
        pEndSegment, ...
        numGeometryPointsPerSegment);


    %% --------------------------------------------------------------------
    % B. Arc-length parameterize geometric line
    % ---------------------------------------------------------------------
    %
    % arcSegment     -> Cartesian path samples approximately equally spaced
    %                   in physical distance
    %
    % lOriginal      -> cumulative arc length of rawSegment
    %
    % lArc           -> physical arc-length locations of arcSegment samples

    [arcSegment, lOriginal, lArc] = ...
        arcLengthParameterize(rawSegment,arcLengthSpacing);

    segmentLength = lOriginal(end);

    if segmentLength <= eps
        error('Segment %s -> %s has zero length.', ...
            waypointNames(segment), waypointNames(segment+1));
    end


    %% --------------------------------------------------------------------
    % C. Convert physical TCP limits to normalized path limits
    % ---------------------------------------------------------------------
    %
    % Time scaling still uses normalized:
    %
    %       0 <= s <= 1
    %
    % Because:
    %
    %       l = L*s
    %
    % then:
    %
    %       lDot  = L*sDot
    %       lDDot = L*sDDot
    %
    % For arc-length parameterized Cartesian motion:
    %
    %       TCP speed = lDot
    %
    % therefore:
    %
    %       sDotMax  = desiredTCPSpeed / L
    %       sDDotMax = desiredTCPAccel / L

    sDotMax = desiredTCPSpeed / segmentLength;
    sDDotMax = desiredTCPAccel / segmentLength;


    %% --------------------------------------------------------------------
    % D. Generate normalized time scaling s(t)
    % ---------------------------------------------------------------------

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

    tSegment = tSegment(:).';
    sSegment = sSegment(:).';
    sDotSegment = sDotSegment(:).';
    sDDotSegment = sDDotSegment(:).';


    %% --------------------------------------------------------------------
    % E. Convert normalized progress s(t) to physical arc length l(t)
    % ---------------------------------------------------------------------

    lTimed = segmentLength * sSegment;

    % Physical TCP speed along an arc-length-parameterized path.
    tcpSpeedSegment = segmentLength * sDotSegment;


    %% --------------------------------------------------------------------
    % F. Evaluate Cartesian path p(l(t))
    % ---------------------------------------------------------------------
    %
    % arcSegment is known at physical arc-length locations lArc.
    % We now ask: where on that geometric path should the TCP be at every
    % time-scaled physical distance lTimed?

    pSegment = zeros(3,numel(lTimed));

    for axisIndex = 1:3

        pSegment(axisIndex,:) = interp1( ...
            lArc, ...
            arcSegment(axisIndex,:), ...
            lTimed, ...
            'linear');

    end


    %% --------------------------------------------------------------------
    % G. Build Cartesian pose trajectory
    % ---------------------------------------------------------------------

    numberOfSegmentSamples = numel(tSegment);

    TSegment = zeros(4,4,numberOfSegmentSamples);

    for k = 1:numberOfSegmentSamples

        T = eye(4);

        T(1:3,1:3) = RConstant;
        T(1:3,4) = pSegment(:,k);

        TSegment(:,:,k) = T;

    end


    %% --------------------------------------------------------------------
    % H. Convert local arc length to cumulative/global arc length
    % ---------------------------------------------------------------------

    lGlobalSegment = currentGlobalArcLength + lTimed;


    %% --------------------------------------------------------------------
    % I. Remove duplicate connection sample
    % ---------------------------------------------------------------------

    if segment > 1

        tSegment = tSegment(2:end);
        pSegment = pSegment(:,2:end);
        TSegment = TSegment(:,:,2:end);

        tcpSpeedSegment = tcpSpeedSegment(2:end);
        lGlobalSegment = lGlobalSegment(2:end);

    end


    %% --------------------------------------------------------------------
    % J. Convert local time to global time
    % ---------------------------------------------------------------------

    tSegmentGlobal = currentGlobalTime + tSegment;


    %% --------------------------------------------------------------------
    % K. Append timed trajectory
    % ---------------------------------------------------------------------

    TPath = cat(3,TPath,TSegment);

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

    arcPositionPath = [
        arcPositionPath ...
        lGlobalSegment
    ];

    segmentIndexPath = [
        segmentIndexPath ...
        segment * ones(1,numel(tSegment))
    ];


    %% --------------------------------------------------------------------
    % L. Append geometry for comparison plots
    % ---------------------------------------------------------------------

    if segment > 1
        rawSegmentToAppend = rawSegment(:,2:end);
        arcSegmentToAppend = arcSegment(:,2:end);
    else
        rawSegmentToAppend = rawSegment;
        arcSegmentToAppend = arcSegment;
    end

    rawGeometryPath = [
        rawGeometryPath ...
        rawSegmentToAppend
    ];

    arcGeometryPath = [
        arcGeometryPath ...
        arcSegmentToAppend
    ];


    %% --------------------------------------------------------------------
    % M. Console summary
    % ---------------------------------------------------------------------

    fprintf( ...
        ['Segment %s -> %s | L = %.4f m | Raw = %d pts | ' ...
         'Arc = %d pts | Profile = %-11s | T = %.3f s | ' ...
         'Peak TCP = %.4f m/s\n'], ...
        waypointNames(segment), ...
        waypointNames(segment+1), ...
        segmentLength, ...
        size(rawSegment,2), ...
        size(arcSegment,2), ...
        profileInfo.type, ...
        profileInfo.T, ...
        max(tcpSpeedSegment));


    %% --------------------------------------------------------------------
    % N. Advance global offsets
    % ---------------------------------------------------------------------

    currentGlobalTime = currentGlobalTime + profileInfo.T;
    currentGlobalArcLength = currentGlobalArcLength + segmentLength;

end


%% ========================================================================
% 9. COMPLETE TRAJECTORY INFORMATION
% =========================================================================

numSamples = numel(tGlobal);
totalPathLength = currentGlobalArcLength;

fprintf('------------------------------------------------------------\n');
fprintf('Segments:                 %d\n',numSegments);
fprintf('Total trajectory samples: %d\n',numSamples);
fprintf('Total path length:        %.4f m\n',totalPathLength);
fprintf('Total duration:           %.3f s\n',tGlobal(end));
fprintf('Requested TCP speed:      %.4f m/s\n',desiredTCPSpeed);
fprintf('Arc-length spacing:       %.4f m\n',arcLengthSpacing);
fprintf('============================================================\n');


%% ========================================================================
% 10. ARC-LENGTH GEOMETRY VISUALIZATION
% =========================================================================

figure;

plot3( ...
    rawGeometryPath(1,:), ...
    rawGeometryPath(2,:), ...
    rawGeometryPath(3,:), ...
    '--', ...
    'LineWidth',1.0);

hold on;

plot3( ...
    arcGeometryPath(1,:), ...
    arcGeometryPath(2,:), ...
    arcGeometryPath(3,:), ...
    '.', ...
    'MarkerSize',10);

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

legend( ...
    'Raw geometric samples', ...
    'Arc-length-resampled geometry', ...
    'User waypoints', ...
    'Location','best');

title('Geometry Before and After Arc-Length Parameterization');


%% ========================================================================
% 11. TCP SPEED
% =========================================================================

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

title('TCP Speed Through Arc-Length-Parameterized Path');


%% ========================================================================
% 12. CUMULATIVE PHYSICAL ARC LENGTH
% =========================================================================

figure;

plot( ...
    tGlobal, ...
    arcPositionPath, ...
    'LineWidth',1.5);

grid on;

xlabel('Time [s]');
ylabel('Cumulative Arc Length [m]');

title('Physical Path Progress l(t)');


%% ========================================================================
% 13. SEQUENTIAL ADLS IK
% =========================================================================

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

        segment = segmentIndexPath(k);

        fprintf('\n');
        fprintf('IK FAILURE\n');
        fprintf('----------\n');
        fprintf('Global sample: %d / %d\n',k,numSamples);
        fprintf( ...
            'Segment: %s -> %s\n', ...
            waypointNames(segment), ...
            waypointNames(segment+1));
        fprintf('Position error: %.3e m\n',info.positionError);
        fprintf('Orientation error: %.3e rad\n',info.orientationError);

        error('Arc-length trajectory failed during sequential ADLS IK.');

    end

    qPath(:,k) = qSolution;
    qSeed = qSolution;

end

fprintf('Successful IK samples: %d / %d\n', ...
    sum(ikConverged),numSamples);

fprintf('============================================================\n');


%% ========================================================================
% 14. INDEPENDENT FK VALIDATION
% =========================================================================

positionError = zeros(1,numSamples);
orientationError = zeros(1,numSamples);

pAchieved = zeros(3,numSamples);

for k = 1:numSamples

    TAchieved = controlFK( ...
        robot, ...
        qPath(:,k));

    pAchieved(:,k) = TAchieved(1:3,4);

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
fprintf('Maximum position error:    %.3e m\n',max(positionError));
fprintf('Maximum orientation error: %.3e rad\n',max(orientationError));
fprintf('Maximum IK iterations:     %d\n',max(ikIterations));
fprintf('============================================================\n');


%% ========================================================================
% 15. JOINT VELOCITY + ACCELERATION
% =========================================================================

qDot = zeros(size(qPath));
qDDot = zeros(size(qPath));

for joint = 1:n

    qDot(joint,:) = gradient( ...
        qPath(joint,:), ...
        tGlobal);

    qDDot(joint,:) = gradient( ...
        qDot(joint,:), ...
        tGlobal);

end


%% ========================================================================
% 16. JOINT LIMIT CHECKS
% =========================================================================

qMin = robot.limits.qMin(:);
qMax = robot.limits.qMax(:);

positionViolation = false(n,1);

for joint = 1:n

    positionViolation(joint) = ...
        any(qPath(joint,:) < qMin(joint)) || ...
        any(qPath(joint,:) > qMax(joint));

end

peakJointVelocity = max(abs(qDot),[],2);

if ~isempty(robot.limits.qdMax)

    qdMax = robot.limits.qdMax(:);
    velocityViolation = peakJointVelocity > qdMax;

else

    qdMax = nan(n,1);
    velocityViolation = false(n,1);

end


%% ========================================================================
% 17. JOINT VELOCITY VIOLATION DIAGNOSTICS
% =========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' JOINT VELOCITY VIOLATION DIAGNOSTICS\n');
fprintf('============================================================\n');

anyVelocityViolation = false;

if ~isempty(robot.limits.qdMax)

    for joint = 1:n

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

                actualVelocity = abs(qDot(joint,sample));
                velocityLimit = qdMax(joint);

                percentOver = ...
                    100 * (actualVelocity / velocityLimit - 1);

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

end

if ~anyVelocityViolation
    fprintf('No joint velocity violations detected.\n');
end

fprintf('============================================================\n');


%% ========================================================================
% 18. JOINT CONTINUITY SUMMARY
% =========================================================================

deltaQ = diff(qPath,1,2);
maxJointStep = max(abs(deltaQ),[],2);

fprintf('\n');
fprintf('============================================================\n');
fprintf(' JOINT TRAJECTORY SUMMARY\n');
fprintf('============================================================\n');

for joint = 1:n

    fprintf( ...
        ['J%d | peak vel %.4f rad/s | largest step %.4f deg | ' ...
         'position violation %d | velocity violation %d\n'], ...
        joint, ...
        peakJointVelocity(joint), ...
        rad2deg(maxJointStep(joint)), ...
        positionViolation(joint), ...
        velocityViolation(joint));

end

fprintf('============================================================\n');


%% ========================================================================
% 19. DESIRED VS ACHIEVED CARTESIAN PATH
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

plot3( ...
    waypoints(1,:), ...
    waypoints(2,:), ...
    waypoints(3,:), ...
    'o', ...
    'MarkerSize',8);

grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

legend( ...
    'Desired', ...
    'IK + FK achieved', ...
    'Waypoints', ...
    'Location','best');

title('Desired vs Achieved Arc-Length Trajectory');


%% ========================================================================
% 20. JOINT POSITION
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

title('Joint Motion Through Arc-Length Trajectory');


%% ========================================================================
% 21. JOINT VELOCITY
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

title('Joint Velocity Through Arc-Length Trajectory');


%% ========================================================================
% 22. IK ITERATIONS
% =========================================================================

figure;

plot( ...
    tGlobal, ...
    ikIterations, ...
    'LineWidth',1.2);

grid on;

xlabel('Time [s]');
ylabel('ADLS iterations');

title('ADLS Difficulty Through Arc-Length Trajectory');


%% ========================================================================
% 23. ANIMATION
% =========================================================================

figure;

ax = axes;

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

show( ...
    station.mountingPlane.object, ...
    'Parent',ax);

show( ...
    station.table.object, ...
    'Parent',ax);

plot3( ...
    ax, ...
    pPath(1,:), ...
    pPath(2,:), ...
    pPath(3,:), ...
    'LineWidth',2);

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

playTrajectory( ...
    visualRobot, ...
    qPath, ...
    tGlobal, ...
    ax, ...
    playbackScale);

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
% 24. FINAL RESULT
% =========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' ARC-LENGTH TRAJECTORY PIPELINE RESULT\n');
fprintf('============================================================\n');
fprintf('Cartesian segments:      %d\n',numSegments);
fprintf('Total path length:       %.4f m\n',totalPathLength);
fprintf('IK samples passed:       %d / %d\n', ...
    sum(ikConverged),numSamples);
fprintf('Max position error:      %.3e m\n',max(positionError));
fprintf('Max orientation error:   %.3e rad\n',max(orientationError));
fprintf('Joint position limits:   %s\n',passFail(~any(positionViolation)));
fprintf('Joint velocity limits:   %s\n',passFail(~any(velocityViolation)));
fprintf('============================================================\n');


%% ========================================================================
% LOCAL FUNCTION — ORIENTATION ERROR
% =========================================================================

function angle = rotationError(RTarget,RActual)

RRelative = RTarget * RActual';

c = (trace(RRelative)-1)/2;
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
            'UR5 Arc-Length Motion | t = %.2f / %.2f s', ...
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
