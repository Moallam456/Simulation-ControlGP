%% testMultiSegmentADLS_TrapVsScurve
%
% TRAPEZOIDAL VS S-CURVE MULTI-SEGMENT CARTESIAN STRESS TEST
%
% PURPOSE
% -------
% Run the EXACT SAME multi-segment Cartesian trajectory through:
%
%       1) Trapezoidal time scaling
%       2) Jerk-limited S-curve time scaling
%
% and compare the complete trajectory pipeline:
%
%       A -> B -> C -> D -> E -> F
%
%                     ↓
%
%               SAME geometry
%               SAME orientation
%               SAME TCP speed
%               SAME TCP acceleration
%
%                     ↓
%
%          ┌──────────────────────┐
%          │                      │
%   Trapezoidal              S-Curve
%          │                      │
%          └──────────────────────┘
%
%                     ↓
%
%              Sequential ADLS IK
%
%                     ↓
%
%             Independent FK check
%
%                     ↓
%
%             q(t), qDot(t), qDDot(t)
%
%                     ↓
%
%           Joint-limit validation
%
%                     ↓
%
%             Side-by-side plots
%
%
% IMPORTANT
% ---------
% The S-curve has one additional physical requirement:
%
%       desiredTCPJerk
%
% because jerk is explicitly bounded by the S-curve.
%
%
% Both methods still stop at every Cartesian waypoint.
%
% Corner blending is NOT tested here.
%
%
% FIGURES
% -------
% Figure 1 : TCP speed
% Figure 2 : TCP acceleration
% Figure 3 : TCP jerk
% Figure 4 : Desired vs achieved Cartesian path
% Figure 5 : Joint positions
% Figure 6 : Joint velocities
% Figure 7 : Joint accelerations
% Figure 8 : ADLS iterations
% Figure 9 : Side-by-side robot animation
%
%
% NOTE ABOUT TRAPEZOIDAL JERK
% ---------------------------
% Ideal trapezoidal time scaling has discontinuous acceleration.
%
% Therefore jerk is mathematically undefined/infinite at switching
% instants.
%
% trapezoidalTimeScaling() represents those switching samples using NaN.
%
%
% NOTE ABOUT S-CURVE JERK
% -----------------------
% S-curve acceleration transitions are produced through finite jerk:
%
%       +J
%        0
%       -J
%
% depending on the current stage.


clear;
clc;
close all;


%% ========================================================================
% 1. LOAD ROBOT
% =========================================================================

robot = config.UR5();

n = robot.dof;


%% ========================================================================
% 2. VISUALIZATION MODEL
% =========================================================================

visualRobot = robotmodel.buildUR5geometry(robot);

station = environment.buildPreliminaryWeldingStation();


%% ========================================================================
% 3. STARTING ROBOT CONFIGURATION
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
% 4. STARTING TCP POSE
% =========================================================================

TStart = controlFK(robot,qStart);

pA = TStart(1:3,4);

RConstant = TStart(1:3,1:3);


%% ========================================================================
% 5. DEFINE HARSH MULTI-SEGMENT CARTESIAN PATH
% =========================================================================
%
% Same path used by testMultiSegmentCartesianADLS.
%
% Approximate lengths:
%
%       A -> B : 0.50 m
%       B -> C : 0.22 m
%       C -> D : 0.39 m
%       D -> E : 0.39 m
%       E -> F : 0.30 m

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


%% Store waypoints

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


waypointNames = [
    "A"
    "B"
    "C"
    "D"
    "E"
    "F"
];


%% ========================================================================
% 6. PHYSICAL TRAJECTORY REQUIREMENTS
% =========================================================================
%
% Both methods receive the SAME:
%
%       TCP speed
%       TCP acceleration
%
% The S-curve additionally receives:
%
%       TCP jerk
%
%
% These are TEST values.
%
% They are not yet claimed to be the final welding parameters.

desiredTCPSpeed = 0.10;      % [m/s]
desiredTCPAccel = 0.25;      % [m/s^2]

desiredTCPJerk  = 1.00;      % [m/s^3]

dt = 0.05;                   % [s]


%% ========================================================================
% 7. RUN TRAPEZOIDAL PIPELINE
% =========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('                 TRAPEZOIDAL PIPELINE\n');
fprintf('============================================================\n');


trap = runTrajectoryMethod( ...
    "Trapezoidal", ...
    robot, ...
    qStart, ...
    waypoints, ...
    waypointNames, ...
    RConstant, ...
    desiredTCPSpeed, ...
    desiredTCPAccel, ...
    desiredTCPJerk, ...
    dt);


%% ========================================================================
% 8. RUN S-CURVE PIPELINE
% =========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('                    S-CURVE PIPELINE\n');
fprintf('============================================================\n');


scurve = runTrajectoryMethod( ...
    "S-Curve", ...
    robot, ...
    qStart, ...
    waypoints, ...
    waypointNames, ...
    RConstant, ...
    desiredTCPSpeed, ...
    desiredTCPAccel, ...
    desiredTCPJerk, ...
    dt);


%% ========================================================================
% 9. GLOBAL COMPARISON SUMMARY
% =========================================================================

fprintf('\n\n');
fprintf('=================================================================================================================\n');
fprintf('                                 TRAPEZOIDAL VS S-CURVE COMPARISON\n');
fprintf('=================================================================================================================\n');

fprintf('%-34s | %-22s | %-22s\n', ...
    'Metric', ...
    'Trapezoidal', ...
    'S-Curve');

fprintf('-----------------------------------------------------------------------------------------------------------------\n');

fprintf('%-34s | %-22.3f | %-22.3f\n', ...
    'Total trajectory time [s]', ...
    trap.t(end), ...
    scurve.t(end));

fprintf('%-34s | %-22d | %-22d\n', ...
    'Number of samples', ...
    trap.numSamples, ...
    scurve.numSamples);

fprintf('%-34s | %-22d | %-22d\n', ...
    'IK converged samples', ...
    sum(trap.ikConverged), ...
    sum(scurve.ikConverged));

fprintf('%-34s | %-22d | %-22d\n', ...
    'Maximum ADLS iterations', ...
    max(trap.ikIterations), ...
    max(scurve.ikIterations));

fprintf('%-34s | %-22.3e | %-22.3e\n', ...
    'Maximum position error [m]', ...
    max(trap.positionError), ...
    max(scurve.positionError));

fprintf('%-34s | %-22.3e | %-22.3e\n', ...
    'Maximum orientation error [rad]', ...
    max(trap.orientationError), ...
    max(scurve.orientationError));

fprintf('%-34s | %-22.4f | %-22.4f\n', ...
    'Maximum TCP speed [m/s]', ...
    max(trap.tcpSpeed), ...
    max(scurve.tcpSpeed));

fprintf('%-34s | %-22.4f | %-22.4f\n', ...
    'Maximum |TCP acceleration| [m/s^2]', ...
    max(abs(trap.tcpAcceleration)), ...
    max(abs(scurve.tcpAcceleration)));

fprintf('%-34s | %-22s | %-22.4f\n', ...
    'TCP jerk behavior', ...
    'Discontinuous', ...
    max(abs(scurve.tcpJerk)));

fprintf('%-34s | %-22s | %-22s\n', ...
    'Joint position limits', ...
    passFail(~any(trap.positionViolation)), ...
    passFail(~any(scurve.positionViolation)));

fprintf('%-34s | %-22s | %-22s\n', ...
    'Joint velocity limits', ...
    passFail(~any(trap.velocityViolation)), ...
    passFail(~any(scurve.velocityViolation)));

fprintf('=================================================================================================================\n');


%% ========================================================================
% 10. PER-SEGMENT PROFILE COMPARISON
% =========================================================================

fprintf('\n');
fprintf('=================================================================================================================\n');
fprintf('                                      PER-SEGMENT COMPARISON\n');
fprintf('=================================================================================================================\n');

fprintf( ...
    '%-9s | %-10s | %-13s | %-13s | %-13s | %-13s | %-13s\n', ...
    'Segment', ...
    'Length[m]', ...
    'Trap T[s]', ...
    'S-Curve T[s]', ...
    'Trap vmax', ...
    'S vmax', ...
    'S cruise[s]');

fprintf('-----------------------------------------------------------------------------------------------------------------\n');


for segment = 1:numSegments

    fprintf( ...
        '%-9s | %-10.4f | %-13.3f | %-13.3f | %-13.4f | %-13.4f | %-13.3f\n', ...
        waypointNames(segment) + "->" + waypointNames(segment+1), ...
        trap.segmentLength(segment), ...
        trap.segmentDuration(segment), ...
        scurve.segmentDuration(segment), ...
        trap.segmentPeakSpeed(segment), ...
        scurve.segmentPeakSpeed(segment), ...
        scurve.segmentCruiseTime(segment));

end


fprintf('=================================================================================================================\n');


%% ========================================================================
% 11. PER-JOINT COMPARISON
% =========================================================================

fprintf('\n');
fprintf('=================================================================================================================\n');
fprintf('                                         PER-JOINT COMPARISON\n');
fprintf('=================================================================================================================\n');

fprintf( ...
    '%-6s | %-14s | %-14s | %-14s | %-14s | %-12s | %-12s\n', ...
    'Joint', ...
    'Trap vmax', ...
    'S vmax', ...
    'Trap amax', ...
    'S amax', ...
    'Trap limit', ...
    'S limit');

fprintf('-----------------------------------------------------------------------------------------------------------------\n');


for joint = 1:n

    fprintf( ...
        'J%-5d | %-14.4f | %-14.4f | %-14.4f | %-14.4f | %-12s | %-12s\n', ...
        joint, ...
        trap.peakJointVelocity(joint), ...
        scurve.peakJointVelocity(joint), ...
        trap.peakJointAcceleration(joint), ...
        scurve.peakJointAcceleration(joint), ...
        passFail(~trap.velocityViolation(joint)), ...
        passFail(~scurve.velocityViolation(joint)));

end


fprintf('=================================================================================================================\n');


%% ========================================================================
% 12. COMMON PLOT TIME LIMIT
% =========================================================================
%
% Use the same horizontal extent on paired plots so visual comparison is
% easier.

comparisonTimeMax = ...
    max(trap.t(end),scurve.t(end));


%% ========================================================================
% 13. FIGURE 1 — TCP SPEED
% =========================================================================

figure('Name','TCP Speed Comparison');

tiledlayout(1,2);


%% Trapezoidal

nexttile;

plot( ...
    trap.t, ...
    trap.tcpSpeed, ...
    'LineWidth',1.5);

hold on;

yline( ...
    desiredTCPSpeed, ...
    '--', ...
    'Desired TCP speed');

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('TCP speed [m/s]');

title('Trapezoidal');


%% S-Curve

nexttile;

plot( ...
    scurve.t, ...
    scurve.tcpSpeed, ...
    'LineWidth',1.5);

hold on;

yline( ...
    desiredTCPSpeed, ...
    '--', ...
    'Desired TCP speed');

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('TCP speed [m/s]');

title('S-Curve');

sgtitle('TCP Speed — Trapezoidal vs S-Curve');


%% ========================================================================
% 14. FIGURE 2 — TCP ACCELERATION
% =========================================================================

figure('Name','TCP Acceleration Comparison');

tiledlayout(1,2);


%% Trapezoidal

nexttile;

plot( ...
    trap.t, ...
    trap.tcpAcceleration, ...
    'LineWidth',1.5);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('TCP acceleration [m/s^2]');

title('Trapezoidal');


%% S-Curve

nexttile;

plot( ...
    scurve.t, ...
    scurve.tcpAcceleration, ...
    'LineWidth',1.5);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('TCP acceleration [m/s^2]');

title('S-Curve');

sgtitle('TCP Acceleration — Trapezoidal vs S-Curve');


%% ========================================================================
% 15. FIGURE 3 — TCP JERK
% =========================================================================
%
% Trapezoidal:
%
%       zero inside phases
%       undefined at instantaneous acceleration transitions
%
% The trapezoidal time-scaling function represents switching points using
% NaN.
%
%
% S-Curve:
%
%       explicitly bounded piecewise jerk.

figure('Name','TCP Jerk Comparison');

tiledlayout(1,2);


%% Trapezoidal

nexttile;

plot( ...
    trap.t, ...
    trap.tcpJerk, ...
    'LineWidth',1.5);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('TCP jerk [m/s^3]');

title('Trapezoidal — undefined at switching points');


%% S-Curve

nexttile;

plot( ...
    scurve.t, ...
    scurve.tcpJerk, ...
    'LineWidth',1.5);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('TCP jerk [m/s^3]');

title('S-Curve — bounded jerk');

sgtitle('TCP Jerk — Trapezoidal vs S-Curve');


%% ========================================================================
% 16. FIGURE 4 — DESIRED VS ACHIEVED CARTESIAN PATH
% =========================================================================

figure('Name','Cartesian Path Comparison');

tiledlayout(1,2);


%% Trapezoidal

axTrapPath = nexttile;

plot3( ...
    trap.pPath(1,:), ...
    trap.pPath(2,:), ...
    trap.pPath(3,:), ...
    'LineWidth',1.5);

hold on;

plot3( ...
    trap.pAchieved(1,:), ...
    trap.pAchieved(2,:), ...
    trap.pAchieved(3,:), ...
    '--', ...
    'LineWidth',1.5);

plot3( ...
    waypoints(1,:), ...
    waypoints(2,:), ...
    waypoints(3,:), ...
    'o', ...
    'MarkerSize',7);

grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title('Trapezoidal');

legend( ...
    'Desired', ...
    'IK + FK achieved', ...
    'Waypoints', ...
    'Location','best');


%% S-Curve

axSCurvePath = nexttile;

plot3( ...
    scurve.pPath(1,:), ...
    scurve.pPath(2,:), ...
    scurve.pPath(3,:), ...
    'LineWidth',1.5);

hold on;

plot3( ...
    scurve.pAchieved(1,:), ...
    scurve.pAchieved(2,:), ...
    scurve.pAchieved(3,:), ...
    '--', ...
    'LineWidth',1.5);

plot3( ...
    waypoints(1,:), ...
    waypoints(2,:), ...
    waypoints(3,:), ...
    'o', ...
    'MarkerSize',7);

grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');

title('S-Curve');

legend( ...
    'Desired', ...
    'IK + FK achieved', ...
    'Waypoints', ...
    'Location','best');


%% Match camera view

[az, el] = view(axTrapPath);

view(axSCurvePath, az, el);

sgtitle('Desired vs Achieved Cartesian Path');


%% ========================================================================
% 17. FIGURE 5 — JOINT POSITIONS
% =========================================================================

figure('Name','Joint Position Comparison');

tiledlayout(1,2);


%% Trapezoidal

nexttile;

plot( ...
    trap.t, ...
    rad2deg(trap.qPath.'), ...
    'LineWidth',1.1);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('Joint angle [deg]');

title('Trapezoidal');

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% S-Curve

nexttile;

plot( ...
    scurve.t, ...
    rad2deg(scurve.qPath.'), ...
    'LineWidth',1.1);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('Joint angle [deg]');

title('S-Curve');

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');

sgtitle('Joint Positions — Trapezoidal vs S-Curve');


%% ========================================================================
% 18. FIGURE 6 — JOINT VELOCITIES
% =========================================================================

figure('Name','Joint Velocity Comparison');

tiledlayout(1,2);


%% Trapezoidal

nexttile;

plot( ...
    trap.t, ...
    trap.qDot.', ...
    'LineWidth',1.1);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('Joint velocity [rad/s]');

title('Trapezoidal');

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% S-Curve

nexttile;

plot( ...
    scurve.t, ...
    scurve.qDot.', ...
    'LineWidth',1.1);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('Joint velocity [rad/s]');

title('S-Curve');

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');

sgtitle('Joint Velocities — Trapezoidal vs S-Curve');


%% ========================================================================
% 19. FIGURE 7 — JOINT ACCELERATIONS
% =========================================================================
%
% This is especially useful here because the major reason for using an
% S-curve is smoother acceleration behavior.

figure('Name','Joint Acceleration Comparison');

tiledlayout(1,2);


%% Trapezoidal

nexttile;

plot( ...
    trap.t, ...
    trap.qDDot.', ...
    'LineWidth',1.1);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('Joint acceleration [rad/s^2]');

title('Trapezoidal');

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');


%% S-Curve

nexttile;

plot( ...
    scurve.t, ...
    scurve.qDDot.', ...
    'LineWidth',1.1);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('Joint acceleration [rad/s^2]');

title('S-Curve');

legend( ...
    'J1','J2','J3','J4','J5','J6', ...
    'Location','best');

sgtitle('Joint Accelerations — Trapezoidal vs S-Curve');


%% ========================================================================
% 20. FIGURE 8 — ADLS ITERATIONS
% =========================================================================

figure('Name','ADLS Iteration Comparison');

tiledlayout(1,2);


%% Trapezoidal

nexttile;

plot( ...
    trap.t, ...
    trap.ikIterations, ...
    'LineWidth',1.2);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('ADLS iterations');

title('Trapezoidal');


%% S-Curve

nexttile;

plot( ...
    scurve.t, ...
    scurve.ikIterations, ...
    'LineWidth',1.2);

grid on;

xlim([0 comparisonTimeMax]);

xlabel('Time [s]');
ylabel('ADLS iterations');

title('S-Curve');

sgtitle('ADLS Iterations — Trapezoidal vs S-Curve');


%% ========================================================================
% 21. FIGURE 9 — SIDE-BY-SIDE ANIMATION
% =========================================================================
%
% Both trajectories are replayed using their TRUE time histories.
%
% Since their total durations may be different:
%
%       - both begin together
%       - each follows its own time vector
%       - the shorter trajectory remains at its final configuration
%
% until the longer trajectory finishes.

animationFigure = figure( ...
    'Name', ...
    'Trapezoidal vs S-Curve Animation');


layout = tiledlayout( ...
    animationFigure, ...
    1,2);


%% ------------------------------------------------------------------------
% Trapezoidal animation axes
% -------------------------------------------------------------------------

axTrap = nexttile(layout);


show( ...
    visualRobot, ...
    trap.qPath(:,1), ...
    'Parent',axTrap, ...
    'FastUpdate',true, ...
    'PreservePlot',false, ...
    'Visuals','on', ...
    'Collisions','off');


hold(axTrap,'on');

grid(axTrap,'on');

axis(axTrap,'equal');


show( ...
    station.mountingPlane.object, ...
    'Parent',axTrap);

show( ...
    station.table.object, ...
    'Parent',axTrap);


plot3( ...
    axTrap, ...
    trap.pPath(1,:), ...
    trap.pPath(2,:), ...
    trap.pPath(3,:), ...
    'LineWidth',2);


plot3( ...
    axTrap, ...
    waypoints(1,:), ...
    waypoints(2,:), ...
    waypoints(3,:), ...
    'o', ...
    'MarkerSize',7);


xlabel(axTrap,'X [m]');
ylabel(axTrap,'Y [m]');
zlabel(axTrap,'Z [m]');

title(axTrap,'Trapezoidal');


%% ------------------------------------------------------------------------
% S-Curve animation axes
% -------------------------------------------------------------------------

axSCurve = nexttile(layout);


show( ...
    visualRobot, ...
    scurve.qPath(:,1), ...
    'Parent',axSCurve, ...
    'FastUpdate',true, ...
    'PreservePlot',false, ...
    'Visuals','on', ...
    'Collisions','off');


hold(axSCurve,'on');

grid(axSCurve,'on');

axis(axSCurve,'equal');


show( ...
    station.mountingPlane.object, ...
    'Parent',axSCurve);

show( ...
    station.table.object, ...
    'Parent',axSCurve);


plot3( ...
    axSCurve, ...
    scurve.pPath(1,:), ...
    scurve.pPath(2,:), ...
    scurve.pPath(3,:), ...
    'LineWidth',2);


plot3( ...
    axSCurve, ...
    waypoints(1,:), ...
    waypoints(2,:), ...
    waypoints(3,:), ...
    'o', ...
    'MarkerSize',7);


xlabel(axSCurve,'X [m]');
ylabel(axSCurve,'Y [m]');
zlabel(axSCurve,'Z [m]');

title(axSCurve,'S-Curve');


%% Match viewing angle

[azAnim, elAnim] = view(axTrap);

view(axSCurve, azAnim, elAnim);


sgtitle( ...
    layout, ...
    'Multi-Segment ADLS Motion — Trapezoidal vs S-Curve');


%% Playback speed

playbackScale = 1.0;


%% Play immediately

playBothTrajectories( ...
    visualRobot, ...
    trap, ...
    scurve, ...
    axTrap, ...
    axSCurve, ...
    dt, ...
    playbackScale);


%% Replay button

uicontrol( ...
    animationFigure, ...
    'Style','pushbutton', ...
    'String','Replay Both', ...
    'Units','normalized', ...
    'Position',[0.44 0.01 0.12 0.05], ...
    'FontSize',11, ...
    'Callback', @(~,~) playBothTrajectories( ...
        visualRobot, ...
        trap, ...
        scurve, ...
        axTrap, ...
        axSCurve, ...
        dt, ...
        playbackScale));


%% ========================================================================
% 22. FINAL RESULT
% =========================================================================

fprintf('\n\n');
fprintf('=================================================================================================================\n');
fprintf('                                 FINAL TRAJECTORY TEST RESULT\n');
fprintf('=================================================================================================================\n');

fprintf('%-30s | %-18s | %-18s\n', ...
    'Check', ...
    'Trapezoidal', ...
    'S-Curve');

fprintf('-----------------------------------------------------------------------------------------------------------------\n');

fprintf('%-30s | %-18s | %-18s\n', ...
    'All IK samples converged', ...
    passFail(all(trap.ikConverged)), ...
    passFail(all(scurve.ikConverged)));

fprintf('%-30s | %-18s | %-18s\n', ...
    'Joint position limits', ...
    passFail(~any(trap.positionViolation)), ...
    passFail(~any(scurve.positionViolation)));

fprintf('%-30s | %-18s | %-18s\n', ...
    'Joint velocity limits', ...
    passFail(~any(trap.velocityViolation)), ...
    passFail(~any(scurve.velocityViolation)));

fprintf('%-30s | %-18s | %-18s\n', ...
    'Finite joint trajectory', ...
    passFail(all(isfinite(trap.qPath(:)))), ...
    passFail(all(isfinite(scurve.qPath(:)))));

fprintf('=================================================================================================================\n');


%% ========================================================================
% LOCAL FUNCTION
% RUN ONE COMPLETE TRAJECTORY METHOD
% =========================================================================

function result = runTrajectoryMethod( ...
    method, ...
    robot, ...
    qStart, ...
    waypoints, ...
    waypointNames, ...
    RConstant, ...
    desiredTCPSpeed, ...
    desiredTCPAccel, ...
    desiredTCPJerk, ...
    dt)


%% Basic dimensions

n = robot.dof;

numWaypoints = size(waypoints,2);

numSegments = numWaypoints - 1;


%% ========================================================================
% STORAGE
% =========================================================================

TPath = zeros(4,4,0);

pPath = zeros(3,0);

tGlobal = [];

segmentIndexPath = [];

tcpSpeed = [];

tcpAcceleration = [];

tcpJerk = [];


%% Segment statistics

segmentLength = zeros(numSegments,1);

segmentDuration = zeros(numSegments,1);

segmentPeakSpeed = zeros(numSegments,1);

segmentPeakAcceleration = zeros(numSegments,1);

segmentCruiseTime = zeros(numSegments,1);


currentGlobalTime = 0;


fprintf('\n');
fprintf('%s SEGMENT GENERATION\n',upper(char(method)));
fprintf('------------------------------------------------------------\n');


%% ========================================================================
% GENERATE EACH CARTESIAN SEGMENT
% =========================================================================

for segment = 1:numSegments

    pStartSegment = ...
        waypoints(:,segment);

    pEndSegment = ...
        waypoints(:,segment+1);


    %% Geometry

    segmentVector = ...
        pEndSegment - pStartSegment;

    L = norm(segmentVector);

    segmentLength(segment) = L;


    %% -------------------------------------------------------------
    % Convert physical limits into normalized path limits
    % -------------------------------------------------------------
    %
    %       V_TCP = L * sDot
    %
    %       A_TCP = L * sDDot
    %
    %       J_TCP = L * sDDDot

    sDotMax = ...
        desiredTCPSpeed / L;

    sDDotMax = ...
        desiredTCPAccel / L;

    sDDDotMax = ...
        desiredTCPJerk / L;


    %% -------------------------------------------------------------
    % Generate selected time scaling
    % -------------------------------------------------------------

    switch lower(method)

        case "trapezoidal"

            [ ...
                tSegment, ...
                sSegment, ...
                sDotSegment, ...
                sDDotSegment, ...
                sDDDotSegment, ...
                profileInfo ...
            ] = trapezoidalTimeScaling( ...
                    sDotMax, ...
                    sDDotMax, ...
                    dt);


            % Trapezoidal info uses tCruise.

            if isfield(profileInfo,'tCruise')

                cruiseTime = ...
                    profileInfo.tCruise;

            else

                cruiseTime = 0;

            end


        case "s-curve"

            [ ...
                tSegment, ...
                sSegment, ...
                sDotSegment, ...
                sDDotSegment, ...
                sDDDotSegment, ...
                profileInfo ...
            ] = sCurveTimeScaling( ...
                    sDotMax, ...
                    sDDotMax, ...
                    sDDDotMax, ...
                    dt);


            % S-curve info uses tConstantVelocity.

            if isfield(profileInfo,'tConstantVelocity')

                cruiseTime = ...
                    profileInfo.tConstantVelocity;

            else

                cruiseTime = 0;

            end


        otherwise

            error( ...
                "Unknown trajectory method: %s", ...
                method);

    end


    %% Force row vectors

    tSegment = tSegment(:).';

    sSegment = sSegment(:).';

    sDotSegment = sDotSegment(:).';

    sDDotSegment = sDDotSegment(:).';

    sDDDotSegment = sDDDotSegment(:).';


    %% -------------------------------------------------------------
    % Convert normalized derivatives into physical Cartesian quantities
    % -------------------------------------------------------------

    tcpSpeedSegment = ...
        L * sDotSegment;

    tcpAccelerationSegment = ...
        L * sDDotSegment;

    tcpJerkSegment = ...
        L * sDDDotSegment;


    %% -------------------------------------------------------------
    % Generate straight Cartesian path
    % -------------------------------------------------------------

    pSegment = linearInterpolation( ...
        pStartSegment, ...
        pEndSegment, ...
        sSegment);


    %% -------------------------------------------------------------
    % Build complete Cartesian pose path
    % -------------------------------------------------------------

    numberOfSegmentSamples = ...
        numel(tSegment);

    TSegment = ...
        zeros(4,4,numberOfSegmentSamples);


    for k = 1:numberOfSegmentSamples

        T = eye(4);

        T(1:3,1:3) = ...
            RConstant;

        T(1:3,4) = ...
            pSegment(:,k);

        TSegment(:,:,k) = T;

    end


    %% -------------------------------------------------------------
    % Store segment statistics BEFORE removing duplicate point
    % -------------------------------------------------------------

    segmentDuration(segment) = ...
        profileInfo.T;

    segmentPeakSpeed(segment) = ...
        max(tcpSpeedSegment);

    segmentPeakAcceleration(segment) = ...
        max(abs(tcpAccelerationSegment));

    segmentCruiseTime(segment) = ...
        cruiseTime;


    %% Console output

    fprintf( ...
        ['%-7s -> %-7s | L = %.4f m | ' ...
         'T = %.3f s | Vpeak = %.4f m/s | ' ...
         'Apeak = %.4f m/s^2 | cruise = %.3f s\n'], ...
        waypointNames(segment), ...
        waypointNames(segment+1), ...
        L, ...
        profileInfo.T, ...
        max(tcpSpeedSegment), ...
        max(abs(tcpAccelerationSegment)), ...
        cruiseTime);


    %% -------------------------------------------------------------
    % Remove duplicated waypoint sample
    % -------------------------------------------------------------

    if segment > 1

        tSegment = ...
            tSegment(2:end);

        pSegment = ...
            pSegment(:,2:end);

        TSegment = ...
            TSegment(:,:,2:end);

        tcpSpeedSegment = ...
            tcpSpeedSegment(2:end);

        tcpAccelerationSegment = ...
            tcpAccelerationSegment(2:end);

        tcpJerkSegment = ...
            tcpJerkSegment(2:end);

    end


    %% -------------------------------------------------------------
    % Convert local time to global time
    % -------------------------------------------------------------

    tSegmentGlobal = ...
        currentGlobalTime + tSegment;


    %% -------------------------------------------------------------
    % Append
    % -------------------------------------------------------------

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


    tcpSpeed = [
        tcpSpeed ...
        tcpSpeedSegment
    ];


    tcpAcceleration = [
        tcpAcceleration ...
        tcpAccelerationSegment
    ];


    tcpJerk = [
        tcpJerk ...
        tcpJerkSegment
    ];


    segmentIndexPath = [
        segmentIndexPath ...
        segment * ones(1,numel(tSegment))
    ];


    %% Update global time

    currentGlobalTime = ...
        currentGlobalTime + profileInfo.T;

end


numSamples = ...
    numel(tGlobal);


fprintf('------------------------------------------------------------\n');

fprintf( ...
    'Total duration : %.3f s\n', ...
    tGlobal(end));

fprintf( ...
    'Total samples  : %d\n', ...
    numSamples);


%% ========================================================================
% SEQUENTIAL ADLS IK
% =========================================================================

fprintf('\n%s SEQUENTIAL ADLS IK\n',upper(char(method)));
fprintf('------------------------------------------------------------\n');


qPath = ...
    zeros(n,numSamples);

ikConverged = ...
    false(1,numSamples);

ikIterations = ...
    zeros(1,numSamples);


qSeed = qStart;


for k = 1:numSamples

    [qSolution,info] = ADLS_IK( ...
        robot, ...
        TPath(:,:,k), ...
        qSeed);


    ikConverged(k) = ...
        info.converged;

    ikIterations(k) = ...
        info.iterations;


    if ~info.converged

        segment = ...
            segmentIndexPath(k);


        fprintf('\nIK FAILURE\n');

        fprintf( ...
            'Method: %s\n', ...
            method);

        fprintf( ...
            'Sample: %d / %d\n', ...
            k, ...
            numSamples);

        fprintf( ...
            'Time: %.4f s\n', ...
            tGlobal(k));

        fprintf( ...
            'Segment: %s -> %s\n', ...
            waypointNames(segment), ...
            waypointNames(segment+1));

        fprintf( ...
            'Position error: %.3e m\n', ...
            info.positionError);

        fprintf( ...
            'Orientation error: %.3e rad\n', ...
            info.orientationError);


        error( ...
            "%s trajectory failed during ADLS IK.", ...
            method);

    end


    qPath(:,k) = ...
        qSolution;

    qSeed = ...
        qSolution;

end


fprintf( ...
    'IK convergence: %d / %d\n', ...
    sum(ikConverged), ...
    numSamples);


%% ========================================================================
% INDEPENDENT FK VALIDATION
% =========================================================================

positionError = ...
    zeros(1,numSamples);

orientationError = ...
    zeros(1,numSamples);

pAchieved = ...
    zeros(3,numSamples);


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


fprintf( ...
    'Max position error    : %.3e m\n', ...
    max(positionError));

fprintf( ...
    'Max orientation error : %.3e rad\n', ...
    max(orientationError));

fprintf( ...
    'Max ADLS iterations   : %d\n', ...
    max(ikIterations));


%% ========================================================================
% JOINT VELOCITY
% =========================================================================

qDot = ...
    zeros(size(qPath));


for joint = 1:n

    qDot(joint,:) = gradient( ...
        qPath(joint,:), ...
        tGlobal);

end


%% ========================================================================
% JOINT ACCELERATION
% =========================================================================

qDDot = ...
    zeros(size(qPath));


for joint = 1:n

    qDDot(joint,:) = gradient( ...
        qDot(joint,:), ...
        tGlobal);

end


%% ========================================================================
% JOINT POSITION LIMITS
% =========================================================================

qMin = ...
    robot.limits.qMin(:);

qMax = ...
    robot.limits.qMax(:);


positionViolation = ...
    false(n,1);


for joint = 1:n

    positionViolation(joint) = ...
        any(qPath(joint,:) < qMin(joint)) || ...
        any(qPath(joint,:) > qMax(joint));

end


%% ========================================================================
% JOINT VELOCITY LIMITS
% =========================================================================

peakJointVelocity = ...
    max(abs(qDot),[],2);


if ~isempty(robot.limits.qdMax)

    qdMax = ...
        robot.limits.qdMax(:);

    velocityViolation = ...
        peakJointVelocity > qdMax;

else

    qdMax = ...
        nan(n,1);

    velocityViolation = ...
        false(n,1);

end


%% ========================================================================
% JOINT ACCELERATION SUMMARY
% =========================================================================

peakJointAcceleration = ...
    max(abs(qDDot),[],2);


%% ========================================================================
% JOINT CONTINUITY
% =========================================================================

deltaQ = ...
    diff(qPath,1,2);

maxJointStep = ...
    max(abs(deltaQ),[],2);


%% ========================================================================
% EXACT VELOCITY VIOLATION DIAGNOSTICS
% =========================================================================

if any(velocityViolation)

    fprintf('\nVELOCITY VIOLATIONS — %s\n',upper(char(method)));
    fprintf('------------------------------------------------------------\n');


    for joint = 1:n

        violatingSamples = find( ...
            abs(qDot(joint,:)) > qdMax(joint));


        for idx = 1:numel(violatingSamples)

            sample = ...
                violatingSamples(idx);

            segment = ...
                segmentIndexPath(sample);


            fprintf( ...
                ['J%d | t = %.3f s | %s -> %s | ' ...
                 '|qDot| = %.4f | limit = %.4f rad/s\n'], ...
                joint, ...
                tGlobal(sample), ...
                waypointNames(segment), ...
                waypointNames(segment+1), ...
                abs(qDot(joint,sample)), ...
                qdMax(joint));

        end

    end

else

    fprintf( ...
        'Joint velocity violations: NONE\n');

end


%% ========================================================================
% PACKAGE RESULTS
% =========================================================================

result.method = method;

result.t = tGlobal;

result.numSamples = numSamples;

result.TPath = TPath;

result.pPath = pPath;

result.pAchieved = pAchieved;

result.segmentIndexPath = segmentIndexPath;


result.tcpSpeed = tcpSpeed;

result.tcpAcceleration = tcpAcceleration;

result.tcpJerk = tcpJerk;


result.qPath = qPath;

result.qDot = qDot;

result.qDDot = qDDot;


result.ikConverged = ikConverged;

result.ikIterations = ikIterations;


result.positionError = positionError;

result.orientationError = orientationError;


result.positionViolation = positionViolation;

result.velocityViolation = velocityViolation;


result.peakJointVelocity = peakJointVelocity;

result.peakJointAcceleration = peakJointAcceleration;

result.maxJointStep = maxJointStep;


result.segmentLength = segmentLength;

result.segmentDuration = segmentDuration;

result.segmentPeakSpeed = segmentPeakSpeed;

result.segmentPeakAcceleration = segmentPeakAcceleration;

result.segmentCruiseTime = segmentCruiseTime;

end


%% ========================================================================
% LOCAL FUNCTION
% ORIENTATION ERROR
% =========================================================================

function angle = rotationError(RTarget,RActual)

RRelative = ...
    RTarget * RActual';

c = ...
    (trace(RRelative)-1)/2;

c = ...
    max(-1,min(1,c));

angle = ...
    acos(c);

end


%% ========================================================================
% LOCAL FUNCTION
% PASS / FAIL STRING
% =========================================================================

function txt = passFail(tf)

if tf

    txt = 'PASS';

else

    txt = 'FAIL';

end

end


%% ========================================================================
% LOCAL FUNCTION
% SIDE-BY-SIDE TRAJECTORY PLAYBACK
% =========================================================================

function playBothTrajectories( ...
    robotModel, ...
    trap, ...
    scurve, ...
    axTrap, ...
    axSCurve, ...
    playbackDt, ...
    playbackScale)
%
% Both trajectories are replayed against a COMMON real-time clock.
%
% This preserves their actual timing differences.
%
% Example:
%
%       Trapezoidal T = 20.1 s
%
%       S-Curve T     = 21.4 s
%
%
% At t = 20.1 s:
%
%       Trapezoidal remains at final configuration
%
% while:
%
%       S-Curve continues until its own final time.

finalTime = ...
    max(trap.t(end),scurve.t(end));


tPlayback = ...
    0:playbackDt:finalTime;


if tPlayback(end) < finalTime

    tPlayback = [
        tPlayback ...
        finalTime
    ];

end


for k = 1:numel(tPlayback)

    currentTime = ...
        tPlayback(k);


    %% -------------------------------------------------------------
    % Find corresponding trapezoidal sample
    % -------------------------------------------------------------

    trapIndex = find( ...
        trap.t <= currentTime, ...
        1, ...
        'last');


    if isempty(trapIndex)

        trapIndex = 1;

    end


    %% -------------------------------------------------------------
    % Find corresponding S-curve sample
    % -------------------------------------------------------------

    sCurveIndex = find( ...
        scurve.t <= currentTime, ...
        1, ...
        'last');


    if isempty(sCurveIndex)

        sCurveIndex = 1;

    end


    %% -------------------------------------------------------------
    % Draw trapezoidal robot
    % -------------------------------------------------------------

    show( ...
        robotModel, ...
        trap.qPath(:,trapIndex), ...
        'Parent',axTrap, ...
        'FastUpdate',true, ...
        'PreservePlot',false, ...
        'Visuals','on', ...
        'Collisions','off');


    title( ...
        axTrap, ...
        sprintf( ...
            'Trapezoidal | %.2f / %.2f s', ...
            min(currentTime,trap.t(end)), ...
            trap.t(end)));


    %% -------------------------------------------------------------
    % Draw S-curve robot
    % -------------------------------------------------------------

    show( ...
        robotModel, ...
        scurve.qPath(:,sCurveIndex), ...
        'Parent',axSCurve, ...
        'FastUpdate',true, ...
        'PreservePlot',false, ...
        'Visuals','on', ...
        'Collisions','off');


    title( ...
        axSCurve, ...
        sprintf( ...
            'S-Curve | %.2f / %.2f s', ...
            min(currentTime,scurve.t(end)), ...
            scurve.t(end)));


    drawnow;


    %% -------------------------------------------------------------
    % Preserve playback timing
    % -------------------------------------------------------------

    if k < numel(tPlayback)

        pause( ...
            playbackScale * ...
            (tPlayback(k+1)-tPlayback(k)));

    end

end

end