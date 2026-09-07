function visualKinematicsTest(testNumber)
% validation.visualKinematicsTest(testNumber)
%
% VISUAL KINEMATICS VALIDATION
% Simulation Team vs Control Team
%
% This function visually checks:
%
%   A) FORWARD KINEMATICS
%      - Animate the preliminary robot geometry from q = 0 to qTarget.
%      - Plot the TCP trajectory predicted by Control FK.
%      - Plot the TCP trajectory produced by the independent Simulation model.
%      - At the target configuration, both TCP markers/frames should overlap.
%
%   B) INVERSE KINEMATICS
%      - Generate a target pose using the independent Simulation model.
%      - Solve that target using controlIK().
%      - Animate the robot from the IK seed to the returned solution.
%      - Overlay the target TCP frame and the achieved Simulation TCP frame.
%
% Usage:
%       validation.visualKinematicsTest(1)
%
% testNumber may be 1...5 and corresponds to the same configurations used
% in the numerical cross-validation.
%
% Required:
%   +config/UR5.m
%   controlFK.m
%   controlIK.m
%   controlJacobian.m
%   dhTransform.m
%   Robotics System Toolbox

if nargin < 1
    testNumber = 1;
end

validateattributes(testNumber, {'numeric'}, ...
    {'scalar','integer','>=',1,'<=',5});

clc;

%% ------------------------------------------------------------
% 1. SHARED ROBOT CONFIGURATION
% -------------------------------------------------------------

robot = config.UR5();

a           = robot.kinematics.a(:);
d           = robot.kinematics.d(:);
alpha       = robot.kinematics.alpha(:);
thetaOffset = robot.kinematics.thetaOffset(:);

n = robot.dof;

%% ------------------------------------------------------------
% 2. BUILD INDEPENDENT SIMULATION RIGID BODY TREE
% -------------------------------------------------------------

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
setFixedTransform(tcpJoint,robot.tool.T_F_TCP);
tcpBody.Joint = tcpJoint;
addBody(simRobot,tcpBody,parentName);

%% ------------------------------------------------------------
% 3. SAME FIVE VALIDATION CONFIGURATIONS
% -------------------------------------------------------------

qTestsDeg = [
     30   -45    60    20   -30    45
    -30   -60    80   -20    40    60
     60   -30    40    90   -45   -30
    -60   -75    70    30    45   -60
     15   -90    90     0    30    90
];

qTarget = deg2rad(qTestsDeg(testNumber,:)).';

fprintf('\n============================================================\n');
fprintf(' VISUAL KINEMATICS TEST - CASE %d\n',testNumber);
fprintf('============================================================\n');
fprintf('Target-generating joint configuration [deg]:\n');
disp(qTestsDeg(testNumber,:));

%% ------------------------------------------------------------
% 4. FIGURE SETUP
% -------------------------------------------------------------

fig = figure( ...
    'Name',sprintf('Visual Kinematics Validation - Test %d',testNumber), ...
    'Color','w');

ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
view(ax,135,25);

xlabel(ax,'X [m]');
ylabel(ax,'Y [m]');
zlabel(ax,'Z [m]');

title(ax, ...
    sprintf('Visual Kinematics Validation - Test %d',testNumber));

% Workspace limits based on robot reach estimate from the kinematic chain.
reachEstimate = sum(abs(a)) + sum(abs(d)) + ...
    norm(robot.tool.T_F_TCP(1:3,4));

if reachEstimate <= 0
    reachEstimate = 1;
end

margin = 1.20*reachEstimate;

xlim(ax,[-margin margin]);
ylim(ax,[-margin margin]);
zlim(ax,[-0.15*margin margin]);

%% ------------------------------------------------------------
% 5. FK VISUAL CROSS-VALIDATION
% -------------------------------------------------------------

fprintf('\n--- PHASE A: VISUAL FK CROSS-VALIDATION ---\n');

qStart = zeros(n,1);

% Clip start inside limits if needed.
qMin = robot.limits.qMin(:);
qMax = robot.limits.qMax(:);
qStart = min(max(qStart,qMin),qMax);

numFrames = 80;

qTrajectory = zeros(n,numFrames);

for j = 1:n
    qTrajectory(j,:) = linspace(qStart(j),qTarget(j),numFrames);
end

controlPath = nan(3,numFrames);
simulationPath = nan(3,numFrames);

for k = 1:numFrames

    q = qTrajectory(:,k);

    % Control-team FK
    TControl = controlFK(robot,q);
    controlPath(:,k) = TControl(1:3,4);

    % Independent Simulation FK
    TSimulation = getTransform( ...
        simRobot, ...
        q + thetaOffset, ...
        'TCP');

    simulationPath(:,k) = TSimulation(1:3,4);

end

% Draw the complete expected paths faintly first.
hControlPath = plot3( ...
    ax, ...
    controlPath(1,:), ...
    controlPath(2,:), ...
    controlPath(3,:), ...
    '-', ...
    'LineWidth',2.0, ...
    'DisplayName','Control FK TCP path');

hSimulationPath = plot3( ...
    ax, ...
    simulationPath(1,:), ...
    simulationPath(2,:), ...
    simulationPath(3,:), ...
    '--', ...
    'LineWidth',1.5, ...
    'DisplayName','Simulation TCP path');

% Create TCP markers.
hControlTCP = plot3( ...
    ax,nan,nan,nan,'o', ...
    'MarkerSize',9, ...
    'LineWidth',2, ...
    'DisplayName','Control FK TCP');

hSimulationTCP = plot3( ...
    ax,nan,nan,nan,'x', ...
    'MarkerSize',11, ...
    'LineWidth',2, ...
    'DisplayName','Simulation TCP');

legend(ax,'Location','bestoutside');

for k = 1:numFrames

    q = qTrajectory(:,k);

    % Clear only previously drawn robot geometry while preserving path/markers.
    oldRobot = findobj(ax,'Tag','VisualValidationRobot');
    delete(oldRobot);

    drawPreliminaryRobot(ax,simRobot,q + thetaOffset);

    set(hControlTCP, ...
        'XData',controlPath(1,k), ...
        'YData',controlPath(2,k), ...
        'ZData',controlPath(3,k));

    set(hSimulationTCP, ...
        'XData',simulationPath(1,k), ...
        'YData',simulationPath(2,k), ...
        'ZData',simulationPath(3,k));

    title(ax,sprintf( ...
        'Phase A - FK Visual Check | Frame %d/%d', ...
        k,numFrames));

    drawnow;

end

% Final FK matrices and error.
TControlFinal = controlFK(robot,qTarget);

TSimulationFinal = getTransform( ...
    simRobot, ...
    qTarget + thetaOffset, ...
    'TCP');

fkPositionError = norm( ...
    TControlFinal(1:3,4) - ...
    TSimulationFinal(1:3,4));

fkOrientationError = rotationError( ...
    TControlFinal(1:3,1:3), ...
    TSimulationFinal(1:3,1:3));

drawFrame(ax,TControlFinal,0.10*reachEstimate,'Control TCP');
drawFrame(ax,TSimulationFinal,0.075*reachEstimate,'Simulation TCP');

fprintf('FK final position error: %.3e m\n',fkPositionError);
fprintf('FK final orientation error: %.3e rad\n',fkOrientationError);

title(ax,sprintf( ...
    ['Phase A PASS candidate | FK position error = %.2e m | ', ...
     'orientation error = %.2e rad'], ...
    fkPositionError,fkOrientationError));

drawnow;
pause(1.0);

%% ------------------------------------------------------------
% 6. IK VISUAL CROSS-VALIDATION
% -------------------------------------------------------------

fprintf('\n--- PHASE B: VISUAL IK CROSS-VALIDATION ---\n');

% Target is generated ONLY from Simulation.
TTarget = TSimulationFinal;

qSeed = zeros(n,1);

[qSolution,info] = controlIK(robot,TTarget,qSeed);
qSolution = qSolution(:);

TAchieved = getTransform( ...
    simRobot, ...
    qSolution + thetaOffset, ...
    'TCP');

ikPositionError = norm( ...
    TTarget(1:3,4) - ...
    TAchieved(1:3,4));

ikOrientationError = rotationError( ...
    TTarget(1:3,1:3), ...
    TAchieved(1:3,1:3));

fprintf('Control IK converged: %d\n',info.converged);
fprintf('Iterations: %d\n',info.iterations);
fprintf('IK position error verified by Simulation: %.3e m\n', ...
    ikPositionError);
fprintf('IK orientation error verified by Simulation: %.3e rad\n', ...
    ikOrientationError);

fprintf('\nControl IK solution [deg]:\n');
disp(rad2deg(qSolution).');

% Remove FK paths/markers to make the IK visual clearer.
delete(hControlPath);
delete(hSimulationPath);
delete(hControlTCP);
delete(hSimulationTCP);

% Animate from seed to IK solution.
numIKFrames = 100;

qIKTrajectory = zeros(n,numIKFrames);

for j = 1:n
    qIKTrajectory(j,:) = linspace(qSeed(j),qSolution(j),numIKFrames);
end

% Draw target TCP frame once.
hTargetFrame = drawFrame( ...
    ax,TTarget,0.11*reachEstimate,'IK Target TCP');

for k = 1:numIKFrames

    q = qIKTrajectory(:,k);

    oldRobot = findobj(ax,'Tag','VisualValidationRobot');
    delete(oldRobot);

    drawPreliminaryRobot(ax,simRobot,q + thetaOffset);

    TCurrentSim = getTransform( ...
        simRobot, ...
        q + thetaOffset, ...
        'TCP');

    % Current TCP point during IK-result animation.
    hCurrent = findobj(ax,'Tag','CurrentIKTCP');

    if isempty(hCurrent)
        hCurrent = plot3( ...
            ax, ...
            TCurrentSim(1,4), ...
            TCurrentSim(2,4), ...
            TCurrentSim(3,4), ...
            'o', ...
            'MarkerSize',8, ...
            'LineWidth',2, ...
            'Tag','CurrentIKTCP', ...
            'DisplayName','Current IK robot TCP');
    else
        set(hCurrent, ...
            'XData',TCurrentSim(1,4), ...
            'YData',TCurrentSim(2,4), ...
            'ZData',TCurrentSim(3,4));
    end

    title(ax,sprintf( ...
        'Phase B - IK Solution Animation | Frame %d/%d', ...
        k,numIKFrames));

    drawnow;

end

% Draw achieved frame over target frame.
drawFrame( ...
    ax, ...
    TAchieved, ...
    0.075*reachEstimate, ...
    'IK Achieved TCP');

% Final target and achieved position markers.
plot3( ...
    ax, ...
    TTarget(1,4),TTarget(2,4),TTarget(3,4), ...
    's', ...
    'MarkerSize',12, ...
    'LineWidth',2, ...
    'DisplayName','IK target position');

plot3( ...
    ax, ...
    TAchieved(1,4),TAchieved(2,4),TAchieved(3,4), ...
    '+', ...
    'MarkerSize',13, ...
    'LineWidth',2, ...
    'DisplayName','IK achieved position');

withinLimits = all(qSolution >= qMin-1e-12) && ...
               all(qSolution <= qMax+1e-12);

overallPass = ...
    info.converged && ...
    withinLimits && ...
    ikPositionError < 2e-6 && ...
    ikOrientationError < 2e-6;

if overallPass
    resultText = 'PASS';
else
    resultText = 'FAIL';
end

title(ax,sprintf( ...
    ['Phase B - IK VISUAL RESULT: %s | pos %.2e m | ', ...
     'ori %.2e rad'], ...
    resultText,ikPositionError,ikOrientationError));

legend(ax,'Location','bestoutside');

fprintf('\n============================================================\n');
fprintf(' VISUAL KINEMATICS TEST SUMMARY\n');
fprintf('============================================================\n');
fprintf('Test case: %d\n',testNumber);
fprintf('FK position error: %.3e m\n',fkPositionError);
fprintf('FK orientation error: %.3e rad\n',fkOrientationError);
fprintf('IK converged: %d\n',info.converged);
fprintf('IK solution within limits: %d\n',withinLimits);
fprintf('IK position error: %.3e m\n',ikPositionError);
fprintf('IK orientation error: %.3e rad\n',ikOrientationError);
fprintf('Overall visual/numerical result: %s\n',resultText);
fprintf('============================================================\n');

% Keep variable alive intentionally; frame handles remain visible.
if isempty(hTargetFrame) %#ok<UNRCH>
    disp('');
end

end


%% ============================================================
% LOCAL HELPER: DRAW PRELIMINARY GEOMETRICAL ROBOT
% =============================================================
function drawPreliminaryRobot(ax,robot,qReference)

bodyNames = robot.BodyNames;

points = zeros(3,numel(bodyNames)+1);
points(:,1) = [0;0;0];

for i = 1:numel(bodyNames)

    T = getTransform(robot,qReference,bodyNames{i});

    points(:,i+1) = T(1:3,4);

end

% Draw links as a clean preliminary geometric representation.
plot3( ...
    ax, ...
    points(1,:), ...
    points(2,:), ...
    points(3,:), ...
    '-', ...
    'LineWidth',5, ...
    'Tag','VisualValidationRobot', ...
    'HandleVisibility','off');

% Draw joint centers.
scatter3( ...
    ax, ...
    points(1,:), ...
    points(2,:), ...
    points(3,:), ...
    50, ...
    'filled', ...
    'Tag','VisualValidationRobot', ...
    'HandleVisibility','off');

% Draw base marker.
plot3( ...
    ax, ...
    0,0,0, ...
    's', ...
    'MarkerSize',10, ...
    'LineWidth',2, ...
    'Tag','VisualValidationRobot', ...
    'HandleVisibility','off');

end


%% ============================================================
% LOCAL HELPER: DRAW A 3-D COORDINATE FRAME
% =============================================================
function handles = drawFrame(ax,T,scale,labelText)

p = T(1:3,4);
R = T(1:3,1:3);

xEnd = p + scale*R(:,1);
yEnd = p + scale*R(:,2);
zEnd = p + scale*R(:,3);

handles = gobjects(4,1);

handles(1) = quiver3( ...
    ax,p(1),p(2),p(3), ...
    xEnd(1)-p(1),xEnd(2)-p(2),xEnd(3)-p(3), ...
    0,'LineWidth',2,'MaxHeadSize',0.8, ...
    'HandleVisibility','off');

handles(2) = quiver3( ...
    ax,p(1),p(2),p(3), ...
    yEnd(1)-p(1),yEnd(2)-p(2),yEnd(3)-p(3), ...
    0,'LineWidth',2,'MaxHeadSize',0.8, ...
    'HandleVisibility','off');

handles(3) = quiver3( ...
    ax,p(1),p(2),p(3), ...
    zEnd(1)-p(1),zEnd(2)-p(2),zEnd(3)-p(3), ...
    0,'LineWidth',2,'MaxHeadSize',0.8, ...
    'HandleVisibility','off');

handles(4) = text( ...
    ax, ...
    p(1),p(2),p(3), ...
    ['  ' labelText], ...
    'FontWeight','bold', ...
    'HandleVisibility','off');

end


%% ============================================================
% LOCAL HELPER: TRUE ROTATION-ANGLE ERROR
% =============================================================
function angle = rotationError(R1,R2)

RRelative = R1*R2';

c = (trace(RRelative)-1)/2;
c = max(-1,min(1,c));

angle = acos(c);

end
