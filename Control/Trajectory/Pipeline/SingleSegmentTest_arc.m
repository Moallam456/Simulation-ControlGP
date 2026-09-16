%% trajectoryPipelineArcLength_circle
%
% SINGLE-SEGMENT CIRCULAR-ARC TRAJECTORY PIPELINE
% WITH PATH-INDUCED ROTATION + LEFTOVER-TWIST ORIENTATION
%
% Operator supplies THREE positions:  P_start, P_mid, P_end.
% A circular arc is fitted through them.  Position follows the arc;
% orientation follows the arc's natural path-induced rotation (torch
% angle locked to the seam) plus a graduated leftover twist that lands
% exactly on the taught end orientation at s = 1.

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
station     = environment.buildPreliminaryWeldingStation();


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

TStart    = controlFK(robot,qStart);
pA        = TStart(1:3,4);
RConstant = TStart(1:3,1:3);


%% ========================================================================
% 5. DEFINE THREE POSITIONS ON THE CIRCLE  <<<<<<<<<< EDIT THIS BLOCK >>>>
% =========================================================================
%
% P_start must equal the current TCP position pA (so the first IK sample
% matches qStart).  P_mid lies on the arc between start and end.
% P_end is the end of the arc.  All three define the plane and the sweep.

P_start = pA;

P_mid   = pA + [
     0.20
     0.14
     0.00
];

P_end   = pA + [
     0.40
     0.00
     0.00
];


%% ========================================================================
% 5b. ARC GEOMETRY + ORIENTATION SETUP
% =========================================================================
%
% generateCircleWaypoints() returns both the sampled positions and the
% arc metadata (center, radius, plane normal b, total sweep thetaTotal).
% The metadata is what the orientation pipeline needs.

numGeometryPointsPerSegment = 200;   % generous for a curved segment

[rawSegment, arcInfo] = generateCircleWaypoints( ...
    P_start, P_mid, P_end, numGeometryPointsPerSegment);

% ----- Editable: start and end orientations ----------------------------
% R_start MUST stay equal to RConstant so the first IK sample matches
% qStart.  R_end is composed from RConstant via a ZYX Euler rotation.
% Change the three deg2rad(...) values to set how the torch ends up.

R_start = RConstant;
R_end   = RConstant * eulZYX_local( deg2rad(  0), deg2rad( 0), deg2rad( 0) );

q_start_orientation = rotm2quat_custom(R_start);
q_end_orientation   = rotm2quat_custom(R_end);

fprintf('\nArc geometry:\n');
fprintf('  center     = [%.4f %.4f %.4f] m\n', arcInfo.center);
fprintf('  radius     = %.4f m\n',             arcInfo.radius);
fprintf('  plane n    = [%.4f %.4f %.4f]\n',   arcInfo.axis);
fprintf('  sweep      = %.2f deg\n',           rad2deg(arcInfo.thetaTotal));
fprintf('  arc length = %.4f m\n',             arcInfo.radius*abs(arcInfo.thetaTotal));

fprintf('\nArc waypoints (start / mid / end):\n');
fprintf('  P_start = [%.4f %.4f %.4f]\n', rawSegment(:,1));
fprintf('  P_mid   = [%.4f %.4f %.4f]\n', rawSegment(:,round(end/2)));
fprintf('  P_end   = [%.4f %.4f %.4f]\n', rawSegment(:,end));


%% ========================================================================
% 6. PATH + TRAJECTORY SETTINGS
% =========================================================================

arcLengthSpacing = 0.005;    % [m]
desiredTCPSpeed  = 0.10;     % [m/s]
desiredTCPAccel  = 0.25;     % [m/s^2]
desiredTCPJerk   = 1.00;    % [m/s^3]
dt               = 0.05;     % [s]


%% ========================================================================
% 7. GLOBAL STORAGE
% =========================================================================

TPath    = zeros(4,4,0);
pPath    = zeros(3,0);
quatPath = zeros(4,0);

tGlobal          = [];
tcpSpeedPath     = [];
arcPositionPath  = [];
segmentIndexPath = [];

rawGeometryPath = zeros(3,0);
arcGeometryPath = zeros(3,0);

currentGlobalTime      = 0;
currentGlobalArcLength = 0;


%% ========================================================================
% 8. ARC-LENGTH PARAMETERIZE + TIME-SCALE THE SEGMENT
% =========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' CIRCULAR-ARC TRAJECTORY PIPELINE\n');
fprintf('============================================================\n');

segment = 1;

% A. Raw geometry was already produced in 5b (rawSegment).

% B. Arc-length parameterization ---------------------------------------
[arcSegment, lOriginal, lArc] = ...
    arcLengthParameterize(rawSegment, arcLengthSpacing);

segmentLength = lOriginal(end);

if segmentLength <= eps
    error('Arc segment has zero length.');
end

% C. Normalized limits -------------------------------------------------
%
% The S-curve time scaling operates on the normalized path coordinate:
%
%       0 <= s <= 1
%
% Therefore:
%
%   sDotMax   = vMax / L
%   sDDotMax  = aMax / L
%   sDDDotMax = jMax / L
%
% where L is the physical arc length.

sDotMax   = desiredTCPSpeed / segmentLength;
sDDotMax  = desiredTCPAccel / segmentLength;
sDDDotMax = desiredTCPJerk / segmentLength;

% D. S-curve time scaling ----------------------------------------------
[tSegment, sSegment, sDotSegment, sDDotSegment, sDDDotSegment, profileInfo] = ...
    sCurveTimeScaling(sDotMax, sDDotMax, sDDDotMax, dt);

tSegment      = tSegment(:).';
sSegment      = sSegment(:).';
sDotSegment   = sDotSegment(:).';
sDDotSegment  = sDDotSegment(:).';
sDDDotSegment = sDDDotSegment(:).';

% D.5. Orientation along the arc ---------------------------------------
% Path-induced rotation about arcInfo.axis by s*thetaTotal, blended with
% the leftover twist so that the final sample lands exactly on
% q_end_orientation.
quatSegment = zeros(4,numel(sSegment));

for k = 1:numel(sSegment)
    quatSegment(:,k) = orientationFromArc( ...
        q_start_orientation, ...
        q_end_orientation, ...
        arcInfo.axis, ...
        arcInfo.thetaTotal, ...
        sSegment(k));
end

% E. Physical arc length ----------------------------------------------
lTimed          = segmentLength * sSegment;
tcpSpeedSegment = segmentLength * sDotSegment;

% F. Cartesian position p(l(t)) ---------------------------------------
pSegment = zeros(3,numel(lTimed));
for axisIndex = 1:3
    pSegment(axisIndex,:) = interp1( ...
        lArc, arcSegment(axisIndex,:), lTimed, 'linear');
end

% G. Cartesian pose ---------------------------------------------------
numberOfSegmentSamples = numel(tSegment);
TSegment = zeros(4,4,numberOfSegmentSamples);

for k = 1:numberOfSegmentSamples
    T = eye(4);
    T(1:3,1:3) = quat2rotm_custom(quatSegment(:,k));
    T(1:3,4)   = pSegment(:,k);
    TSegment(:,:,k) = T;
end

% H. Global arc length -----------------------------------------------
lGlobalSegment = currentGlobalArcLength + lTimed;

% I. Local -> global time --------------------------------------------
tSegmentGlobal = currentGlobalTime + tSegment;

% J. Append to global storage ----------------------------------------
TPath    = cat(3,TPath,TSegment);
pPath    = [pPath    pSegment];
quatPath = [quatPath quatSegment];

tGlobal          = [tGlobal tSegmentGlobal];
tcpSpeedPath     = [tcpSpeedPath tcpSpeedSegment];
arcPositionPath  = [arcPositionPath lGlobalSegment];
segmentIndexPath = [segmentIndexPath segment*ones(1,numel(tSegment))];

rawGeometryPath = rawSegment;
arcGeometryPath = arcSegment;

% K. Console summary -------------------------------------------------
fprintf( ...
    ['Arc segment | L = %.4f m | Raw = %d pts | Arc = %d pts | ' ...
     'Profile = %-11s | T = %.3f s | Peak TCP = %.4f m/s\n'], ...
    segmentLength, size(rawSegment,2), size(arcSegment,2), ...
    profileInfo.type, profileInfo.T, max(tcpSpeedSegment));

currentGlobalTime      = currentGlobalTime + profileInfo.T;
currentGlobalArcLength = currentGlobalArcLength + segmentLength;


%% ========================================================================
% 9. TRAJECTORY INFORMATION
% =========================================================================

numSamples      = numel(tGlobal);
totalPathLength = currentGlobalArcLength;

fprintf('------------------------------------------------------------\n');
fprintf('Total trajectory samples: %d\n',numSamples);
fprintf('Total path length:        %.4f m\n',totalPathLength);
fprintf('Total duration:           %.3f s\n',tGlobal(end));
fprintf('============================================================\n');


%% ========================================================================
% 10. ARC GEOMETRY VISUALIZATION
% =========================================================================

figure;
plot3(rawGeometryPath(1,:),rawGeometryPath(2,:),rawGeometryPath(3,:), ...
    '--','LineWidth',1.0);
hold on;
plot3(arcGeometryPath(1,:),arcGeometryPath(2,:),arcGeometryPath(3,:), ...
    '.','MarkerSize',10);

pts = [P_start P_mid P_end];
plot3(pts(1,:),pts(2,:),pts(3,:),'o','MarkerSize',10,'LineWidth',1.5);
text(pts(1,1),pts(1,2),pts(1,3),'  start');
text(pts(1,2),pts(1,3)*0 + pts(2,2),pts(3,3),'  mid');   % just marker labels
text(pts(1,3),pts(2,3),pts(3,3),'  end');

% Plot center as a small cross
plot3(arcInfo.center(1),arcInfo.center(2),arcInfo.center(3), ...
    'kx','MarkerSize',12,'LineWidth',2);

grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Raw arc samples','Arc-length-resampled','User points', ...
    'Circle center','Location','best');
title('Circular Arc Geometry');


%% ========================================================================
% 11. TCP SPEED
% =========================================================================

figure;
plot(tGlobal,tcpSpeedPath,'LineWidth',1.5); hold on;
yline(desiredTCPSpeed,'--','Desired TCP cruise speed');
grid on;
xlabel('Time [s]'); ylabel('TCP Speed [m/s]');
title('TCP Speed Along Arc');


%% ========================================================================
% 12. CUMULATIVE ARC LENGTH
% =========================================================================

figure;
plot(tGlobal,arcPositionPath,'LineWidth',1.5);
grid on;
xlabel('Time [s]'); ylabel('Cumulative Arc Length [m]');
title('Physical Path Progress l(t)');


%% ========================================================================
% 13. SEQUENTIAL ADLS IK
% =========================================================================

qPath        = zeros(n,numSamples);
ikConverged  = false(1,numSamples);
ikIterations = zeros(1,numSamples);

qSeed = qStart;

fprintf('\n');
fprintf('============================================================\n');
fprintf(' SEQUENTIAL ADLS IK\n');
fprintf('============================================================\n');

for k = 1:numSamples

    [qSolution,info] = ADLS_IK(robot, TPath(:,:,k), qSeed);

    ikConverged(k)  = info.converged;
    ikIterations(k) = info.iterations;

    if ~info.converged
        fprintf('\nIK FAILURE\n----------\n');
        fprintf('Sample: %d / %d\n',k,numSamples);
        fprintf('Position error:    %.3e m\n',info.positionError);
        fprintf('Orientation error: %.3e rad\n',info.orientationError);
        error('Arc trajectory failed during sequential ADLS IK.');
    end

    qPath(:,k) = qSolution;
    qSeed      = qSolution;

end

fprintf('Successful IK samples: %d / %d\n',sum(ikConverged),numSamples);
fprintf('============================================================\n');


%% ========================================================================
% 14. INDEPENDENT FK VALIDATION
% =========================================================================

positionError    = zeros(1,numSamples);
orientationError = zeros(1,numSamples);
pAchieved        = zeros(3,numSamples);

for k = 1:numSamples
    TAchieved = controlFK(robot, qPath(:,k));
    pAchieved(:,k) = TAchieved(1:3,4);

    positionError(k) = norm(TPath(1:3,4,k) - TAchieved(1:3,4));

    orientationError(k) = rotationError( ...
        TPath(1:3,1:3,k), TAchieved(1:3,1:3));
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
% 15-18. JOINT VELOCITY / LIMITS (unchanged from straight-line script)
% =========================================================================

qDot  = zeros(size(qPath));
qDDot = zeros(size(qPath));

for joint = 1:n
    qDot(joint,:)  = gradient(qPath(joint,:), tGlobal);
    qDDot(joint,:) = gradient(qDot(joint,:), tGlobal);
end

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

fprintf('\n');
fprintf('============================================================\n');
fprintf(' JOINT SUMMARY\n');
fprintf('============================================================\n');
for joint = 1:n
    fprintf(['J%d | peak vel %.4f rad/s | position violation %d | ' ...
             'velocity violation %d\n'], ...
        joint, peakJointVelocity(joint), ...
        positionViolation(joint), velocityViolation(joint));
end
fprintf('============================================================\n');


%% ========================================================================
% 19. DESIRED VS ACHIEVED
% =========================================================================

figure;
plot3(pPath(1,:),pPath(2,:),pPath(3,:),'LineWidth',1.5); hold on;
plot3(pAchieved(1,:),pAchieved(2,:),pAchieved(3,:),'--','LineWidth',1.5);
plot3(pts(1,:),pts(2,:),pts(3,:),'o','MarkerSize',8);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Desired','IK + FK achieved','User points','Location','best');
title('Desired vs Achieved Arc');


%% ========================================================================
% 20. JOINT POSITION
% =========================================================================

figure;
plot(tGlobal, rad2deg(qPath.'),'LineWidth',1.2);
grid on;
xlabel('Time [s]'); ylabel('Joint angle [deg]');
legend('J1','J2','J3','J4','J5','J6','Location','best');
title('Joint Motion Along Arc');


%% ========================================================================
% 21. JOINT VELOCITY
% =========================================================================

figure;
plot(tGlobal, qDot.','LineWidth',1.2);
grid on;
xlabel('Time [s]'); ylabel('Joint velocity [rad/s]');
legend('J1','J2','J3','J4','J5','J6','Location','best');
title('Joint Velocity Along Arc');


%% ========================================================================
% 22. IK ITERATIONS
% =========================================================================

figure;
plot(tGlobal, ikIterations,'LineWidth',1.2);
grid on;
xlabel('Time [s]'); ylabel('ADLS iterations');
title('ADLS Difficulty Along Arc');


%% ========================================================================
% 23. ANIMATION
% =========================================================================

figure;
ax = axes;

show(visualRobot, qPath(:,1), ...
    'Parent',ax,'PreservePlot',false, ...
    'Visuals','on','Collisions','off');

hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');

show(station.mountingPlane.object,'Parent',ax);
show(station.table.object,'Parent',ax);

plot3(ax, pPath(1,:),pPath(2,:),pPath(3,:),'LineWidth',2);
plot3(ax, pts(1,:),pts(2,:),pts(3,:),'o','MarkerSize',8);

xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');

playbackScale = 1;
playTrajectory(visualRobot, qPath, tGlobal, ax, playbackScale);

animationFigure = ancestor(ax,'figure');
uicontrol(animationFigure, ...
    'Style','pushbutton', ...
    'String','Replay Motion', ...
    'Units','normalized', ...
    'Position',[0.82 0.02 0.14 0.06], ...
    'FontSize',11, ...
    'Callback', @(~,~) replayTrajectory( ...
        visualRobot, qPath, tGlobal, playbackScale));
%% ========================================================================
% LOCAL FUNCTION — SAFE REPLAY CALLBACK
% =========================================================================
% Looks up the axes dynamically so a stale captured handle can never be
% used. Bails out silently if the figure or axes has been destroyed.

function replayTrajectory(visualRobot, qPath, tGlobal, playbackScale)

    fig = gcbf;
    if isempty(fig) || ~isvalid(fig)
        return;
    end

    ax = findobj(fig, 'Type', 'axes');
    if isempty(ax) || ~isvalid(ax(1))
        return;
    end

    playTrajectory(visualRobot, qPath, tGlobal, ax(1), playbackScale);

end


%% ========================================================================
% 24. FINAL RESULT
% =========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' CIRCULAR-ARC PIPELINE RESULT\n');
fprintf('============================================================\n');
fprintf('Total path length:       %.4f m\n',totalPathLength);
fprintf('IK samples passed:       %d / %d\n',sum(ikConverged),numSamples);
fprintf('Max position error:      %.3e m\n',max(positionError));
fprintf('Max orientation error:   %.3e rad\n',max(orientationError));
fprintf('Joint position limits:   %s\n',passFail(~any(positionViolation)));
fprintf('Joint velocity limits:   %s\n',passFail(~any(velocityViolation)));
fprintf('============================================================\n');


% ########################################################################
% #  ALL LOCAL FUNCTIONS BELOW — no executable code after this line.
% ########################################################################


%% ========================================================================
% LOCAL FUNCTION — ORIENTATION ALONG THE ARC
% =========================================================================
%
% q(s) = slerp(I, q_leftover, s) * ( q_path(s) * q_start )
%
%   q_path(s)  = rotation about `axis` by s * thetaTotal
%   q_pred_end = q_path(1) * q_start
%   q_leftover = q_end * q_pred_end^-1      (inverse = conjugate for unit q)
%
% At s=0: returns q_start.   At s=1: returns q_end.

function q = orientationFromArc(q_start, q_end, axis, thetaTotal, s)

axis = axis(:) / norm(axis);

q_start = quatNormalize_custom(q_start);
q_end   = quatNormalize_custom(q_end);

% --- Path-induced rotation at s ---
halfS = 0.5 * s * thetaTotal;
q_path_s = quatNormalize_custom([cos(halfS); sin(halfS)*axis]);

% --- Predicted orientation from path alone at s ---
q_pred_s = quatMultiply_custom(q_path_s, q_start);

% --- Full path-induced prediction at s = 1 ---
halfFull  = 0.5 * thetaTotal;
q_path_f  = quatNormalize_custom([cos(halfFull); sin(halfFull)*axis]);
q_pred_end = quatMultiply_custom(q_path_f, q_start);

% --- Leftover twist: q_end = q_leftover * q_pred_end ---
q_pred_end_inv = quatConjugate_custom(q_pred_end);
q_leftover = quatNormalize_custom( ...
    quatMultiply_custom(q_end, q_pred_end_inv));

% Shortest-path convention
if q_leftover(1) < 0
    q_leftover = -q_leftover;
end

% --- Blend leftover with SLERP over s ---
q_identity   = [1;0;0;0];
q_leftover_s = quatSlerp_custom(q_identity, q_leftover, s);

% --- Final orientation ---
q = quatNormalize_custom( quatMultiply_custom(q_leftover_s, q_pred_s) );

end


%% ========================================================================
% LOCAL FUNCTION — QUATERNION MULTIPLICATION (Hamilton product)
% =========================================================================

function q = quatMultiply_custom(a, b)
wa = a(1); xa = a(2); ya = a(3); za = a(4);
wb = b(1); xb = b(2); yb = b(3); zb = b(4);

q = [ wa*wb - xa*xb - ya*yb - za*zb;
      wa*xb + xa*wb + ya*zb - za*yb;
      wa*yb - xa*zb + ya*wb + za*xb;
      wa*zb + xa*yb - ya*xb + za*wb ];
end


%% ========================================================================
% LOCAL FUNCTION — QUATERNION CONJUGATE
% =========================================================================

function qc = quatConjugate_custom(q)
qc = [q(1); -q(2); -q(3); -q(4)];
end


%% ========================================================================
% LOCAL FUNCTION — ELEMENTAL ROTATIONS
% =========================================================================

function R = rotx_local(a)
R = [1  0       0;
     0  cos(a) -sin(a);
     0  sin(a)  cos(a)];
end

function R = roty_local(a)
R = [ cos(a) 0 sin(a);
      0      1 0;
     -sin(a) 0 cos(a)];
end

function R = rotz_local(a)
R = [cos(a) -sin(a) 0;
     sin(a)  cos(a) 0;
     0       0      1];
end

function R = eulZYX_local(rz, ry, rx)
R = rotz_local(rz) * roty_local(ry) * rotx_local(rx);
end

function e = rotm2eulZYX_local(R)
sy = -R(3,1);
cy = sqrt(R(1,1)^2 + R(2,1)^2);
if cy > 1e-9
    rz = atan2(R(2,1), R(1,1));
    ry = atan2(sy,    cy     );
    rx = atan2(R(3,2), R(3,3));
else
    rz = 0;
    ry = atan2(sy, cy);
    rx = atan2(-R(2,3), R(2,2));
end
e = [rz; ry; rx];
end


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
    show(robotModel, qPath(:,i), ...
        'Parent',ax,'FastUpdate',true,'PreservePlot',false, ...
        'Visuals','on','Collisions','off');

    title(ax, sprintf('UR5 Circular Arc | t = %.2f / %.2f s', ...
        t(i), t(end)));
    drawnow;

    if i < numSamples
        pause(playbackScale * (t(i+1)-t(i)));
    end
end
end


%% ========================================================================
% LOCAL FUNCTION — ROTATION MATRIX -> QUATERNION
% =========================================================================

function q = rotm2quat_custom(R)
tr = R(1,1) + R(2,2) + R(3,3);
if tr > 0
    S = sqrt(tr + 1.0) * 2;
    w = 0.25 * S;
    x = (R(3,2) - R(2,3)) / S;
    y = (R(1,3) - R(3,1)) / S;
    z = (R(2,1) - R(1,2)) / S;
elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
    S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
    w = (R(3,2) - R(2,3)) / S;
    x = 0.25 * S;
    y = (R(1,2) + R(2,1)) / S;
    z = (R(1,3) + R(3,1)) / S;
elseif R(2,2) > R(3,3)
    S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
    w = (R(1,3) - R(3,1)) / S;
    x = (R(1,2) + R(2,1)) / S;
    y = 0.25 * S;
    z = (R(2,3) + R(3,2)) / S;
else
    S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
    w = (R(2,1) - R(1,2)) / S;
    x = (R(1,3) + R(3,1)) / S;
    y = (R(2,3) + R(3,2)) / S;
    z = 0.25 * S;
end
q = quatNormalize_custom([w;x;y;z]);
end


%% ========================================================================
% LOCAL FUNCTION — QUATERNION -> ROTATION MATRIX
% =========================================================================

function R = quat2rotm_custom(q)
q = quatNormalize_custom(q);
w = q(1); x = q(2); y = q(3); z = q(4);
R = [
    1-2*(y^2+z^2),  2*(x*y - z*w),  2*(x*z + y*w);
    2*(x*y + z*w),  1-2*(x^2+z^2),  2*(y*z - x*w);
    2*(x*z - y*w),  2*(y*z + x*w),  1-2*(x^2+y^2)
];
end


%% ========================================================================
% LOCAL FUNCTION — QUATERNION NORMALIZATION
% =========================================================================

function qn = quatNormalize_custom(q)
nrm = sqrt(q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
if nrm < eps
    qn = [1;0;0;0];
else
    qn = q / nrm;
end
end


%% ========================================================================
% LOCAL FUNCTION — QUATERNION SLERP (SHORTEST PATH)
% =========================================================================

function qs = quatSlerp_custom(q0,q1,s)
q0 = quatNormalize_custom(q0);
q1 = quatNormalize_custom(q1);

dotProduct = q0(1)*q1(1) + q0(2)*q1(2) + q0(3)*q1(3) + q0(4)*q1(4);

if dotProduct < 0
    q1 = -q1;
    dotProduct = -dotProduct;
end

dotProduct = max(-1,min(1,dotProduct));

if dotProduct > 0.9995
    qs = quatNormalize_custom(q0 + s * (q1 - q0));
    return;
end

theta0 = acos(dotProduct);
theta  = theta0 * s;

q2 = quatNormalize_custom(q1 - q0 * dotProduct);

qs = q0 * cos(theta) + q2 * sin(theta);
end