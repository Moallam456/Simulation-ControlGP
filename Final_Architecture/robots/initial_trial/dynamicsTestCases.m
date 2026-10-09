function cases = dynamicsTestCases(robot)
% DYNAMICSTESTCASES Preliminary load-study motions for initial_trial.

% Shared editable motion settings.
tcpSpeed = 0.10;          % m/s
tcpAcceleration = 0.25;  % m/s^2
tcpJerk = 1.00;          % m/s^3
timeStep = 0.05;         % s
spatialStep = 0.005;     % m between Cartesian IK knots

q = deg2rad([0 20 -30 0 20 0]);
limits = robot.params.joints.positionLimits;
if robot.structure.dof ~= 6 || ...
        any(q < limits(:,1).' | q > limits(:,2).')
    error('initial_trial:InvalidDynamicsCases', ...
        'The initial_trial load cases require the expected six-axis model.');
end
T = forward_kinematics(robot,q);
p = T(1:3,4);
motion = struct('profile',"scurve",'tcpSpeed',tcpSpeed, ...
    'tcpAcceleration',tcpAcceleration,'tcpJerk',tcpJerk, ...
    'timeStep',timeStep,'spatialStep',spatialStep);
checks = struct('maxPositionError_m',2e-3, ...
    'maxOrientationError_rad',5e-3,'maxJointStepRad',deg2rad(15), ...
    'maxSpatialJointStepRad',deg2rad(15),'requireSmoothStop',true);
base = struct('id',"",'name',"",'description',"",'category',"", ...
    'path',struct(),'startConfiguration',q,'motion',motion, ...
    'validation',checks,'enabled',true,'dataStatus',"PRELIMINARY", ...
    'collisionStatus',"NOT_CHECKED",'source',"initial_trial test catalog", ...
    'disabledReason',"",'notes',strings(0,1));
cases = repmat(base,8,1);

cases(1).id = "GRAVITY_HOLD";
cases(1).name = "Stationary gravity search";
cases(1).category = "gravity";
cases(1).description = "Finds separate stationary gravity peaks for every joint.";
cases(1).source = "Robot joint limits and gravityCandidatePoses.m";

cases(2).id = "LINE_WELD";
cases(2).name = "Straight weld";
cases(2).category = "weld";
cases(2).path = struct('type',"line",'endPosition',p+[0.04;0;0], ...
    'orientationMode',"constant");
cases(2).description = "Adapted 40 mm line with constant TCP orientation.";

cases(3).id = "ARC_WELD";
cases(3).name = "Arc weld";
cases(3).category = "weld";
cases(3).path = struct('type',"arc",'midPosition',p+[0.03;0.015;0], ...
    'endPosition',p+[0.06;0;0],'orientationMode',"followArc");
cases(3).description = "Three-point arc with path-following orientation.";

cases(4).id = "FULL_CIRCLE_WELD";
cases(4).name = "Full-circle weld";
cases(4).category = "weld";
cases(4).path = struct('type',"fullcircle",'point2',p+[0.02;0.02;0], ...
    'point3',p+[0.04;0;0],'direction',1,'orientationMode',"constant");
cases(4).description = "40 mm-diameter circle with constant orientation.";

cases(5).id = "MULTI_SEGMENT";
cases(5).name = "Multi-segment weld";
cases(5).category = "weld";
cases(5).path = struct('type',"multiLine",'waypoints', ...
    [p.';(p+[0.025;0;0]).';(p+[0.025;0.025;0]).'; ...
    (p+[0.045;0.025;0]).']);
cases(5).description = "Three lines with a full stop at each junction.";

cases(6).id = "CONTROL_LINE_ORIGINAL";
cases(6).name = "Original Control line";
cases(6).category = "source_reference";
cases(6).startConfiguration = deg2rad([30 -45 60 20 -30 45]);
TOriginal = forward_kinematics(robot,cases(6).startConfiguration);
cases(6).path = struct('type',"line", ...
    'endPosition',TOriginal(1:3,4)+[0.5;0;0], ...
    'orientationMode',"constant");
cases(6).enabled = false;
cases(6).dataStatus = "INFEASIBLE_SOURCE";
cases(6).disabledReason = ...
    "The original 500 mm endpoint exceeds this robot's gross reach.";
cases(6).source = "Control/Trajectory/Pipeline/SingleSegmentTest_line.m";

cases(7).id = "TRANSFER_MOVE";
cases(7).name = "Transfer move";
cases(7).category = "transfer";
cases(7).enabled = false;
cases(7).dataStatus = "AWAITING_REQUIREMENTS";
cases(7).disabledReason = ...
    "Approved geometry, speed, acceleration, and jerk are not supplied.";
cases(7).motion = struct();
cases(7).source = "Future duty cycle";

cases(8) = cases(4);
cases(8).id = "FULL_CIRCLE_FOLLOW_ORIENTATION";
cases(8).name = "Full-circle orientation-following trial";
cases(8).path.orientationMode = "followArc";
cases(8).enabled = false;
cases(8).dataStatus = "FAILED_CONTINUITY_TRIAL";
cases(8).disabledReason = ...
    "No continuous IK branch met the current spatial joint-step limit.";

end
