function cases = dynamicsTestCases(robot)
% DYNAMICSTESTCASES Preliminary gravity and three-leg load-study motions.

% Edit motion settings here; actuator_sizing_analysis needs no arguments.
tcpSpeed = 0.18;          % m/s
tcpAcceleration = 0.30;   % m/s^2
tcpJerk = 0.80;           % m/s^3
timeStep = 0.05;          % s
spatialStep = 0.04;       % m between Cartesian IK knots

if robot.structure.dof ~= 6
    error('robot_ver2:DynamicsCaseDOF','The ver2 test cases require six joints.');
end
qStart = deg2rad([0 60 105 135 0 0]);
limits = robot.params.joints.positionLimits;
if any(qStart < limits(:,1).' | qStart > limits(:,2).')
    error('robot_ver2:InvalidDynamicsStart', ...
        'The three-leg test start is outside the current joint limits.');
end
T = forward_kinematics(robot,qStart);
p = T(1:3,4).';
qWrist = qStart;
qWrist(4) = qWrist(4)+deg2rad(8);
TWrist = forward_kinematics(robot,qWrist);
R0 = T(1:3,1:3);
R1 = TWrist(1:3,1:3);

motion = struct('profile',"scurve",'tcpSpeed',tcpSpeed, ...
    'tcpAcceleration',tcpAcceleration,'tcpJerk',tcpJerk, ...
    'timeStep',timeStep,'spatialStep',spatialStep);
checks = struct('maxPositionError_m',2e-3, ...
    'maxOrientationError_rad',5e-3, ...
    'maxJointStepRad',deg2rad(15), ...
    'maxSpatialJointStepRad',deg2rad(15), ...
    'requireSmoothStop',true);
base = struct('id',"",'name',"",'description',"",'category',"", ...
    'path',struct(),'startConfiguration',qStart,'motion',motion, ...
    'validation',checks,'enabled',true,'dataStatus',"PRELIMINARY", ...
    'collisionStatus',"NOT_CHECKED",'source',"robot_ver2 test catalog", ...
    'disabledReason',"",'notes',strings(0,1));
cases = repmat(base,3,1);

cases(1).id = "GRAVITY_HOLD";
cases(1).name = "Stationary gravity search";
cases(1).description = ...
    "Searches stationary holding-torque peaks separately for each joint.";
cases(1).category = "gravity";
cases(1).notes = ...
    "Gravity search does not reject self-colliding configurations.";

cases(2).id = "THREE_LEG_SWEEP";
cases(2).name = "Outer-reach horizontal-vertical-reverse sweep";
cases(2).description = ...
    "Horizontal, downward, then opposite horizontal TCP motion with full stops.";
cases(2).category = "preliminary_dynamics";
cases(2).path = struct('type',"multiLine",'waypoints',[ ...
    p; p+[0 0.30 0]; p+[0 0.30 -0.35]; p+[0 -0.20 -0.35]], ...
    'orientations',cat(3,R0,R1,R1,R0));
cases(2).source = "Robot-specific outer-reach, downward-torch start pose";
cases(2).notes = [ ...
    "J3 starts 15 degrees lower than the first downward-pose trial." ...
    "The torch starts downward; J4 deliberately tilts it 8 degrees and back." ...
    "Each path leg stops smoothly at its junction." ...
    "Collision interpolation, physical mesh accuracy, and motor suitability remain unverified."];

% One continuous 180-degree J2 sweep; the other joints move through J2 = 0.
jointProgress = [0 0.16 0.33 0.50 0.67 0.84 1];
jointAnglesDeg = [ ...
     0 -90  70 110   0    0;
    50 -60 130 150 -45  100;
   -40 -30  50  80  45 -100;
     0   0 110 120   0    0;
    40  30 145 155 -45  100;
   -50  60  60 110  45 -100;
     0  90  70 110   0    0];
cases(3).id = "JOINT_SWEEP_180";
cases(3).name = "180-degree shoulder and multi-joint sweep";
cases(3).description = ...
    "J2 crosses zero while the other five joints continue sweeping.";
cases(3).category = "visualization";
cases(3).startConfiguration = deg2rad(jointAnglesDeg(1,:));
cases(3).path = struct('type',"jointSpline", ...
    'progress',jointProgress,'configurations',deg2rad(jointAnglesDeg));
cases(3).motion = struct('profile',"quintic",'duration',12,'timeStep',timeStep);
cases(3).validation.requireCollisionFree = true;
cases(3).validation.minimumClearance_m = 0.010;
cases(3).collisionStatus = "REQUIRED_AT_RUNTIME";
cases(3).dataStatus = "EXPLORATORY";
cases(3).source = "Robot-specific joint-space motion showcase";
cases(3).notes = [ ...
    "J2 moves from -90 to +90 degrees, not the colliding +/-180-degree range." ...
    "The other joints continue moving as J2 passes through zero." ...
    "Sampled mesh clearance is required; swept-volume clearance is unverified." ...
    "This unusual motion is exploratory, not a welding duty cycle or final motor rating."];
end
