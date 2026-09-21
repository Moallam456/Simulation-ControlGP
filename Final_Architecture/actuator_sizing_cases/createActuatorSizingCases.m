function cases = createActuatorSizingCases(robot)
% CREATEACTUATORSIZINGCASES Preliminary, traceable load-case definitions.
q = deg2rad([0 20 -30 0 20 0]);
limits = robot.params.joints.positionLimits;
if numel(q) ~= robot.structure.dof || any(q < limits(:,1).' | q > limits(:,2).')
    error('sizing:CaseStartInvalid','Example start pose is invalid for this robot; supply a robot-specific case catalog.');
end
T = forward_kinematics(robot,q);
p = T(1:3,4);
motion = struct('profile','scurve','tcpSpeed',0.10, ...
    'tcpAcceleration',0.25,'tcpJerk',1.0,'timeStep',0.05, ...
    'spatialStep',0.005);
checks = struct('maxPositionError_m',2e-3, ...
    'maxOrientationError_rad',5e-3,'maxJointStepRad',deg2rad(15), ...
    'maxSpatialJointStepRad',deg2rad(15),'requireSmoothStop',true);
base = struct('id',"",'name',"",'description',"",'category',"", ...
    'path',struct(),'startConfiguration',q,'motion',motion, ...
    'validation',checks,'enabled',true,'dataStatus',"PRELIMINARY", ...
    'source',"Control/Trajectory example geometry; adapted for initial_trial", ...
    'disabledReason',"");
cases = repmat(base,8,1);
cases(1).id="GRAVITY_HOLD";
cases(1).name="Gravity hold";
cases(1).category="gravity";
cases(1).description="Joint-limit-domain gravity search, not a global proof.";
cases(1).source="Current dynamics gravity search";

cases(2).id="LINE_WELD";
cases(2).name="Straight weld";
cases(2).category="weld";
cases(2).path=struct('type','line','endPosition',p+[0.04;0;0], ...
    'orientationMode','constant');
cases(2).description="Adapted 40 mm line; original Control line was 500 mm.";

cases(3).id="ARC_WELD";
cases(3).name="Arc weld";
cases(3).category="weld";
cases(3).path=struct('type','arc','midPosition',p+[0.03;0.015;0], ...
    'endPosition',p+[0.06;0;0],'orientationMode','followArc');
cases(3).description="Adapted three-point arc with path-following orientation.";

cases(4).id="FULL_CIRCLE_WELD";
cases(4).name="Full-circle weld";
cases(4).category="weld";
cases(4).path=struct('type','fullcircle','point2',p+[0.02;0.02;0], ...
    'point3',p+[0.04;0;0],'direction',1,'orientationMode','constant');
cases(4).description="Adapted full circle with constant closed orientation.";

cases(5).id="MULTI_SEGMENT";
cases(5).name="Multi-segment weld";
cases(5).category="weld";
cases(5).path=struct('type','multiLine','waypoints', ...
    [p.';(p+[0.025;0;0]).';(p+[0.025;0.025;0]).'; ...
    (p+[0.045;0.025;0]).']);
cases(5).description="Three straight segments, full stop at each junction; no blending.";

cases(6).id="CONTROL_LINE_ORIGINAL";
cases(6).name="Original Control line";
cases(6).category="source_reference";
cases(6).startConfiguration=deg2rad([30 -45 60 20 -30 45]);
TOriginal=forward_kinematics(robot,cases(6).startConfiguration);
cases(6).path=struct('type','line','endPosition',TOriginal(1:3,4)+[0.5;0;0], ...
    'orientationMode','constant');
cases(6).enabled=false;
cases(6).dataStatus="INFEASIBLE_SOURCE";
cases(6).disabledReason="Original +0.5 m X endpoint exceeds current robot's gross kinematic reach; original was for another robot.";
cases(6).source="Control/Trajectory/Pipeline/SingleSegmentTest_line.m";

cases(7).id="TRANSFER_MOVE";
cases(7).name="Transfer move";
cases(7).category="transfer";
cases(7).enabled=false;
cases(7).dataStatus="AWAITING_REQUIREMENTS";
cases(7).disabledReason="Approved transfer geometry, TCP speed, acceleration, and jerk are not yet supplied.";
cases(7).motion=struct();
cases(7).source="Future client duty cycle";

cases(8)=cases(4);
cases(8).id="FULL_CIRCLE_FOLLOW_ORIENTATION";
cases(8).name="Full-circle orientation-following trial";
cases(8).path.orientationMode="followArc";
cases(8).enabled=false;
cases(8).dataStatus="FAILED_CONTINUITY_TRIAL";
cases(8).disabledReason="No continuous branch met the current spatial joint-step limit at sample 2; revisit tool orientation and path spacing before enabling.";
end
