function report = validateTrajectory(robot,trajectory,options)
% VALIDATETRAJECTORY Check SI states, limits, smoothness, and actual FK poses.
if nargin < 3, options = struct(); end
report.pass = false;
report.reason = "";
report.failedSample = NaN;
report.maxPositionError_m = 0;
report.maxOrientationError_rad = 0;
report.maxJointStep_rad = 0;
report.maxStepJoint = NaN;
report.maxStepSample = NaN;
report.positionLimits = "UNKNOWN";
report.velocityLimits = "UNKNOWN";
report.accelerationLimits = "UNKNOWN";
required = {'time','q','qd','qdd'};
if ~all(isfield(trajectory,required))
    report.reason = "Missing trajectory time/q/qd/qdd."; return;
end
t=trajectory.time; q=trajectory.q; qd=trajectory.qd; qdd=trajectory.qdd;
[n,dof]=size(q);
if n<2 || dof~=robot.structure.dof || ~isequal(size(t),[n 1]) || ...
        ~isequal(size(qd),[n dof]) || ~isequal(size(qdd),[n dof]) || ...
        any(~isfinite([t(:);q(:);qd(:);qdd(:)])) || any(diff(t)<=0)
    report.reason = "Trajectory arrays have invalid shape, timing, or nonfinite values.";
    return;
end
limits=robot.params.joints.positionLimits;
violation=find(any(q<limits(:,1).'-1e-10 | q>limits(:,2).'+1e-10,2),1);
if ~isempty(violation)
    report.reason = "Position limit violation.";
    report.failedSample = violation;
    report.positionLimits = "FAIL"; return;
end
report.positionLimits = "PASS";
if isfield(robot.params.joints,'velocityLimits')
    speedLimits=robot.params.joints.velocityLimits(:).';
    report.velocityLimits = "PASS";
    violation=find(any(abs(qd)>speedLimits+1e-10,2),1);
    if ~isempty(violation)
        report.velocityLimits = "FAIL";
        report.reason = "Velocity limit violation.";
        report.failedSample = violation; return;
    end
end
if isfield(robot.params.joints,'accelerationLimits')
    accelLimits=robot.params.joints.accelerationLimits(:).';
    report.accelerationLimits = "PASS";
    violation=find(any(abs(qdd)>accelLimits+1e-10,2),1);
    if ~isempty(violation)
        report.accelerationLimits = "FAIL";
        report.reason = "Acceleration limit violation.";
        report.failedSample = violation; return;
    end
end
[report.maxJointStep_rad,flatIndex]=max(abs(diff(q,1,1)),[],'all','linear');
[stepSample,stepJoint]=ind2sub([n-1 dof],flatIndex);
report.maxStepSample=stepSample+1;
report.maxStepJoint=stepJoint;
if isfield(options,'maxJointStepRad') && ...
        report.maxJointStep_rad > options.maxJointStepRad
    report.reason = "Suspicious joint step; inspect IK branch continuity.";
    report.failedSample = report.maxStepSample; return;
end
if isfield(trajectory,'cartesian') && isfield(trajectory.cartesian,'pose')
    poses=trajectory.cartesian.pose;
    if ~isequal(size(poses),[4 4 n])
        report.reason = "Cartesian pose array must be 4-by-4-by-N."; return;
    end
    for k=1:n
        actual=forward_kinematics(robot,q(k,:));
        pError=norm(actual(1:3,4)-poses(1:3,4,k));
        R=poses(1:3,1:3,k).'*actual(1:3,1:3);
        rError=acos(max(-1,min(1,(trace(R)-1)/2)));
        report.maxPositionError_m=max(report.maxPositionError_m,pError);
        report.maxOrientationError_rad=max(report.maxOrientationError_rad,rError);
        if isfield(options,'maxPositionError_m') && ...
                pError>options.maxPositionError_m || ...
                isfield(options,'maxOrientationError_rad') && ...
                rError>options.maxOrientationError_rad
            report.reason = "Timed FK pose error exceeded tolerance.";
            report.failedSample = k; return;
        end
    end
end
report.boundarySpeed_rad_s = max(abs([qd(1,:) qd(end,:)]));
report.boundaryAcceleration_rad_s2 = max(abs([qdd(1,:) qdd(end,:)]));
if isfield(options,'requireSmoothStop') && options.requireSmoothStop && ...
        (report.boundarySpeed_rad_s>1e-8 || ...
        report.boundaryAcceleration_rad_s2>1e-6)
    report.reason = "Start/end joint derivatives are not smooth."; return;
end
report.pass = true;
end
