function [trajectory,report] = generateTrajectory(robot,testCase)
% GENERATETRAJECTORY Produce validated N-by-DOF Cartesian or joint-space motion.
% A failed path returns report.pass=false and is never suitable for dynamics.
trajectory = struct('time',[],'q',[],'qd',[],'qdd',[]);
report = struct('pass',false,'reason',"",'failedSegment',NaN, ...
    'failedSample',NaN,'failedTime_s',NaN,'failedPose',[], ...
    'failedConfiguration',[],'ikReports',[],'validation',[],'collision',[]);
if ~isfield(testCase,'startConfiguration') || ...
        ~isfield(testCase,'path') || ~isfield(testCase,'motion')
    error('trajectory:InvalidCase','Case needs startConfiguration, path, and motion.');
end
qStart=reshape(testCase.startConfiguration,1,[]);
limits=robot.params.joints.positionLimits;
if numel(qStart)~=robot.structure.dof || any(~isfinite(qStart)) || ...
        any(qStart<limits(:,1).' | qStart>limits(:,2).')
    report.reason="Start configuration violates joint limits."; return;
end
if isfield(testCase,'validation')
    checks=testCase.validation;
else
    checks=struct();
end
if ~isfield(checks,'maxPositionError_m'), checks.maxPositionError_m=2e-3; end
if ~isfield(checks,'maxOrientationError_rad'), checks.maxOrientationError_rad=5e-3; end
if ~isfield(checks,'maxJointStepRad'), checks.maxJointStepRad=deg2rad(15); end
if ~isfield(checks,'maxSpatialJointStepRad'), checks.maxSpatialJointStepRad=deg2rad(15); end
if ~isfield(checks,'requireSmoothStop'), checks.requireSmoothStop=true; end
if lower(string(testCase.path.type))=="jointspline"
    [trajectory,report]=generateJointSplineTrajectory( ...
        robot,testCase,qStart,checks,report);
    return;
end
TStart=forward_kinematics(robot,qStart);
path=testCase.path;
if isfield(path,'startPosition') && ...
        norm(path.startPosition(:)-TStart(1:3,4))>1e-6
    report.reason="Path start position differs from FK of start configuration."; return;
end
if isfield(path,'startOrientation') && ...
        norm(path.startOrientation-TStart(1:3,1:3),'fro')>1e-6
    report.reason="Path start orientation differs from FK of start configuration."; return;
end
path.startPosition=TStart(1:3,4);
path.startOrientation=TStart(1:3,1:3);
if lower(string(path.type))=="multiline"
    waypoints=path.waypoints;
    if size(waypoints,2)~=3 || size(waypoints,1)<2 || ...
            norm(waypoints(1,:).'-path.startPosition)>1e-6
        report.reason="multiLine waypoints must be N-by-3 and start at FK(qStart).";
        return;
    end
    count=size(waypoints,1)-1;
    segments=repmat(struct(),count,1);
    hasOrientations=isfield(path,'orientations') && ~isempty(path.orientations);
    if hasOrientations
        orientations=path.orientations;
        if ~isequal(size(orientations),[3 3 size(waypoints,1)]) || ...
                norm(orientations(:,:,1)-path.startOrientation,'fro')>1e-6
            report.reason="multiLine orientations must be 3-by-3-by-N and start at FK(qStart).";
            return;
        end
    end
    for i=1:count
        segments(i).type="line";
        segments(i).startPosition=waypoints(i,:).';
        segments(i).endPosition=waypoints(i+1,:).';
        if hasOrientations
            segments(i).startOrientation=orientations(:,:,i);
            segments(i).endOrientation=orientations(:,:,i+1);
            segments(i).orientationMode="slerp";
        else
            segments(i).startOrientation=path.startOrientation;
            segments(i).orientationMode="constant";
        end
    end
else
    segments=path;
    count=1;
end
if ~isfield(testCase.motion,'spatialStep') || ...
        ~isscalar(testCase.motion.spatialStep) || ...
        testCase.motion.spatialStep<=0
    error('trajectory:InvalidSpatialStep','motion.spatialStep must be positive meters.');
end
allTime=[]; allQ=[]; allQd=[]; allQdd=[]; allPos=[];
allSpeed=[]; allArc=[]; allPoses=zeros(4,4,0);
distanceOffset=0;
report.ikReports=cell(count,1);
for segmentIndex=1:count
    geometry=generatePathGeometry(segments(segmentIndex));
    profile=timeScalingProfile(geometry.length,testCase.motion);
    sNodes=linspace(0,1,max(9,ceil(geometry.length/ ...
        testCase.motion.spatialStep)+1)).';
    spatialPoses=evaluatePathGeometry(geometry,sNodes);
    [qNodes,ik]=solveTrajectoryIK(robot,spatialPoses,qStart,checks);
    report.ikReports{segmentIndex}=ik;
    if ~ik.pass || ik.maxJointStep_rad>checks.maxSpatialJointStepRad
        report.failedSegment=segmentIndex;
        report.failedSample=ik.failedSample;
        report.failedPose=ik.failedPose;
        report.failedConfiguration=ik.lastValidConfiguration;
        if ik.pass
            report.reason="Suspicious spatial IK joint step.";
            report.failedSample=ik.maxStepSample;
            report.failedPose=spatialPoses(:,:,ik.maxStepSample);
            report.failedConfiguration=qNodes(ik.maxStepSample-1,:);
        else
            report.reason=ik.reason;
        end
        [~,nearest]=min(abs(profile.s-sNodes(report.failedSample)));
        report.failedTime_s=profile.time(nearest)+timeOffset(allTime);
        return;
    end
    [q,qd,qdd]=computeJointDerivatives(sNodes,qNodes, ...
        profile.s,profile.sd,profile.sdd);
    [poses,positions]=evaluatePathGeometry(geometry,profile.s);
    keep=1:numel(profile.time);
    if segmentIndex>1, keep=2:numel(profile.time); end
    allTime=[allTime;profile.time(keep)+timeOffset(allTime)]; %#ok<AGROW>
    allQ=[allQ;q(keep,:)]; %#ok<AGROW>
    allQd=[allQd;qd(keep,:)]; %#ok<AGROW>
    allQdd=[allQdd;qdd(keep,:)]; %#ok<AGROW>
    allPos=[allPos;positions(keep,:)]; %#ok<AGROW>
    allSpeed=[allSpeed;profile.tcpSpeed(keep)]; %#ok<AGROW>
    allArc=[allArc;distanceOffset+geometry.length*profile.s(keep)]; %#ok<AGROW>
    allPoses=cat(3,allPoses,poses(:,:,keep));
    distanceOffset=distanceOffset+geometry.length;
    qStart=qNodes(end,:);
end
trajectory.time=allTime;
trajectory.q=allQ;
trajectory.qd=allQd;
trajectory.qdd=allQdd;
trajectory.cartesian.pose=allPoses;
trajectory.cartesian.position=allPos;
trajectory.cartesian.tcpSpeed=allSpeed;
trajectory.cartesian.arcLength=allArc;
trajectory.meta.type=string(testCase.path.type);
trajectory.meta.name=string(testCase.name);
trajectory.meta.motionProfile=string(testCase.motion.profile);
trajectory.meta.tcpSpeedCommand=testCase.motion.tcpSpeed;
trajectory.meta.tcpAccelerationCommand=testCase.motion.tcpAcceleration;
trajectory.meta.timeStep=testCase.motion.timeStep;
trajectory.meta.pathLength=distanceOffset;
trajectory.meta.accelerationContinuous=profile.accelerationContinuous;
if isfield(testCase.motion,'tcpJerk')
    trajectory.meta.tcpJerkCommand=testCase.motion.tcpJerk;
end
report.validation=validateTrajectory(robot,trajectory,checks);
report.pass=report.validation.pass;
report.reason=report.validation.reason;
report.failedSample=report.validation.failedSample;
if isfinite(report.failedSample)
    report.failedTime_s=trajectory.time(report.failedSample);
    report.failedPose=trajectory.cartesian.pose(:,:,report.failedSample);
    report.failedConfiguration=trajectory.q(report.failedSample,:);
end
if ~profile.accelerationContinuous
    report.pass=false;
    report.reason="Trapezoidal acceleration has ideal jumps; excluded from sizing.";
end
trajectory.validation=report;
end

function offset=timeOffset(time)
if isempty(time), offset=0; else, offset=time(end); end
end

function [trajectory,report]=generateJointSplineTrajectory( ...
    robot,testCase,qStart,checks,report)
trajectory=struct('time',[],'q',[],'qd',[],'qdd',[]);
path=testCase.path;
motion=testCase.motion;
knots=path.progress(:);
qKnots=path.configurations;
dof=robot.structure.dof;
limits=robot.params.joints.positionLimits;
if numel(knots)<4 || ~isequal(size(qKnots),[numel(knots) dof]) || ...
        any(~isfinite([knots;qKnots(:)])) || ...
        abs(knots(1))>1e-12 || abs(knots(end)-1)>1e-12 || ...
        any(diff(knots)<=0) || norm(qKnots(1,:)-qStart,Inf)>1e-10
    report.reason="Joint spline needs increasing 0-to-1 progress and matching N-by-DOF configurations.";
    return;
end
if ~isfield(motion,'duration') || ~isfield(motion,'timeStep') || ...
        ~isscalar(motion.duration) || ~isscalar(motion.timeStep) || ...
        any(~isfinite([motion.duration motion.timeStep])) || ...
        motion.duration<=0 || motion.timeStep<=0 || ...
        lower(string(motion.profile))~="quintic"
    report.reason="Joint spline motion needs positive duration/timeStep and quintic timing.";
    return;
end
qDense=spline(knots,qKnots.',linspace(0,1,1001)).';
if any(qDense<limits(:,1).'-1e-10 | qDense>limits(:,2).'+1e-10,'all')
    report.reason="Joint spline exceeds a position limit between waypoints.";
    return;
end
t=(0:motion.timeStep:motion.duration).';
if motion.duration-t(end)>1e-12
    t(end+1)=motion.duration;
else
    t(end)=motion.duration;
end
u=t/motion.duration;
% Quintic progress gives zero speed and acceleration at both endpoints.
s=10*u.^3-15*u.^4+6*u.^5;
sd=(30*u.^2-60*u.^3+30*u.^4)/motion.duration;
sdd=(60*u-180*u.^2+120*u.^3)/motion.duration^2;
[q,qd,qdd]=computeJointDerivatives(knots,qKnots,s,sd,sdd);
trajectory.time=t;
trajectory.q=q;
trajectory.qd=qd;
trajectory.qdd=qdd;
trajectory.cartesian.pose=zeros(4,4,numel(t));
trajectory.cartesian.position=zeros(numel(t),3);
for i=1:numel(t)
    T=forward_kinematics(robot,q(i,:));
    trajectory.cartesian.pose(:,:,i)=T;
    trajectory.cartesian.position(i,:)=T(1:3,4).';
end
trajectory.meta.type="jointSpline";
trajectory.meta.name=string(testCase.name);
trajectory.meta.motionProfile="quintic";
trajectory.meta.duration=motion.duration;
trajectory.meta.timeStep=motion.timeStep;
report.validation=validateTrajectory(robot,trajectory,checks);
report.pass=report.validation.pass;
report.reason=report.validation.reason;
report.failedSample=report.validation.failedSample;
if report.pass && isfield(checks,'requireCollisionFree') && ...
        checks.requireCollisionFree
    report.collision=analyzeCollisions(robot,q);
    failed=find(report.collision.isSelfColliding,1);
    if isempty(failed) && isfield(checks,'minimumClearance_m')
        failed=find(report.collision.minimumClearance < ...
            checks.minimumClearance_m,1);
    end
    if ~isempty(failed)
        report.pass=false;
        report.failedSample=failed;
        report.reason="Joint spline has a collision or insufficient mesh clearance.";
    end
end
if isfinite(report.failedSample)
    report.failedTime_s=t(report.failedSample);
    report.failedPose=trajectory.cartesian.pose(:,:,report.failedSample);
    report.failedConfiguration=q(report.failedSample,:);
end
trajectory.validation=report;
end
