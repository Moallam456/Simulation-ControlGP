function [trajectory,report] = generateTrajectory(robot,testCase)
% GENERATETRAJECTORY Produce validated N-by-DOF states from Cartesian data.
% A failed path returns report.pass=false and is never suitable for dynamics.
trajectory = struct('time',[],'q',[],'qd',[],'qdd',[]);
report = struct('pass',false,'reason',"",'failedSegment',NaN, ...
    'failedSample',NaN,'failedTime_s',NaN,'failedPose',[], ...
    'failedConfiguration',[],'ikReports',[],'validation',[]);
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
    for i=1:count
        segments(i).type="line";
        segments(i).startPosition=waypoints(i,:).';
        segments(i).endPosition=waypoints(i+1,:).';
        segments(i).startOrientation=path.startOrientation;
        segments(i).orientationMode="constant";
    end
else
    segments=path;
    count=1;
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
