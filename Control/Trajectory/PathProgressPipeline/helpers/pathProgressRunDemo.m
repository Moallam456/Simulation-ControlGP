function [data,robot] = pathProgressRunDemo(demo)
%PATHPROGRESSRUNDEMO Build references using the unchanged Control planners.
helpers = fileparts(mfilename('fullpath'));
folder = fileparts(helpers);
trajectory = fileparts(folder);
repo = fileparts(fileparts(trajectory));
oldPath = path;
restore = onCleanup(@() path(oldPath));
addpath(helpers,repo,trajectory,fullfile(trajectory,'TimeScaling'), ...
    fullfile(repo,'Control','Kinematics'),fullfile(trajectory,'Pipeline'));

[robot,tcpFK,solveSample] = loadTrajectoryRobot();
switch demo
    case "line", startDegrees = [30;-45;10;20;-30;45];
    case {"arc","arc_exact","arc_exact_length"}, startDegrees = [30;-45;20;20;-30;45];
    case {"fullcircle","fullcircle_exact","fullcircle_exact_length"}, startDegrees = [30;-45;30;20;-30;45];
    case {"multi","multi_scurve"}, startDegrees = [30;-65;-10;20;-30;45];
    otherwise, error('PathProgress:UnknownDemo','Unknown path-progress demo: %s.',demo);
end
data.qStart = deg2rad(startDegrees);
data.TStart = tcpFK(data.qStart);
origin = data.TStart(1:3,4);
rotation = data.TStart(1:3,1:3);
segments = planSegments(demo,origin,rotation);
data.demo = demo;
data.tGlobal = [];
data.TPath = zeros(4,4,0);
data.arcPositionPath = [];
data.tcpSpeedPath = [];
data.segmentIndexPath = [];
data.qPath = zeros(robot.structure.dof,0);
data.qDot = data.qPath;
data.qDDot = data.qPath;
data.qJerk = data.qPath;
data.pAchieved = zeros(3,0);
data.positionError = [];
data.orientationError = [];
data.rawGeometryPath = zeros(3,0);
data.arcGeometryPath = zeros(3,0);
elapsed = 0;
distance = 0;
seed = data.qStart;
for k = 1:numel(segments)
    segment = segments(k);
    fit = pathProgressFitSegment(segment,tcpFK,solveSample, ...
        robot.params.joints.positionLimits,seed,k>1);
    seed = fit.qNodes(:,end);
    poses = pathProgressPose(segment,segment.timing(2,:));
    % Retain the preceding segment's terminal timestamp and one-sided
    % acceleration. The following segment's local start is stored separately.
    keep = (1+(k>1)):size(segment.timing,2);
    fit.globalIndices = numel(data.tGlobal)+(1:numel(keep));
    data.fit(k) = fit;
    data.tGlobal = [data.tGlobal,elapsed+segment.timing(1,keep)];
    data.TPath = cat(3,data.TPath,poses(:,:,keep));
    pathLength = segment.length;
    tcpSpeed = pathLength*segment.timing(3,keep);
    arcGeometry = segment.arcGeometry;
    if isfield(segment,'exactCircle')
        pathLength = segment.exactLength;
        [~,ps] = pathProgressCircleGeometry(segment.exactCircle,segment.timing(2,keep));
        tcpSpeed = vecnorm(ps.*segment.timing(3,keep),2,1);
        arcGeometry = pathProgressCircleGeometry(segment.exactCircle,segment.arcLength/segment.length);
    end
    data.arcPositionPath = [data.arcPositionPath,distance+pathLength*segment.timing(2,keep)];
    data.tcpSpeedPath = [data.tcpSpeedPath,tcpSpeed];
    data.segmentIndexPath = [data.segmentIndexPath,k*ones(1,numel(keep))];
    data.qPath = [data.qPath,fit.q(:,keep)];
    data.qDot = [data.qDot,fit.velocity(:,keep)];
    data.qDDot = [data.qDDot,fit.acceleration(:,keep)];
    data.qJerk = [data.qJerk,fit.jerk(:,keep)];
    data.pAchieved = [data.pAchieved,fit.pAchieved(:,keep)];
    data.positionError = [data.positionError,fit.positionError(keep)];
    data.orientationError = [data.orientationError,fit.orientationError(keep)];
    firstGeometry = 1+(k>1);
    data.rawGeometryPath = [data.rawGeometryPath,segment.rawGeometry(:,firstGeometry:end)];
    data.arcGeometryPath = [data.arcGeometryPath,arcGeometry(:,firstGeometry:end)];
    elapsed = elapsed+segment.profile.T;
    distance = distance+pathLength;
end
assert(all(diff(data.tGlobal)>0) && all(abs(diff(data.qPath,1,2))<=0.10,'all'), ...
    'PathProgress:InvalidReference','Non-increasing timestamps or discontinuous joint references.');
data.segments = segments;
data.pPath = reshape(data.TPath(1:3,4,:),3,[]);
data.quatPath = rotm2quat(data.TPath(1:3,1:3,:)).';
for i = 2:size(data.quatPath,2)
    if dot(data.quatPath(:,i-1),data.quatPath(:,i)) < 0
        data.quatPath(:,i) = -data.quatPath(:,i);
    end
end
data.ikConverged = true(size(data.tGlobal));
data.ikIterations = zeros(size(data.tGlobal));
data.rateLimits = checkRates(robot,data);
if demo == "line"
    pathProgressExportLine(folder,data);
end
end

function segments = planSegments(demo,origin,rotation)
spacing = 0.005;     % m, original Cartesian polyline
speed = 0.10;        % m/s
acceleration = 0.25; % m/s^2
jerk = 1.00;         % m/s^3
dt = 0.05;           % s
endRotation = rotation;
axis = [0 0 1];
sweep = 0;
switch demo
    case "line"
        raw = {generateLineWaypoints(origin,origin+[0.50;0;0],100)};
        endRotation = rotation*eul2rotm(deg2rad([15 15 15]),'ZYX');
    case {"arc","arc_exact","arc_exact_length"}
        [points,arc] = generateCircleWaypoints(origin,origin+[0.20;0.14;0], ...
            origin+[0.40;0;0],200);
        raw = {points};
        axis = arc.axis(:).';
        sweep = arc.thetaTotal;
    case {"fullcircle","fullcircle_exact","fullcircle_exact_length"}
        [points,arc] = generateFullCircleWaypoints(origin,origin+[0.15;0.15;0], ...
            origin+[0.30;0;0],+1,400);
        raw = {points};
    case {"multi","multi_scurve"}
        offsets = [0.50 0 -0.35 0.15 -0.20; 0 0.20 0.10 -0.35 0.10; 0 0.10 0.15 -0.10 -0.20];
        raw = cell(1,size(offsets,2));
        for k = 1:numel(raw)
            endpoint = origin+offsets(:,k);
            raw{k} = generateLineWaypoints(origin,endpoint,100);
            origin = endpoint;
        end
end
for k = 1:numel(raw)
    [geometry,originalLength,arcLength] = arcLengthParameterize(raw{k},spacing);
    length = originalLength(end);
    timingLength = length;
    if demo == "arc_exact_length" || demo == "fullcircle_exact_length"
        timingLength = arc.radius*abs(arc.thetaTotal);
    end
    if demo == "multi"
        [t,s,sd,sdd,sddd,profile] = trapezoidalTimeScaling(speed/length,acceleration/length,dt);
    else
        [t,s,sd,sdd,sddd,profile] = sCurveTimeScaling( ...
            speed/timingLength,acceleration/timingLength,jerk/timingLength,dt);
    end
    segments(k) = struct('rawGeometry',raw{k},'arcGeometry',geometry,'arcLength',arcLength, ...
        'length',length,'startRotation',rotation,'endRotation',endRotation, ...
        'axis',axis,'orientationSweep',sweep,'timing',[t(:).';s(:).';sd(:).';sdd(:).';sddd(:).'], ...
        'profile',profile); %#ok<AGROW>
    if demo == "arc_exact" || demo == "fullcircle_exact" || ...
            demo == "arc_exact_length" || demo == "fullcircle_exact_length"
        % Keep chord metadata for the existing geometric verification grid.
        segments(k).exactCircle = arc;
        segments(k).exactLength = arc.radius*abs(arc.thetaTotal);
    end
end
end

function report = checkRates(robot,data)
rates = {data.qDot,data.qDDot,data.qJerk};
fields = ["velocityLimits","accelerationLimits","jerkLimits"];
report.status = repmat("Unavailable",1,3);
report.peak = zeros(robot.structure.dof,3);
report.velocityURDF = nan(robot.structure.dof,1);
for k = 1:numel(fields)
    values = rates{k};
    report.peak(:,k) = max(abs(values),[],2,'omitnan');
    if isfield(robot.params.joints,fields(k))
        limits = robot.params.joints.(fields(k));
        if any(report.peak(:,k) > limits(:))
            error('PathProgress:RateLimit','A sampled reference exceeds %s.',fields(k));
        end
        report.status(k) = "Within documented limits at samples";
    end
end
report.undefinedJerkSamples = nnz(any(~isfinite(data.qJerk),1));
if report.status(1) == "Unavailable"
    % robot_ver2 labels these URDF values as software placeholders. Compare
    % them for evidence without promoting them to hardware ratings.
    document = xmlread(char(robot.params.source.urdf));
    joints = document.getElementsByTagName('joint');
    for i = 0:joints.getLength-1
        joint = joints.item(i);
        index = find(robot.structure.jointNames == string(char(joint.getAttribute('name'))),1);
        limit = joint.getElementsByTagName('limit').item(0);
        if ~isempty(index) && ~isempty(limit)
            report.velocityURDF(index) = str2double(char(limit.getAttribute('velocity')));
        end
    end
    if all(isfinite(report.velocityURDF) & report.velocityURDF > 0)
        if any(report.peak(:,1) > report.velocityURDF)
            report.status(1) = "Exceeds provisional URDF values at samples";
        else
            report.status(1) = "Within provisional URDF values at samples";
        end
    end
end
end
