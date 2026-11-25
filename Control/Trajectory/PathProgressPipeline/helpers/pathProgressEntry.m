% Shared script so the demo retains the original joint and pose workspace arrays.
[pathProgressData,robot] = pathProgressRunDemo(pathProgressDemo);
qStart = pathProgressData.qStart;
TStart = pathProgressData.TStart;
TPath = pathProgressData.TPath;
pPath = pathProgressData.pPath;
pAchieved = pathProgressData.pAchieved;
quatPath = pathProgressData.quatPath;
tGlobal = pathProgressData.tGlobal;
arcPositionPath = pathProgressData.arcPositionPath;
tcpSpeedPath = pathProgressData.tcpSpeedPath;
segmentIndexPath = pathProgressData.segmentIndexPath;
qPath = pathProgressData.qPath;
qDot = pathProgressData.qDot;
qDDot = pathProgressData.qDDot;
qJerk = pathProgressData.qJerk;
positionError = pathProgressData.positionError;
orientationError = pathProgressData.orientationError;
ikConverged = pathProgressData.ikConverged;
ikIterations = pathProgressData.ikIterations;
rawGeometryPath = pathProgressData.rawGeometryPath;
arcGeometryPath = pathProgressData.arcGeometryPath;
numSamples = numel(tGlobal);
numSegments = numel(pathProgressData.segments);
profileInfo = pathProgressData.segments(end).profile;
pathProgressTiming = pathProgressData.segments(end).timing;
tSegment = pathProgressTiming(1,1+(numSegments>1):end);
sSegment = pathProgressTiming(2,:);
sDotSegment = pathProgressTiming(3,:);
sDDotSegment = pathProgressTiming(4,:);
sDDDotSegment = pathProgressTiming(5,:);
if ~exist('pathProgressRender','var') || pathProgressRender
    animationFigure = pathProgressShowDemo(pathProgressData,robot);
end
