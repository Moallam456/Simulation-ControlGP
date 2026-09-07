function result = workspace(robotModel, robot, numSamples, randomSeed)
%WORKSPACE Sample the mathematical kinematic TCP workspace.
%
% result = analysis.workspace(robotModel, robot, numSamples, randomSeed)
%
% This function calculates the KINEMATIC workspace only.
%
% It considers:
%   - robot geometry
%   - joint limits
%   - TCP definition
%
% It does NOT yet consider:
%   - self collision
%   - base collision
%   - table collision
%   - workpiece collision
%   - welding torch orientation requirements
%
% INPUTS
%   robotModel   MATLAB rigidBodyTree
%   robot        RobotConfig
%   numSamples   number of joint configurations
%   randomSeed   seed for repeatable comparisons
%
% OUTPUT
%   result.q
%   result.points
%   result.radius3D
%   result.horizontalRadius
%   result.maxReach
%   result.minReach
%   result.maxHorizontalReach
%   result.minHorizontalReach
%   result.xRange
%   result.yRange
%   result.zRange
%   result.closestToBase
%   result.closestToBaseAxis

%% Defaults

if nargin < 3
    numSamples = 10000;
end

if nargin < 4
    randomSeed = 1;
end


%% Validate inputs

if numSamples <= 0 || floor(numSamples) ~= numSamples
    error("analysis:workspace:InvalidSampleCount", ...
        "numSamples must be a positive integer.");
end


%% Repeatable random sampling

rng(randomSeed,"twister");


%% Joint limits

qMin = robot.limits.qMin;
qMax = robot.limits.qMax;

if any(~isfinite(qMin)) || any(~isfinite(qMax))
    error("analysis:workspace:InvalidJointLimits", ...
        "All joint limits must be finite.");
end


%% Generate random configurations

Q = zeros(robot.dof,numSamples);

for joint = 1:robot.dof

    Q(joint,:) = ...
        qMin(joint) + ...
        (qMax(joint)-qMin(joint)) .* rand(1,numSamples);

end


%% Calculate TCP positions

points = zeros(numSamples,3);

for k = 1:numSamples

    q = Q(:,k);

    T_B_TCP = getTransform( ...
        robotModel, ...
        q, ...
        char(robot.frames.tcp));

    points(k,:) = T_B_TCP(1:3,4).';

end


%% Cartesian coordinates

x = points(:,1);
y = points(:,2);
z = points(:,3);


%% 3-D distance from base origin

radius3D = sqrt( ...
    x.^2 + ...
    y.^2 + ...
    z.^2 );


%% Horizontal distance from the base rotation axis

horizontalRadius = sqrt( ...
    x.^2 + ...
    y.^2 );


%% Find extreme points

[minReach,indexMinReach] = min(radius3D);
[maxReach,indexMaxReach] = max(radius3D);

[minHorizontalReach,indexMinHorizontal] = ...
    min(horizontalRadius);

[maxHorizontalReach,indexMaxHorizontal] = ...
    max(horizontalRadius);


%% Store results

result.q = Q;
result.points = points;

result.radius3D = radius3D;
result.horizontalRadius = horizontalRadius;

result.maxReach = maxReach;
result.minReach = minReach;

result.maxHorizontalReach = maxHorizontalReach;
result.minHorizontalReach = minHorizontalReach;

result.xRange = [
    min(x)
    max(x)
];

result.yRange = [
    min(y)
    max(y)
];

result.zRange = [
    min(z)
    max(z)
];

result.numSamples = numSamples;
result.randomSeed = randomSeed;


%% Configuration closest to base origin

result.closestToBase.point = ...
    points(indexMinReach,:);

result.closestToBase.q = ...
    Q(:,indexMinReach);

result.closestToBase.distance = ...
    minReach;


%% Configuration closest to base Z-axis

result.closestToBaseAxis.point = ...
    points(indexMinHorizontal,:);

result.closestToBaseAxis.q = ...
    Q(:,indexMinHorizontal);

result.closestToBaseAxis.horizontalDistance = ...
    minHorizontalReach;

end