function result = collisionFreeWorkspace( ...
    collisionRobot, ...
    robot, ...
    numSamples, ...
    randomSeed)
%COLLISIONFREEWORKSPACE Generate preliminary collision-free TCP workspace.
%
% result = analysis.collisionFreeWorkspace( ...
%     collisionRobot, robot, numSamples, randomSeed)
%
% This workspace considers:
%
%   - Robot geometry
%   - Joint limits
%   - Self-collision
%   - Preliminary physical collision envelopes
%
% It does NOT yet consider:
%
%   - Welding table
%   - Workpiece
%   - TIG torch body
%   - Required welding orientation
%
% OUTPUTS:
%
% result.q
% result.points
% result.radius3D
% result.horizontalRadius
% result.maxReach
% result.minReach
% result.maxHorizontalReach
% result.minHorizontalReach
% result.xRange
% result.yRange
% result.zRange
% result.numSamples
% result.numCollisionFree
% result.numColliding
% result.collisionFreePercent
% result.collisionPercent

%% ========================================================================
% 1. DEFAULT SETTINGS
% =========================================================================

if nargin < 3
    numSamples = 50000;
end

if nargin < 4
    randomSeed = 1;
end


%% ========================================================================
% 2. RANDOM SEED
% =========================================================================

rng(randomSeed,"twister");


%% ========================================================================
% 3. JOINT LIMITS
% =========================================================================

qMin = robot.limits.qMin;
qMax = robot.limits.qMax;


%% ========================================================================
% 4. GENERATE RANDOM JOINT CONFIGURATIONS
% =========================================================================

Q = zeros(robot.dof,numSamples);

for joint = 1:robot.dof

    Q(joint,:) = ...
        qMin(joint) + ...
        (qMax(joint)-qMin(joint)) .* ...
        rand(1,numSamples);

end


%% ========================================================================
% 5. PREALLOCATE
% =========================================================================

points = NaN(numSamples,3);

collisionMask = false(numSamples,1);


%% ========================================================================
% 6. CHECK EVERY CONFIGURATION
% =========================================================================

for k = 1:numSamples

    q = Q(:,k);


    % ---------------------------------------------------------------------
    % Self-collision check
    % ---------------------------------------------------------------------
    %
    % Parent-child bodies are mechanically connected and naturally touch,
    % therefore those pairs are skipped.

    isColliding = checkCollision( ...
        collisionRobot, ...
        q, ...
        SkippedSelfCollisions="parent");


    collisionMask(k) = isColliding;


    % ---------------------------------------------------------------------
    % Store TCP only if collision free
    % ---------------------------------------------------------------------

    if ~isColliding

        T_B_TCP = getTransform( ...
            collisionRobot, ...
            q, ...
            char(robot.frames.tcp));

        points(k,:) = ...
            T_B_TCP(1:3,4).';

    end

end


%% ========================================================================
% 7. REMOVE COLLIDING CONFIGURATIONS
% =========================================================================

validMask = ~collisionMask;

validPoints = points(validMask,:);

validQ = Q(:,validMask);


%% ========================================================================
% 8. VERIFY THAT SOME VALID CONFIGURATIONS EXIST
% =========================================================================

if isempty(validPoints)

    error("analysis:collisionFreeWorkspace:NoValidConfigurations", ...
        "All sampled configurations were rejected by collision checking.");

end


%% ========================================================================
% 9. CALCULATE WORKSPACE METRICS
% =========================================================================

x = validPoints(:,1);
y = validPoints(:,2);
z = validPoints(:,3);

radius3D = sqrt( ...
    x.^2 + ...
    y.^2 + ...
    z.^2);

horizontalRadius = sqrt( ...
    x.^2 + ...
    y.^2);


%% ========================================================================
% 10. STORE RESULTS
% =========================================================================

result.q = validQ;

result.points = validPoints;

result.radius3D = radius3D;

result.horizontalRadius = horizontalRadius;


result.maxReach = ...
    max(radius3D);

result.minReach = ...
    min(radius3D);


result.maxHorizontalReach = ...
    max(horizontalRadius);

result.minHorizontalReach = ...
    min(horizontalRadius);


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


%% ========================================================================
% 11. COLLISION STATISTICS
% =========================================================================

result.numSamples = numSamples;

result.numCollisionFree = ...
    sum(validMask);

result.numColliding = ...
    sum(collisionMask);

result.collisionFreePercent = ...
    100 * result.numCollisionFree / numSamples;

result.collisionPercent = ...
    100 * result.numColliding / numSamples;

result.collisionMask = collisionMask;

result.randomSeed = randomSeed;

end