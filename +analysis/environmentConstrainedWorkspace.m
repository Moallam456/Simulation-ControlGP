function result = environmentConstrainedWorkspace( ...
    collisionRobot, ...
    robot, ...
    worldObjects, ...
    numSamples, ...
    randomSeed)
%ENVIRONMENTCONSTRAINEDWORKSPACE
% Generate workspace after rejecting both self-collision and world
% collisions.
%
% Considers:
%
%   - joint limits
%   - robot collision geometry
%   - self collision
%   - mounting plane
%   - welding table
%
% Does NOT yet include:
%
%   - workpiece
%   - welding torch body
%   - final TCP
%   - welding orientation constraints

%% ========================================================================
% 1. DEFAULTS
% =========================================================================

if nargin < 4
    numSamples = 50000;
end

if nargin < 5
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
% 4. GENERATE RANDOM CONFIGURATIONS
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

selfCollisionMask = false(numSamples,1);

worldCollisionMask = false(numSamples,1);


%% ========================================================================
% 6. CHECK CONFIGURATIONS
% =========================================================================

for k = 1:numSamples

    q = Q(:,k);


    % ---------------------------------------------------------------------
    % MATLAB returns:
    %
    % collisionStatus(1) = self collision
    % collisionStatus(2) = collision with world objects
    % ---------------------------------------------------------------------

    collisionStatus = checkCollision( ...
        collisionRobot, ...
        q, ...
        worldObjects, ...
        SkippedSelfCollisions="parent");


    selfCollisionMask(k) = ...
        collisionStatus(1);

    worldCollisionMask(k) = ...
        collisionStatus(2);


    % ---------------------------------------------------------------------
    % Accept only if BOTH are false
    % ---------------------------------------------------------------------

    if ~collisionStatus(1) && ...
       ~collisionStatus(2)

        T_B_TCP = getTransform( ...
            collisionRobot, ...
            q, ...
            char(robot.frames.tcp));

        points(k,:) = ...
            T_B_TCP(1:3,4).';

    end

end

%% ========================================================================
% 7. VALID CONFIGURATIONS
% =========================================================================

environmentCollisionMask = ...
    selfCollisionMask | ...
    worldCollisionMask;

validMask = ...
    ~environmentCollisionMask;


validPoints = ...
    points(validMask,:);

validQ = ...
    Q(:,validMask);


if isempty(validPoints)

    error( ...
        "analysis:environmentConstrainedWorkspace:NoValidConfigurations", ...
        "No collision-free configurations remain.");

end


%% ========================================================================
% 8. WORKSPACE METRICS
% =========================================================================

x = validPoints(:,1);
y = validPoints(:,2);
z = validPoints(:,3);

radius3D = ...
    sqrt(x.^2 + y.^2 + z.^2);

horizontalRadius = ...
    sqrt(x.^2 + y.^2);


%% ========================================================================
% 9. STORE WORKSPACE
% =========================================================================

result.q = validQ;

result.points = validPoints;

result.radius3D = radius3D;

result.horizontalRadius = horizontalRadius;

result.maxReach = max(radius3D);

result.minReach = min(radius3D);

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
% 10. COLLISION STATISTICS
% =========================================================================

result.numSamples = numSamples;

result.numValid = ...
    sum(validMask);

result.numSelfColliding = ...
    sum(selfCollisionMask);

result.numWorldColliding = ...
    sum(worldCollisionMask);

result.numRejected = ...
    sum(environmentCollisionMask);


result.validPercent = ...
    100 * result.numValid / numSamples;

result.selfCollisionPercent = ...
    100 * result.numSelfColliding / numSamples;

result.worldCollisionPercent = ...
    100 * result.numWorldColliding / numSamples;

result.rejectedPercent = ...
    100 * result.numRejected / numSamples;


result.selfCollisionMask = ...
    selfCollisionMask;

result.worldCollisionMask = ...
    worldCollisionMask;

result.validMask = ...
    validMask;

result.randomSeed = ...
    randomSeed;

end