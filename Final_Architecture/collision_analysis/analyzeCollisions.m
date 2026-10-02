function report = analyzeCollisions(robot,qSamples,options)
% ANALYZECOLLISIONS Evaluate self-collision and body-to-body clearance.
%
% report = analyzeCollisions(robot,qSamples)
% report = analyzeCollisions(robot,qSamples,options)
%
% options.minimumClearance is a reporting threshold in metres. It does not
% change geometric collision detection.

if nargin < 3 || isempty(options)
    options = struct();
end
if ~isfield(options,'minimumClearance')
    options.minimumClearance = defaultClearance(robot);
end
if ~isfield(options,'showProgress')
    options.showProgress = false;
end
if ~isstruct(robot) || ~isnumeric(qSamples) || ~isstruct(options)
    error('collision:InvalidInput', ...
        'Expected robot, numeric configurations, and an options structure.');
end
options.showProgress = logical(options.showProgress);

validateInputs(robot,qSamples,options.minimumClearance);

sampleCount = size(qSamples,1);
bodyNames = [string(robot.model.BodyNames) string(robot.model.BaseName)];
bodyCount = numel(bodyNames);
skippedPairs = skippedSelfCollisionPairs(robot);
skippedCells = cellstr(skippedPairs);

isSelfColliding = false(sampleCount,1);
minimumClearance = inf(sampleCount,1);
closestBodyPair = strings(sampleCount,2);
witnessPoints = NaN(sampleCount,6);
pairCounts = zeros(bodyCount,bodyCount);

progressStep = max(1,floor(sampleCount/10));
for sampleIndex = 1:sampleCount
    [isSelfColliding(sampleIndex),distances,witness] = checkCollision( ...
        robot.model,qSamples(sampleIndex,:), ...
        'SkippedSelfCollisions',skippedCells, ...
        'Exhaustive','on');

    upper = triu(true(bodyCount),1);
    collisionPairs = find(upper & isnan(distances));
    if ~isempty(collisionPairs)
        [rows,columns] = ind2sub([bodyCount bodyCount],collisionPairs);
        for pairIndex = 1:numel(rows)
            pairCounts(rows(pairIndex),columns(pairIndex)) = ...
                pairCounts(rows(pairIndex),columns(pairIndex))+1;
        end
        closestBodyPair(sampleIndex,:) = ...
            [bodyNames(rows(1)) bodyNames(columns(1))];
        minimumClearance(sampleIndex) = 0;
    else
        finitePairs = find(upper & isfinite(distances));
        if ~isempty(finitePairs)
            [distanceValue,localIndex] = min(distances(finitePairs));
            matrixIndex = finitePairs(localIndex);
            [row,column] = ind2sub([bodyCount bodyCount],matrixIndex);
            minimumClearance(sampleIndex) = distanceValue;
            closestBodyPair(sampleIndex,:) = [bodyNames(row) bodyNames(column)];
            points = witness(3*row-2:3*row,2*column-1:2*column);
            if isequal(size(points),[3 2]) && all(isfinite(points(:)))
                witnessPoints(sampleIndex,:) = [points(:,1).' points(:,2).'];
            end
        end
    end

    if options.showProgress && ...
            (sampleIndex == sampleCount || mod(sampleIndex,progressStep) == 0)
        fprintf('Collision analysis: %d / %d configurations\n', ...
            sampleIndex,sampleCount);
    end
end

isCollisionFree = ~isSelfColliding;
hasRequiredClearance = isCollisionFree & ...
    minimumClearance >= options.minimumClearance;
isNearCollision = isCollisionFree & ~hasRequiredClearance;

report.robotID = string(robot.id);
report.status = "EVALUATED";
report.q = qSamples;
report.settings.minimumClearance = options.minimumClearance;
report.settings.skippedSelfCollisionPairs = skippedPairs;
report.geometry.source = geometryField(robot,"source","unspecified");
report.geometry.status = geometryField(robot,"status","UNSPECIFIED");
report.bodyNames = bodyNames;
report.isSelfColliding = isSelfColliding;
report.isCollisionFree = isCollisionFree;
report.minimumClearance = minimumClearance;
report.hasRequiredClearance = hasRequiredClearance;
report.isNearCollision = isNearCollision;
report.closestBodyPair = closestBodyPair;
report.witnessPoints = witnessPoints;
report.collisionPairCounts = pairCounts;
report.summary.sampleCount = sampleCount;
report.summary.collisionCount = nnz(isSelfColliding);
report.summary.collisionFreeCount = nnz(isCollisionFree);
report.summary.nearCollisionCount = nnz(isNearCollision);
report.summary.clearanceSatisfiedCount = nnz(hasRequiredClearance);
report.summary.collisionFraction = nnz(isSelfColliding)/max(1,sampleCount);

end

function clearance = defaultClearance(robot)

clearance = 0.010;
if isfield(robot,'collision') && isstruct(robot.collision) && ...
        isfield(robot.collision,'minimumClearance') && ...
        isscalar(robot.collision.minimumClearance) && ...
        isfinite(robot.collision.minimumClearance)
    clearance = robot.collision.minimumClearance;
end

end

function validateInputs(robot,qSamples,minimumClearance)

if ~isfield(robot,'model') || ~isa(robot.model,'rigidBodyTree') || ...
        ~isfield(robot,'structure') || ~isfield(robot.structure,'dof') || ...
        ~isfield(robot,'params') || ~isfield(robot.params,'joints')
    error('collision:InvalidRobot','Expected a robot returned by loadRobot().');
end
if size(qSamples,2) ~= robot.structure.dof || any(~isfinite(qSamples(:)))
    error('collision:InvalidConfigurations', ...
        'qSamples must be a finite N-by-DOF matrix.');
end
limits = robot.params.joints.positionLimits;
tolerance = 1e-12;
if any(qSamples < limits(:,1).'-tolerance,'all') || ...
        any(qSamples > limits(:,2).'+tolerance,'all')
    error('collision:JointLimitViolation', ...
        'Every collision-test configuration must respect mechanical limits.');
end
if ~isfinite(minimumClearance) || minimumClearance < 0
    error('collision:InvalidClearance', ...
        'minimumClearance must be a finite nonnegative distance in metres.');
end

end

function pairs = skippedSelfCollisionPairs(robot)

pairs = strings(0,2);
for bodyIndex = 1:numel(robot.model.Bodies)
    body = robot.model.Bodies{bodyIndex};
    if ~isempty(body.Parent)
        pairs(end+1,:) = [string(body.Parent.Name) string(body.Name)]; %#ok<AGROW>
    end
end

if isfield(robot,'collision') && isstruct(robot.collision) && ...
        isfield(robot.collision,'ignoredSelfCollisionPairs') && ...
        ~isempty(robot.collision.ignoredSelfCollisionPairs)
    additional = string(robot.collision.ignoredSelfCollisionPairs);
    if size(additional,2) ~= 2
        error('collision:InvalidSkippedPairs', ...
            'ignoredSelfCollisionPairs must be an N-by-2 array.');
    end
    pairs = [pairs;additional];
end

if ~isempty(pairs)
    pairs = unique(pairs,'rows','stable');
end

end

function value = geometryField(robot,fieldName,defaultValue)

value = string(defaultValue);
if isfield(robot,'collision') && isstruct(robot.collision) && ...
        isfield(robot.collision,fieldName)
    value = string(robot.collision.(fieldName));
end

end
