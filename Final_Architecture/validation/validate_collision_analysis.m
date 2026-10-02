function results = validate_collision_analysis(robot)
% VALIDATE_COLLISION_ANALYSIS Validate geometry ownership and collision data.

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'collision_analysis'));
if nargin < 1 || isempty(robot)
    robot = loadRobot();
end

fprintf('\n=====================================\n');
fprintf('COLLISION ANALYSIS VALIDATION: %s\n',robot.id);
fprintf('=====================================\n\n');

if string(robot.id) ~= "initial_trial"
    error('validate_collision_analysis:UnsupportedRobot', ...
        'This geometry-ownership validation currently targets initial_trial.');
end

results.geometry = validateGeometryOwnership(robot);
results.analysis = validateNumericalAnalysis(robot);
results.rendering = validateRendering(robot);
results.success = results.geometry.success && ...
    results.analysis.success && results.rendering.success;

fprintf('Collision validation success: %d\n',results.success);

end

function result = validateGeometryOwnership(robot)

definitions = robot.collision.geometry;
bodyNames = string(robot.model.BodyNames);
expectedCounts = zeros(size(bodyNames));
actualCounts = zeros(size(bodyNames));
for i = 1:numel(bodyNames)
    expectedCounts(i) = nnz([definitions.bodyName] == bodyNames(i));
    actualCounts(i) = numel(getBody(robot.model,char(bodyNames(i))).Collisions);
end

validDefinitions = true;
for i = 1:numel(definitions)
    shape = definitions(i);
    validDefinitions = validDefinitions && ...
        ismember(shape.type,["Cylinder" "Sphere"]) && ...
        all(isfinite(shape.dimensions)) && all(shape.dimensions > 0) && ...
        isequal(size(shape.transform),[4 4]) && ...
        all(isfinite(shape.transform(:))) && ...
        norm(shape.transform(4,:)-[0 0 0 1]) < 1e-12;
end

result.expectedCounts = expectedCounts;
result.actualCounts = actualCounts;
result.definitionCount = numel(definitions);
result.attachmentCount = sum(actualCounts);
result.validDefinitions = validDefinitions;
result.authoritativeCountsMatch = isequal(expectedCounts,actualCounts) && ...
    result.definitionCount == result.attachmentCount;
result.success = robot.collision.available && ...
    result.validDefinitions && result.authoritativeCountsMatch;
fprintf('Collision geometry ownership valid: %d (%d configured/attached)\n', ...
    result.success,result.definitionCount);

end

function result = validateNumericalAnalysis(robot)

sampleCount = 180;
limits = robot.params.joints.positionLimits;
previousState = rng;
cleanup = onCleanup(@() rng(previousState));
rng(5,'twister');
q = limits(:,1).'+rand(sampleCount,robot.structure.dof).* ...
    (limits(:,2)-limits(:,1)).';
q(1,:) = robot.params.joints.homePosition;
report = analyzeCollisions(robot,q);

probeIndices = unique(round(linspace(1,sampleCount,9)));
direct = false(numel(probeIndices),1);
for i = 1:numel(probeIndices)
    direct(i) = checkCollision(robot.model,q(probeIndices(i),:), ...
        'SkippedSelfCollisions', ...
        cellstr(report.settings.skippedSelfCollisionPairs), ...
        'Exhaustive','on');
end

result.homeIsCollisionFree = report.isCollisionFree(1);
result.detectsCollision = any(report.isSelfColliding);
result.partitionsSamples = all(xor(report.isSelfColliding, ...
    report.isCollisionFree));
result.directAgreement = isequal(direct, ...
    report.isSelfColliding(probeIndices));
result.clearanceConsistent = ...
    all(report.minimumClearance(report.isSelfColliding) == 0) && ...
    all(report.minimumClearance(report.isCollisionFree) >= 0);
result.marginConsistent = isequal(report.hasRequiredClearance, ...
    report.isCollisionFree & report.minimumClearance >= ...
    report.settings.minimumClearance);
result.success = result.homeIsCollisionFree && result.detectsCollision && ...
    result.partitionsSamples && result.directAgreement && ...
    result.clearanceConsistent && result.marginConsistent;
fprintf('Collision and clearance calculations valid: %d\n',result.success);

end

function result = validateRendering(robot)

oldVisibility = get(0,'DefaultFigureVisible');
set(0,'DefaultFigureVisible','off');
cleanup = onCleanup(@() set(0,'DefaultFigureVisible',oldVisibility));
try
    figure;
    show(robot.model,robot.params.joints.homePosition, ...
        'Visuals','off','Collisions','on','Frames','off');
    drawnow;
    close(gcf);
    session = inspectCollisions(robot,struct('numSamples',12,'visible',false));
    drawnow;
    result.inspectorReportValid = ...
        session.report.summary.sampleCount == 12;
    close(session.figure);
    result.success = result.inspectorReportValid;
    result.message = "Collision model and inspector rendered.";
catch exception
    close all;
    result.success = false;
    result.message = string(exception.message);
end
fprintf('Collision rendering and inspector valid: %d\n',result.success);

end
