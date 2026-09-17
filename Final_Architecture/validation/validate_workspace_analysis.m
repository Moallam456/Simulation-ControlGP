function results = validate_workspace_analysis(robot)

% VALIDATE_WORKSPACE_ANALYSIS Validate generic workspace-analysis behavior.

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath( ...
    rootDir, ...
    fullfile(rootDir,'kinematics'), ...
    fullfile(rootDir,'validation'), ...
    fullfile(rootDir,'workspace_analysis'));

if nargin < 1 || isempty(robot)
    robot = loadRobot();
end

fprintf('\n=====================================\n');
fprintf('WORKSPACE ANALYSIS VALIDATION: %s\n',robot.id);
fprintf('=====================================\n\n');

options.numSamples = 120;
options.seed = 11;
options.showPlot = false;
options.saveFigure = false;

workspaceA = reachable_workspace(robot,options);
workspaceB = reachable_workspace(robot,options);

optionsDifferentSeed = options;
optionsDifferentSeed.seed = 12;
workspaceC = reachable_workspace(robot,optionsDifferentSeed);

results.outputShape = validateOutputShape(robot,workspaceA,options);
results.jointLimits = validateJointLimits(robot,workspaceA);
results.reproducibility = validateReproducibility(workspaceA,workspaceB,workspaceC);
results.fkPoints = validateWorkspacePoints(robot,workspaceA);
results.bounds = validateBounds(workspaceA);
results.radius = validateRadius(workspaceA);
results.volume = validateVolume(workspaceA);
results.geometryPropagation = validateWorkspaceGeometryPropagation(robot,options);
results.plotSmoke = validateWorkspacePlotSmoke(robot);

results.success = ...
    results.outputShape.success && ...
    results.jointLimits.success && ...
    results.reproducibility.success && ...
    results.fkPoints.success && ...
    results.bounds.success && ...
    results.radius.success && ...
    results.volume.success && ...
    results.geometryPropagation.success && ...
    results.plotSmoke.success;

fprintf('\n=====================================\n');
fprintf('WORKSPACE VALIDATION SUCCESS: %d\n',results.success);
fprintf('=====================================\n');

end

function result = validateOutputShape(robot,workspace,options)

result.hasRobotID = workspace.robotID == robot.id;
result.qSampleSize = isequal(size(workspace.qSamples), ...
    [options.numSamples robot.structure.dof]);
result.pointSize = isequal(size(workspace.points),[options.numSamples 3]);
result.finiteSamples = all(isfinite(workspace.qSamples(:)));
result.finitePoints = all(isfinite(workspace.points(:)));

result.success = result.hasRobotID && ...
    result.qSampleSize && ...
    result.pointSize && ...
    result.finiteSamples && ...
    result.finitePoints;

fprintf('Workspace output shape valid: %d\n',result.success);

end

function result = validateJointLimits(robot,workspace)

limits = robot.params.joints.positionLimits;
lower = limits(:,1).';
upper = limits(:,2).';

result.aboveLower = all(workspace.qSamples >= lower,'all');
result.belowUpper = all(workspace.qSamples <= upper,'all');
result.success = result.aboveLower && result.belowUpper;

fprintf('Workspace samples inside limits: %d\n',result.success);

end

function result = validateReproducibility(workspaceA,workspaceB,workspaceC)

result.sameSeedSameSamples = isequal(workspaceA.qSamples,workspaceB.qSamples);
result.sameSeedSamePoints = isequal(workspaceA.points,workspaceB.points);
result.differentSeedDifferentSamples = ~isequal(workspaceA.qSamples,workspaceC.qSamples);
result.success = result.sameSeedSameSamples && ...
    result.sameSeedSamePoints && ...
    result.differentSeedDifferentSamples;

fprintf('Workspace reproducibility valid: %d\n',result.success);

end

function result = validateWorkspacePoints(robot,workspace)

maxPositionError = 0;
maxTransformError = 0;

for i = 1:size(workspace.qSamples,1)
    q = workspace.qSamples(i,:);

    TCustom = forward_kinematics(robot,q);
    TRBT = getTransform( ...
        robot.model, ...
        q, ...
        char(robot.structure.frames.endEffector));

    maxPositionError = max(maxPositionError, ...
        norm(TCustom(1:3,4).' - workspace.points(i,:)));
    maxTransformError = max(maxTransformError,norm(TCustom - TRBT,'fro'));
end

result.maxPositionError = maxPositionError;
result.maxTransformError = maxTransformError;
result.success = maxPositionError < 1e-10 && maxTransformError < 1e-10;

fprintf('Workspace FK point consistency valid: %d\n',result.success);

end

function result = validateBounds(workspace)

points = workspace.points;

result.x = isequal(workspace.bounds.x,[min(points(:,1)) max(points(:,1))]);
result.y = isequal(workspace.bounds.y,[min(points(:,2)) max(points(:,2))]);
result.z = isequal(workspace.bounds.z,[min(points(:,3)) max(points(:,3))]);
result.success = result.x && result.y && result.z;

fprintf('Workspace bounds valid: %d\n',result.success);

end

function result = validateRadius(workspace)

radius = sqrt(sum(workspace.points.^2,2));

result.min = workspace.radius.min == min(radius);
result.max = workspace.radius.max == max(radius);
result.mean = abs(workspace.radius.mean - mean(radius)) < 1e-12;
result.success = result.min && result.max && result.mean;

fprintf('Workspace radius metrics valid: %d\n',result.success);

end

function result = validateVolume(workspace)

result.finite = isfinite(workspace.volume.estimate);
result.nonnegative = workspace.volume.estimate >= 0;
result.occupiedVoxelsFinite = isfinite(workspace.volume.occupiedVoxels);
result.success = result.finite && result.nonnegative && result.occupiedVoxelsFinite;

fprintf('Workspace volume estimate valid: %d\n',result.success);

end

function result = validateWorkspaceGeometryPropagation(robot,options)

params = robot.params;
params.geometry.l2 = params.geometry.l2 + 0.05;
robotB = loadRobot(robot.id,params);

workspaceA = reachable_workspace(robot,options);
workspaceB = reachable_workspace(robotB,options);

deltaZ = workspaceB.points(:,3) - workspaceA.points(:,3);

result.expectedDeltaZ = 0.05;
result.maxDeltaError = max(abs(deltaZ - result.expectedDeltaZ));
result.success = result.maxDeltaError < 1e-10;

fprintf('Workspace geometry propagation valid: %d\n',result.success);

end

function result = validateWorkspacePlotSmoke(robot)

options.numSamples = 25;
options.seed = 1;
options.showPlot = true;
options.saveFigure = false;

oldVisibility = get(0,'DefaultFigureVisible');
set(0,'DefaultFigureVisible','off');
cleanup = onCleanup(@() set(0,'DefaultFigureVisible',oldVisibility));

try
    workspace = reachable_workspace(robot,options);
    result.hasFigureFields = isfield(workspace.figures,'pointCloud') && ...
        isfield(workspace.figures,'xyDensity') && ...
        isfield(workspace.figures,'xzDensity') && ...
        isfield(workspace.figures,'yzDensity');
    result.hasValidHandles = isvalid(workspace.figures.pointCloud) && ...
        isvalid(workspace.figures.xyDensity) && ...
        isvalid(workspace.figures.xzDensity) && ...
        isvalid(workspace.figures.yzDensity);
    result.success = result.hasFigureFields && result.hasValidHandles;
    result.message = "reachable_workspace plot path executed.";
    close(workspace.figures.pointCloud);
    close(workspace.figures.xyDensity);
    close(workspace.figures.xzDensity);
    close(workspace.figures.yzDensity);
catch ME
    close all;
    result.success = false;
    result.message = string(ME.message);
end

fprintf('Workspace plot smoke valid: %d\n',result.success);

end
