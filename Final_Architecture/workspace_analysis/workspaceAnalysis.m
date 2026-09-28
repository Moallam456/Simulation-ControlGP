function workspace = workspaceAnalysis(robot,analysisType,overrides)
% WORKSPACEANALYSIS Shared engine for workspace-analysis entry points.
% Public commands load the selected robot from projectConfig. Common
% workspace and sampling settings are owned here.

arguments
    robot (1,1) struct
    analysisType (1,1) string
    overrides (1,1) struct = struct()
end

rootDir = fileparts(fileparts(mfilename('fullpath')));

% Common workspace settings. Edit these defaults to change every public
% workspace command. Analysis-specific choices remain in their entry files.
settings.numSamples = 50000;
settings.samplingMethod = "sobol";
settings.seed = 1;
settings.voxelSize = 0.025; % [m]
settings.densityBins = 50;
settings.showPlot = true;
settings.saveFigure = false;
settings.outputFolder = fullfile(rootDir,'workspace_analysis','outputs');
settings.cacheEnabled = true;
settings.cacheFolder = fullfile(rootDir,'workspace_analysis','cache');

% Internal identity of the implemented manipulability algorithm. The public
% manipulability file states these choices explicitly. Usable workspace uses
% the same algorithm and supplies only its threshold.
settings.manipulabilityIndex = "inverse-condition";
settings.manipulabilityMotion = "linear";
settings.manipulabilityThreshold = NaN;

settings = applyOverrides(settings,overrides);

analysisType = lower(analysisType);
if ~ismember(analysisType,["reachable","collision_free", ...
        "manipulability","usable"])
    error('workspace:UnknownAnalysis', ...
        ['Supported analyses are reachable, collision_free, ', ...
         'manipulability, and usable.']);
end

settings = normalizeSettings(settings,analysisType);
validateRobotInterface(robot);

fprintf('\nWorkspace analysis: %s\n',strrep(analysisType,'_',' '));
fprintf('Robot: %s\n',robot.id);
fprintf('Requesting %d configurations with %s sampling...\n', ...
    settings.numSamples,settings.samplingMethod);

requiresManipulability = ismember(analysisType,["manipulability","usable"]);
requirements.manipulability = requiresManipulability;
dataset = prepareWorkspaceDataset(robot,settings,requirements);
qSamples = dataset.q;
transforms = dataset.transforms;
points = dataset.points;
sampling = dataset.sampling;
sampleCount = size(qSamples,1);

jointLimitValid = true(sampleCount,1);
collisionFree = NaN(sampleCount,1);
collisionClearance = NaN(sampleCount,1);

collision.status = "NOT_EVALUATED";
collision.filterApplied = false;
collision.message = ...
    "Collision analysis has not been implemented; no configurations were rejected.";
collision.isCollisionFree = collisionFree;
collision.clearance_m = collisionClearance;
collision.rejectedSampleIndices = zeros(0,1);
collision.rejectedConfigurations = zeros(0,robot.structure.dof);
collision.rejectedPoints = zeros(0,3);

if ismember(analysisType,["collision_free","manipulability","usable"])
    warning('workspace:CollisionNotEvaluated','%s',collision.message);
end

collisionAccepted = true(sampleCount,1);
baseAccepted = jointLimitValid & collisionAccepted;
manipulability = initializeManipulability(sampleCount,robot.structure.dof,settings);
if requiresManipulability
    manipulability = summarizeManipulability( ...
        dataset.manipulability,points,baseAccepted,settings);
end

if analysisType == "usable"
    accepted = baseAccepted & manipulability.isAboveThreshold;
elseif analysisType == "reachable"
    accepted = jointLimitValid;
else
    accepted = baseAccepted;
end

metrics = computeMetrics(points(accepted,:),settings);
figures = plotWorkspace(robot,analysisType,points(accepted,:),metrics, ...
    settings,collision,manipulability);

voxelCoordinates = floor(points/settings.voxelSize);
[~,~,sampleVoxelIndex] = unique(voxelCoordinates,'rows');

workspace.robotID = string(robot.id);
workspace.analysisType = analysisType;
workspace.settings = settings;
workspace.sampling = sampling;
workspace.cache = dataset.cache;
workspace.samples.q = qSamples;
workspace.samples.transforms = transforms;
workspace.samples.points = points;
workspace.samples.voxelCoordinates = voxelCoordinates;
workspace.samples.voxelIndex = sampleVoxelIndex;
workspace.masks.jointLimitValid = jointLimitValid;
workspace.masks.collisionFree = collisionFree;
workspace.masks.aboveManipulabilityThreshold = ...
    manipulability.isAboveThreshold;
workspace.masks.accepted = accepted;
workspace.collision = collision;
workspace.manipulability = manipulability;
workspace.accepted.sampleIndices = find(accepted);
workspace.accepted.q = qSamples(accepted,:);
workspace.accepted.points = points(accepted,:);
workspace.accepted.voxelIndex = metrics.sampleVoxelIndex;

% Compatibility fields retained for existing validation and scripts.
workspace.qSamples = qSamples;
workspace.points = points;
workspace.bounds = metrics.bounds;
workspace.radius = metrics.radius;
workspace.volume = metrics.volume;
workspace.density = metrics.density;
workspace.voxels = metrics.voxels;
workspace.figures = figures;
workspace.summary = createSummary(workspace);
workspace.readiness = readinessMetadata(robot);

fprintf('%s\n',workspace.summary);
if ismember(analysisType,["collision_free","manipulability","usable"])
    fprintf('Collision status: NOT_EVALUATED (pass-through result)\n');
end

end

function settings = applyOverrides(settings,overrides)
names = fieldnames(overrides);
for i = 1:numel(names)
    name = names{i};
    if ~isfield(settings,name)
        error('workspace:UnknownSetting','Unknown workspace setting "%s".',name);
    end
    settings.(name) = overrides.(name);
end
end

function settings = normalizeSettings(settings,analysisType)
validatePositiveInteger(settings.numSamples,'numSamples');
validatePositiveInteger(settings.densityBins,'densityBins');
if ~isscalar(settings.seed) || ~isfinite(settings.seed) || ...
        settings.seed < 0 || settings.seed ~= floor(settings.seed)
    error('workspace:InvalidSetting','seed must be a nonnegative integer.');
end
if ~isscalar(settings.voxelSize) || ~isfinite(settings.voxelSize) || ...
        settings.voxelSize <= 0
    error('workspace:InvalidSetting','voxelSize must be positive meters.');
end
settings.samplingMethod = lower(string(settings.samplingMethod));
if ~ismember(settings.samplingMethod,["sobol","random"])
    error('workspace:InvalidSetting', ...
        'samplingMethod must be sobol or random.');
end
settings.showPlot = logical(settings.showPlot);
settings.saveFigure = logical(settings.saveFigure);
settings.outputFolder = string(settings.outputFolder);
if ~isscalar(settings.cacheEnabled) || ...
        ~(islogical(settings.cacheEnabled) || isnumeric(settings.cacheEnabled))
    error('workspace:InvalidSetting','cacheEnabled must be logical.');
end
settings.cacheEnabled = logical(settings.cacheEnabled);
settings.cacheFolder = string(settings.cacheFolder);
if ~isscalar(settings.cacheFolder) || strlength(settings.cacheFolder) == 0
    error('workspace:InvalidSetting','cacheFolder must be a nonempty path.');
end
settings.manipulabilityIndex = lower(string(settings.manipulabilityIndex));
if settings.manipulabilityIndex ~= "inverse-condition"
    error('workspace:InvalidSetting', ...
        'manipulabilityIndex must be inverse-condition.');
end
settings.manipulabilityMotion = lower(string(settings.manipulabilityMotion));
if settings.manipulabilityMotion ~= "linear"
    error('workspace:InvalidSetting', ...
        'manipulabilityMotion must be linear.');
end
if analysisType == "usable"
    if ~isscalar(settings.manipulabilityThreshold) || ...
            ~isfinite(settings.manipulabilityThreshold) || ...
            settings.manipulabilityThreshold < 0 || ...
            settings.manipulabilityThreshold > 1
        error('workspace:InvalidSetting', ...
            'manipulabilityThreshold must be between 0 and 1.');
    end
elseif ~isscalar(settings.manipulabilityThreshold) || ...
        ~(isnan(settings.manipulabilityThreshold) || ...
        (isfinite(settings.manipulabilityThreshold) && ...
        settings.manipulabilityThreshold >= 0 && ...
        settings.manipulabilityThreshold <= 1))
    error('workspace:InvalidSetting', ...
        'manipulabilityThreshold must be NaN or between 0 and 1.');
end
end

function validatePositiveInteger(value,name)
if ~isscalar(value) || ~isfinite(value) || value < 1 || value ~= floor(value)
    error('workspace:InvalidSetting','%s must be a positive integer.',name);
end
end

function validateRobotInterface(robot)
required = {'id','model','params','structure'};
if ~all(isfield(robot,required)) || ~isa(robot.model,'rigidBodyTree') || ...
        ~isfield(robot.params,'joints') || ...
        ~isfield(robot.params.joints,'positionLimits')
    error('workspace:InvalidRobot','Expected a robot returned by loadRobot().');
end
limits = robot.params.joints.positionLimits;
if ~isequal(size(limits),[robot.structure.dof 2]) || ...
        any(~isfinite(limits(:))) || any(limits(:,1)>=limits(:,2))
    error('workspace:InvalidJointLimits', ...
        'Robot mechanical limits must be a finite DOF-by-2 matrix.');
end
end

function result = initializeManipulability(sampleCount,dof,settings)
result.status = "NOT_EVALUATED";
result.index = settings.manipulabilityIndex;
result.motion = settings.manipulabilityMotion;
result.definition = "sigma_min(Jv) / sigma_max(Jv)";
result.threshold = settings.manipulabilityThreshold;
result.values = NaN(sampleCount,1);
result.conditionNumber = NaN(sampleCount,1);
result.singularValues = NaN(sampleCount,min(3,dof));
result.isAboveThreshold = false(sampleCount,1);
result.statistics.min = NaN;
result.statistics.max = NaN;
result.statistics.mean = NaN;
result.statistics.median = NaN;
result.voxels.indices = zeros(0,3);
result.voxels.centers = zeros(0,3);
result.voxels.sampleCount = zeros(0,1);
result.voxels.min = zeros(0,1);
result.voxels.max = zeros(0,1);
result.voxels.mean = zeros(0,1);
end

function result = summarizeManipulability(raw,points,eligible,settings)
sampleCount = size(points,1);
singularValueCount = size(raw.singularValues,2);
result = initializeManipulability(sampleCount,singularValueCount,settings);
result.status = raw.status;
result.index = raw.index;
result.motion = raw.motion;
result.definition = raw.definition;
result.values = raw.values;
result.conditionNumber = raw.conditionNumber;
result.singularValues = raw.singularValues;
result.isAboveThreshold = raw.values >= settings.manipulabilityThreshold;
result.statistics.min = min(raw.values(eligible));
result.statistics.max = max(raw.values(eligible));
result.statistics.mean = mean(raw.values(eligible));
result.statistics.median = median(raw.values(eligible));
result.voxels = aggregateManipulability( ...
    points(eligible,:),raw.values(eligible),settings.voxelSize);
end

function voxels = aggregateManipulability(points,values,voxelSize)
if isempty(points)
    voxels.indices = zeros(0,3);
    voxels.centers = zeros(0,3);
    voxels.sampleCount = zeros(0,1);
    voxels.min = zeros(0,1);
    voxels.max = zeros(0,1);
    voxels.mean = zeros(0,1);
    return;
end

indices = floor(points/voxelSize);
[uniqueIndices,~,group] = unique(indices,'rows');
voxels.indices = uniqueIndices;
voxels.centers = (uniqueIndices+0.5)*voxelSize;
voxels.sampleCount = accumarray(group,1);
voxels.min = accumarray(group,values,[],@min);
voxels.max = accumarray(group,values,[],@max);
voxels.mean = accumarray(group,values,[],@mean);
end

function metrics = computeMetrics(points,settings)
if size(points,2) ~= 3 || any(~isfinite(points(:)))
    error('workspace:InvalidPoints','Accepted workspace points must be finite N-by-3 data.');
end

if isempty(points)
    metrics.bounds.x = [NaN NaN];
    metrics.bounds.y = [NaN NaN];
    metrics.bounds.z = [NaN NaN];
    metrics.radius.min = NaN;
    metrics.radius.max = NaN;
    metrics.radius.mean = NaN;
    metrics.voxels.origin = [NaN NaN NaN];
    metrics.voxels.size = settings.voxelSize;
    metrics.voxels.indices = zeros(0,3);
    metrics.voxels.centers = zeros(0,3);
    metrics.voxels.sampleCount = zeros(0,1);
    metrics.sampleVoxelIndex = zeros(0,1);
    metrics.volume.estimate = 0;
    metrics.volume.occupiedVoxels = 0;
    metrics.volume.voxelSize = settings.voxelSize;
    metrics.density.xy = emptyDensity(settings.densityBins);
    metrics.density.xz = emptyDensity(settings.densityBins);
    metrics.density.yz = emptyDensity(settings.densityBins);
    return;
end

metrics.bounds.x = [min(points(:,1)) max(points(:,1))];
metrics.bounds.y = [min(points(:,2)) max(points(:,2))];
metrics.bounds.z = [min(points(:,3)) max(points(:,3))];
radius = vecnorm(points,2,2);
metrics.radius.min = min(radius);
metrics.radius.max = max(radius);
metrics.radius.mean = mean(radius);

origin = floor(min(points,[],1)/settings.voxelSize)*settings.voxelSize;
voxelIndices = floor((points-origin)/settings.voxelSize)+1;
[uniqueIndices,~,sampleVoxelIndex] = unique(voxelIndices,'rows');
counts = accumarray(sampleVoxelIndex,1);
centers = origin+(uniqueIndices-0.5)*settings.voxelSize;
metrics.voxels.origin = origin;
metrics.voxels.size = settings.voxelSize;
metrics.voxels.indices = uniqueIndices;
metrics.voxels.centers = centers;
metrics.voxels.sampleCount = counts;
metrics.sampleVoxelIndex = sampleVoxelIndex;
metrics.volume.estimate = size(uniqueIndices,1)*settings.voxelSize^3;
metrics.volume.occupiedVoxels = size(uniqueIndices,1);
metrics.volume.voxelSize = settings.voxelSize;

metrics.density.xy = density2D(points(:,1),points(:,2),settings.densityBins);
metrics.density.xz = density2D(points(:,1),points(:,3),settings.densityBins);
metrics.density.yz = density2D(points(:,2),points(:,3),settings.densityBins);
end

function density = emptyDensity(numBins)
density.counts = zeros(numBins,numBins);
density.edgesA = linspace(0,1,numBins+1);
density.edgesB = linspace(0,1,numBins+1);
density.numBins = numBins;
end

function density = density2D(a,b,numBins)
[counts,aEdges,bEdges] = histcounts2(a,b,numBins);
density.counts = counts;
density.edgesA = aEdges;
density.edgesB = bEdges;
density.numBins = numBins;
end

function figures = plotWorkspace( ...
        robot,analysisType,points,metrics,settings,collision,manipulability)
figures.main = [];
figures.pointCloud = [];
figures.savedFiles = strings(0,1);
if ~settings.showPlot && ~settings.saveFigure
    return;
end
if settings.saveFigure && ~isfolder(settings.outputFolder)
    mkdir(settings.outputFolder);
end
visibility = matlab.lang.OnOffSwitchState(settings.showPlot);
figures.main = figure('Name',char(plotTitle(robot,analysisType,collision)), ...
    'Color','w','Visible',visibility);
figures.pointCloud = figures.main;
if ismember(analysisType,["manipulability","usable"])
    centers = manipulability.voxels.centers;
    values = manipulability.voxels.max;
    if analysisType == "usable"
        keep = values >= manipulability.threshold;
        centers = centers(keep,:);
        values = values(keep);
    end
    scatter3(centers(:,1),centers(:,2),centers(:,3),12,values,'filled');
    colorbarHandle = colorbar;
    colorbarHandle.Label.String = 'Linear inverse condition number';
    clim([0 1]);
    colormap(parula(256));
else
    scatter3(points(:,1),points(:,2),points(:,3),5, ...
        'filled','MarkerFaceAlpha',0.35);
end
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(plotTitle(robot,analysisType,collision),'Interpreter','none');
if analysisType == "usable"
    subtitle(sprintf([ ...
        '%d accepted configurations | index >= %.3f | ', ...
        '%.0f mm voxels | collision filter not evaluated'], ...
        size(points,1),manipulability.threshold,1000*metrics.voxels.size));
elseif analysisType == "manipulability"
    subtitle(sprintf([ ...
        '%d sampled configurations | voxel color = best index | ', ...
        '%.0f mm voxels | collision filter not evaluated'], ...
        size(points,1),1000*metrics.voxels.size));
else
    subtitle(sprintf('%d samples | %.0f mm voxels | %.4f m^3 occupied', ...
        size(points,1),1000*metrics.voxels.size,metrics.volume.estimate));
end
view(135,25);
if settings.saveFigure
    fileName = sprintf('%s_%s.png',robot.id,analysisType);
    filePath = fullfile(settings.outputFolder,fileName);
    exportgraphics(figures.main,filePath,'Resolution',150);
    figures.savedFiles = string(filePath);
end
end

function value = plotTitle(robot,analysisType,collision)
if analysisType == "reachable"
    value = "Reachable Workspace - " + string(robot.id);
elseif analysisType == "collision_free" && ~collision.filterApplied
    value = "Collision-Free Workspace - " + string(robot.id) + ...
        " [COLLISION FILTER NOT EVALUATED]";
elseif analysisType == "collision_free"
    value = "Collision-Free Workspace - " + string(robot.id);
elseif analysisType == "manipulability"
    value = "Manipulability Workspace - " + string(robot.id) + ...
        " [COLLISION FILTER NOT EVALUATED]";
else
    value = "Usable Workspace - " + string(robot.id) + ...
        " [COLLISION FILTER NOT EVALUATED]";
end
end

function summary = createSummary(workspace)
summary = sprintf([ ...
    '%s workspace: X[%.3f, %.3f] m, Y[%.3f, %.3f] m, ', ...
    'Z[%.3f, %.3f] m, maximum radius %.3f m, occupied volume %.4f m^3.'], ...
    strrep(char(workspace.analysisType),'_',' '), ...
    workspace.bounds.x(1),workspace.bounds.x(2), ...
    workspace.bounds.y(1),workspace.bounds.y(2), ...
    workspace.bounds.z(1),workspace.bounds.z(2), ...
    workspace.radius.max,workspace.volume.estimate);
if workspace.analysisType == "manipulability"
    summary = sprintf('%s Index range [%.4f, %.4f], mean %.4f.', ...
        summary,workspace.manipulability.statistics.min, ...
        workspace.manipulability.statistics.max, ...
        workspace.manipulability.statistics.mean);
elseif workspace.analysisType == "usable"
    summary = sprintf('%s Accepted %d of %d configurations at index >= %.3f.', ...
        summary,numel(workspace.accepted.sampleIndices), ...
        size(workspace.samples.q,1),workspace.manipulability.threshold);
end
end

function metadata = readinessMetadata(robot)
if isfield(robot,'readiness')
    metadata = robot.readiness;
else
    metadata.status = "UNSPECIFIED";
    metadata.warnings = strings(0,1);
end
end
