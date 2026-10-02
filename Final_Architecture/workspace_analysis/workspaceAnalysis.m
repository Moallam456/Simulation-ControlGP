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
addpath(fullfile(rootDir,'analysis_cache'), ...
    fullfile(rootDir,'collision_analysis'));

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
settings.cacheFolder = fullfile(rootDir,'analysis_cache','data');

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
requiresCollision = ismember(analysisType,["collision_free", ...
    "manipulability","usable"]);
requirements.collision = requiresCollision;
dataset = prepareWorkspaceDataset(robot,settings,requirements);
qSamples = dataset.q;
transforms = dataset.transforms;
points = dataset.points;
sampling = dataset.sampling;
sampleCount = size(qSamples,1);

jointLimitValid = true(sampleCount,1);
collision = initializeCollision(sampleCount,robot.structure.dof);
collisionAccepted = true(sampleCount,1);
if requiresCollision
    rawCollision = dataset.collision;
    collision.status = rawCollision.status;
    collision.filterApplied = true;
    collision.message = "Self-colliding configurations were rejected.";
    collision.isCollisionFree = rawCollision.isCollisionFree;
    collision.clearance_m = rawCollision.minimumClearance;
    collision.hasRequiredClearance = rawCollision.hasRequiredClearance;
    collision.isNearCollision = rawCollision.isNearCollision;
    collision.closestBodyPair = rawCollision.closestBodyPair;
    collision.witnessPoints = rawCollision.witnessPoints;
    collision.skippedSelfCollisionPairs = ...
        rawCollision.settings.skippedSelfCollisionPairs;
    collision.rejectedSampleIndices = find(~rawCollision.isCollisionFree);
    collision.rejectedConfigurations = ...
        qSamples(~rawCollision.isCollisionFree,:);
    collision.rejectedPoints = points(~rawCollision.isCollisionFree,:);
    collision.summary = rawCollision.summary;
    collisionAccepted = rawCollision.isCollisionFree;
end
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
figures = plotWorkspace(robot,analysisType,points,accepted,metrics, ...
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
workspace.masks.collisionFree = collision.isCollisionFree;
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
if requiresCollision
    fprintf('Collision filter: %d free, %d colliding, %d near the %.1f mm margin.\n', ...
        nnz(collision.isCollisionFree),nnz(~collision.isCollisionFree), ...
        nnz(collision.isNearCollision), ...
        1000*dataset.collision.settings.minimumClearance);
end

end

function collision = initializeCollision(sampleCount,dof)
collision.status = "NOT_REQUESTED";
collision.filterApplied = false;
collision.message = "Collision analysis was not requested for this result.";
collision.isCollisionFree = NaN(sampleCount,1);
collision.clearance_m = NaN(sampleCount,1);
collision.hasRequiredClearance = false(sampleCount,1);
collision.isNearCollision = false(sampleCount,1);
collision.closestBodyPair = strings(sampleCount,2);
collision.witnessPoints = NaN(sampleCount,6);
collision.skippedSelfCollisionPairs = strings(0,2);
collision.rejectedSampleIndices = zeros(0,1);
collision.rejectedConfigurations = zeros(0,dof);
collision.rejectedPoints = zeros(0,3);
collision.summary = struct();
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
if any(eligible)
    result.statistics.min = min(raw.values(eligible));
    result.statistics.max = max(raw.values(eligible));
    result.statistics.mean = mean(raw.values(eligible));
    result.statistics.median = median(raw.values(eligible));
end
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
        robot,analysisType,allPoints,accepted,metrics,settings,collision,manipulability)
figures.main = [];
figures.pointCloud = [];
figures.robotAxes = [];
figures.jointSliders = gobjects(0,1);
figures.homeButton = [];
figures.savedFiles = strings(0,1);
if ~settings.showPlot && ~settings.saveFigure
    return;
end
if settings.saveFigure && ~isfolder(settings.outputFolder)
    mkdir(settings.outputFolder);
end
visibility = matlab.lang.OnOffSwitchState(settings.showPlot);
figures.main = figure('Name',char(plotTitle(robot,analysisType)), ...
    'Color','w','Visible',visibility,'NumberTitle','off', ...
    'Position',[80 70 1280 720]);
figures.pointCloud = figures.main;
workspaceAxes = axes(figures.main,'Position',[0.07 0.11 0.65 0.81]);
if ismember(analysisType,["manipulability","usable"])
    centers = manipulability.voxels.centers;
    values = manipulability.voxels.max;
    if analysisType == "usable"
        keep = values >= manipulability.threshold;
        centers = centers(keep,:);
        values = values(keep);
    end
    scatter3(workspaceAxes,centers(:,1),centers(:,2),centers(:,3), ...
        12,values,'filled');
    colorbarHandle = colorbar(workspaceAxes);
    colorbarHandle.Label.String = 'Linear inverse condition number';
    clim([0 1]);
    colormap(parula(256));
else
    if analysisType == "collision_free"
        colliding = ~collision.isCollisionFree;
        scatter3(workspaceAxes,allPoints(colliding,1),allPoints(colliding,2), ...
            allPoints(colliding,3),5,[0.78 0.82 0.86], ...
            'filled','MarkerFaceAlpha',0.12);
        hold(workspaceAxes,'on');
        scatter3(workspaceAxes,allPoints(accepted,1),allPoints(accepted,2), ...
            allPoints(accepted,3),5,[0.00 0.35 0.68], ...
            'filled','MarkerFaceAlpha',0.45);
        legend(workspaceAxes, ...
            {'Colliding configuration','Collision-free configuration'}, ...
            'Location','southeast');
    else
        scatter3(workspaceAxes,allPoints(accepted,1),allPoints(accepted,2), ...
            allPoints(accepted,3),5,'filled','MarkerFaceAlpha',0.35);
    end
end
grid(workspaceAxes,'on');
axis(workspaceAxes,'equal');
xlabel(workspaceAxes,'X (m)');
ylabel(workspaceAxes,'Y (m)');
zlabel(workspaceAxes,'Z (m)');
title(workspaceAxes,plotTitle(robot,analysisType),'Interpreter','none');
if analysisType == "usable"
    subtitle(workspaceAxes,sprintf([ ...
        '%d accepted configurations | index >= %.3f | ', ...
        '%.0f mm voxels | collision-free only'], ...
        nnz(accepted),manipulability.threshold,1000*metrics.voxels.size));
elseif analysisType == "manipulability"
    subtitle(workspaceAxes,sprintf([ ...
        '%d collision-free configurations | voxel color = best index | ', ...
        '%.0f mm voxels'],nnz(accepted),1000*metrics.voxels.size));
elseif analysisType == "collision_free"
    subtitle(workspaceAxes,sprintf([ ...
        '%d collision-free | %d colliding shown in light gray | ', ...
        '%.0f mm voxels | %.4f m^3 occupied'], ...
        nnz(accepted),nnz(~accepted),1000*metrics.voxels.size, ...
        metrics.volume.estimate));
else
    subtitle(workspaceAxes,sprintf( ...
        '%d samples | %.0f mm voxels | %.4f m^3 occupied', ...
        size(allPoints,1),1000*metrics.voxels.size,metrics.volume.estimate));
end
view(workspaceAxes,135,25);
[figures.robotAxes,figures.jointSliders,figures.homeButton] = ...
    addWorkspaceRobotControls(robot,figures.main,workspaceAxes);
if settings.saveFigure
    fileName = sprintf('%s_%s.png',robot.id,analysisType);
    filePath = fullfile(settings.outputFolder,fileName);
    exportgraphics(figures.main,filePath,'Resolution',150);
    figures.savedFiles = string(filePath);
end
end

function [robotAxes,sliders,homeButton] = ...
        addWorkspaceRobotControls(robot,figureHandle,workspaceAxes)

qHome = robot.params.joints.homePosition;
qCurrent = qHome;
limits = robot.params.joints.positionLimits;
dof = robot.structure.dof;

robotAxes = axes(figureHandle,'Position',workspaceAxes.Position, ...
    'Color','none','HandleVisibility','off');
drawRobot(false);

workspaceLimits = [workspaceAxes.XLim;workspaceAxes.YLim;workspaceAxes.ZLim];
bodyOrigins = zeros(numel(robot.model.BodyNames)+1,3);
for bodyIndex = 1:numel(robot.model.BodyNames)
    bodyTransform = getTransform(robot.model,qHome, ...
        robot.model.BodyNames{bodyIndex});
    bodyOrigins(bodyIndex+1,:) = bodyTransform(1:3,4).';
end
robotLimits = [min(bodyOrigins,[],1).' max(bodyOrigins,[],1).'];
combinedLimits = [min(workspaceLimits(:,1),robotLimits(:,1)) ...
    max(workspaceLimits(:,2),robotLimits(:,2))];
padding = max(0.04,0.04*max(diff(combinedLimits,1,2)));
combinedLimits = combinedLimits+[-padding padding];
workspaceAxes.XLim = combinedLimits(1,:);
workspaceAxes.YLim = combinedLimits(2,:);
workspaceAxes.ZLim = combinedLimits(3,:);
robotAxes.XLim = combinedLimits(1,:);
robotAxes.YLim = combinedLimits(2,:);
robotAxes.ZLim = combinedLimits(3,:);
set([workspaceAxes robotAxes],'XLimMode','manual','YLimMode','manual', ...
    'ZLimMode','manual');

cameraLink = linkprop([workspaceAxes robotAxes],{ ...
    'CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle', ...
    'Projection','View','XLim','YLim','ZLim'});
setappdata(figureHandle,'WorkspaceAxesLink',cameraLink);

panel = uipanel(figureHandle,'Title','Robot joint control', ...
    'Units','normalized','Position',[0.77 0.08 0.21 0.84], ...
    'BackgroundColor','w');
tcpText = uicontrol(panel,'Style','text','Units','normalized', ...
    'Position',[0.06 0.89 0.88 0.07],'BackgroundColor','w', ...
    'HorizontalAlignment','left','FontWeight','bold');

sliders = gobjects(dof,1);
valueLabels = gobjects(dof,1);
sliderListeners = cell(dof,1);
isResetting = false;
top = 0.84;
rowHeight = min(0.115,0.67/max(1,dof));
for jointIndex = 1:dof
    y = top-(jointIndex-1)*rowHeight;
    uicontrol(panel,'Style','text','Units','normalized', ...
        'Position',[0.06 y 0.18 0.045], ...
        'String',char(robot.structure.jointNames(jointIndex)), ...
        'BackgroundColor','w','HorizontalAlignment','left');
    valueLabels(jointIndex) = uicontrol(panel,'Style','text', ...
        'Units','normalized','Position',[0.72 y 0.23 0.045], ...
        'BackgroundColor','w','HorizontalAlignment','right');
    span = limits(jointIndex,2)-limits(jointIndex,1);
    smallStep = min(1,deg2rad(1)/span);
    largeStep = min(1,deg2rad(10)/span);
    sliders(jointIndex) = uicontrol(panel,'Style','slider', ...
        'Units','normalized','Position',[0.06 y-0.045 0.89 0.045], ...
        'Min',limits(jointIndex,1),'Max',limits(jointIndex,2), ...
        'Value',qHome(jointIndex),'SliderStep',[smallStep largeStep], ...
        'Callback',[]);
    sliderListeners{jointIndex} = addlistener( ...
        sliders(jointIndex),'Value','PostSet', ...
        @(~,~) moveJoint(jointIndex,sliders(jointIndex).Value));
end

homeButton = uicontrol(panel,'Style','pushbutton','Units','normalized', ...
    'Position',[0.06 0.035 0.42 0.065],'String','Home', ...
    'Callback',@resetHome);
framesToggle = uicontrol(panel,'Style','checkbox','Units','normalized', ...
    'Position',[0.54 0.035 0.40 0.065],'String','Frames', ...
    'Value',0,'BackgroundColor','w','Callback',@toggleFrames);
setappdata(figureHandle,'WorkspaceRobotConfiguration',qCurrent);
setappdata(figureHandle,'WorkspaceRobotSliderListeners',sliderListeners);
updateReadouts();

    function moveJoint(jointIndex,value)
        if isResetting
            return;
        end
        qCurrent(jointIndex) = value;
        setappdata(figureHandle,'WorkspaceRobotConfiguration',qCurrent);
        drawRobot(true);
        updateReadouts();
    end

    function resetHome(~,~)
        qCurrent = qHome;
        isResetting = true;
        for index = 1:dof
            sliders(index).Value = qCurrent(index);
        end
        isResetting = false;
        setappdata(figureHandle,'WorkspaceRobotConfiguration',qCurrent);
        drawRobot(true);
        updateReadouts();
    end

    function toggleFrames(~,~)
        drawRobot(false);
    end

    function drawRobot(fastUpdate)
        if exist('framesToggle','var') && isgraphics(framesToggle) && ...
                framesToggle.Value
            frames = 'on';
        else
            frames = 'off';
        end
        show(robot.model,qCurrent,'Parent',robotAxes, ...
            'Visuals','on','Collisions','off','Frames',frames, ...
            'PreservePlot',false,'FastUpdate',fastUpdate);
        robotAxes.Color = 'none';
        robotAxes.XColor = 'none';
        robotAxes.YColor = 'none';
        robotAxes.ZColor = 'none';
        robotAxes.GridLineStyle = 'none';
        robotAxes.XLim = combinedLimitOrCurrent(1,robotAxes.XLim);
        robotAxes.YLim = combinedLimitOrCurrent(2,robotAxes.YLim);
        robotAxes.ZLim = combinedLimitOrCurrent(3,robotAxes.ZLim);
        view(robotAxes,workspaceAxes.View);
    end

    function values = combinedLimitOrCurrent(dimension,currentValues)
        if exist('combinedLimits','var')
            values = combinedLimits(dimension,:);
        else
            values = currentValues;
        end
    end

    function updateReadouts()
        for index = 1:dof
            valueLabels(index).String = sprintf('%+.1f deg', ...
                rad2deg(qCurrent(index)));
        end
        T = getTransform(robot.model,qCurrent, ...
            char(robot.structure.frames.endEffector));
        tcpText.String = sprintf('TCP  [%.3f, %.3f, %.3f] m', ...
            T(1,4),T(2,4),T(3,4));
    end

end

function value = plotTitle(robot,analysisType)
if analysisType == "reachable"
    value = "Reachable Workspace - " + string(robot.id);
elseif analysisType == "collision_free"
    value = "Collision-Free Workspace - " + string(robot.id);
elseif analysisType == "manipulability"
    value = "Manipulability Workspace - " + string(robot.id);
else
    value = "Usable Workspace - " + string(robot.id);
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
