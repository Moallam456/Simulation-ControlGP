function results = validate_workspace_analysis(robot)

% VALIDATE_WORKSPACE_ANALYSIS Validate generic workspace-analysis behavior.

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath( ...
    rootDir, ...
    fullfile(rootDir,'kinematics'), ...
    fullfile(rootDir,'validation'), ...
    fullfile(rootDir,'workspace_analysis'), ...
    fullfile(rootDir,'analysis_cache'), ...
    fullfile(rootDir,'collision_analysis'));

if nargin < 1 || isempty(robot)
    robot = loadRobot();
end

fprintf('\n=====================================\n');
fprintf('WORKSPACE ANALYSIS VALIDATION: %s\n',robot.id);
fprintf('=====================================\n\n');

options = struct();
options.numSamples = 120;
options.seed = 11;
options.showPlot = false;
options.saveFigure = false;
options.cacheEnabled = false;

workspaceA = workspaceAnalysis(robot,"reachable",options);
options = workspaceA.settings;
workspaceB = workspaceAnalysis(robot,"reachable",options);

optionsDifferentSeed = options;
optionsDifferentSeed.seed = 12;
workspaceC = workspaceAnalysis(robot,"reachable",optionsDifferentSeed);
collisionWorkspace = workspaceAnalysis(robot,"collision_free",options);
manipulabilityWorkspace = workspaceAnalysis(robot,"manipulability",options);
usableOptions = options;
usableOptions.manipulabilityThreshold = 0.40;
usableWorkspace = workspaceAnalysis(robot,"usable",usableOptions);

results.outputShape = validateOutputShape(robot,workspaceA,options);
results.jointLimits = validateJointLimits(robot,workspaceA);
results.reproducibility = validateReproducibility(workspaceA,workspaceB,workspaceC);
results.fkPoints = validateWorkspacePoints(robot,workspaceA);
results.bounds = validateBounds(workspaceA);
results.radius = validateRadius(workspaceA);
results.volume = validateVolume(workspaceA);
results.collisionFiltering = validateCollisionFiltering( ...
    workspaceA,collisionWorkspace);
results.manipulability = validateManipulability( ...
    robot,workspaceA,manipulabilityWorkspace);
results.usable = validateUsableWorkspace( ...
    manipulabilityWorkspace,usableWorkspace,usableOptions);
results.cache = validateWorkspaceCache(robot,usableOptions);
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
    results.collisionFiltering.success && ...
    results.manipulability.success && ...
    results.usable.success && ...
    results.cache.success && ...
    results.geometryPropagation.success && ...
    results.plotSmoke.success;

fprintf('\n=====================================\n');
fprintf('WORKSPACE VALIDATION SUCCESS: %d\n',results.success);
fprintf('=====================================\n');

end

function result = validateWorkspaceCache(robot,baseOptions)
cacheFolder = string(tempname);
mkdir(cacheFolder);
cleanup = onCleanup(@() removeTemporaryCache(cacheFolder));

options = baseOptions;
options.numSamples = 48;
options.seed = 23;
options.cacheEnabled = true;
options.cacheFolder = cacheFolder;

reachable = workspaceAnalysis(robot,"reachable",options);
manipulability = workspaceAnalysis(robot,"manipulability",options);

changedPresentation = options;
changedPresentation.voxelSize = options.voxelSize/2;
changedPresentation.histogramBinWidthDeg = 15;
changedPresentation.envelopeOpacity = 0.35;
changedPresentation.envelopeAlphaRadius_m = 0.12;
changedPresentation.manipulabilityThreshold = min( ...
    1,options.manipulabilityThreshold+0.1);
usable = workspaceAnalysis(robot,"usable",changedPresentation);

changedCollisionRobot = robot;
changedCollisionRobot.collision.minimumClearance = ...
    robot.collision.minimumClearance+0.001;
changedCollision = workspaceAnalysis( ...
    changedCollisionRobot,"collision_free",options);

params = robot.params;
span = diff(params.joints.positionLimits(1,:));
params.joints.positionLimits(1,2) = ...
    params.joints.positionLimits(1,2)-0.01*span;
changedRobot = loadRobot(robot.id,params);
invalidated = workspaceAnalysis(changedRobot,"reachable",options);

result.firstRunBuildsCore = reachable.cache.coreBuilt && ...
    ~reachable.cache.coreHit;
result.manipulabilityReusesCore = manipulability.cache.coreHit && ...
    manipulability.cache.collisionBuilt && ...
    ~manipulability.cache.collisionHit && ...
    manipulability.cache.manipulabilityBuilt && ...
    ~manipulability.cache.manipulabilityHit;
result.presentationChangesReuseAll = usable.cache.coreHit && ...
    usable.cache.collisionHit && ...
    usable.cache.manipulabilityHit && ...
    isequal(manipulability.samples.q,usable.samples.q) && ...
    isequal(manipulability.manipulability.values, ...
        usable.manipulability.values);
result.collisionChangeReusesCoreOnly = changedCollision.cache.coreHit && ...
    changedCollision.cache.collisionBuilt && ...
    ~changedCollision.cache.collisionHit;
result.robotChangeInvalidates = invalidated.cache.coreBuilt && ...
    ~invalidated.cache.coreHit;
result.meshContentInvalidatesCollision = true;
if isfield(robot.collision,'meshFiles') && ~isempty(robot.collision.meshFiles)
    meshCopy = fullfile(cacheFolder,'collision_signature_probe.stl');
    copyfile(char(robot.collision.meshFiles(1)),meshCopy);
    meshRobot = robot;
    meshRobot.collision.meshFiles(1) = string(meshCopy);
    workspaceAnalysis(meshRobot,"collision_free",options);
    fid = fopen(meshCopy,'ab');
    assert(fid >= 0,'Could not open temporary mesh signature probe.');
    fwrite(fid,uint8(17),'uint8');
    fclose(fid);
    changedMesh = workspaceAnalysis(meshRobot,"collision_free",options);
    result.meshContentInvalidatesCollision = changedMesh.cache.coreHit && ...
        changedMesh.cache.collisionBuilt && ~changedMesh.cache.collisionHit;
end
result.success = result.firstRunBuildsCore && ...
    result.manipulabilityReusesCore && ...
    result.presentationChangesReuseAll && ...
    result.collisionChangeReusesCoreOnly && ...
    result.robotChangeInvalidates && ...
    result.meshContentInvalidatesCollision;

fprintf('Workspace staged cache and invalidation valid: %d\n',result.success);
end

function removeTemporaryCache(folder)
if isfolder(folder)
    rmdir(folder,'s');
end
end

function result = validateManipulability(robot,reachable,workspace)
values = workspace.manipulability.values;
indices = unique(round(linspace(1,numel(values),min(7,numel(values)))));
expected = zeros(numel(indices),1);
endEffector = char(robot.structure.frames.endEffector);

for k = 1:numel(indices)
    i = indices(k);
    jacobian = geometricJacobian(robot.model,workspace.samples.q(i,:),endEffector);
    sigma = svd(jacobian(4:6,:),'econ');
    if sigma(1) <= eps(max(1,sigma(1)))
        expected(k) = 0;
    else
        expected(k) = sigma(end)/sigma(1);
    end
end

result.sameSamples = isequal(reachable.samples.q,workspace.samples.q) && ...
    isequal(reachable.samples.points,workspace.samples.points);
result.statusEvaluated = workspace.manipulability.status == "EVALUATED";
result.finite = all(isfinite(values));
result.bounded = all(values >= 0 & values <= 1);
result.maxIndependentError = max(abs(values(indices)-expected));
result.voxelDataValid = ...
    size(workspace.manipulability.voxels.centers,2) == 3 && ...
    all(isfinite(workspace.manipulability.voxels.max));
result.success = result.sameSamples && result.statusEvaluated && ...
    result.finite && result.bounded && ...
    result.maxIndependentError < 1e-12 && result.voxelDataValid;

fprintf('Inverse-condition manipulability valid: %d\n',result.success);
end

function result = validateUsableWorkspace(manipulability,usable,options)
expectedMask = usable.masks.collisionFree & ...
    usable.manipulability.values >= options.manipulabilityThreshold;

result.sameSamples = ...
    isequal(manipulability.samples.q,usable.samples.q) && ...
    isequal(manipulability.samples.points,usable.samples.points) && ...
    isequal(manipulability.manipulability.values, ...
        usable.manipulability.values);
result.maskExact = isequal(usable.masks.accepted,expectedMask);
result.acceptedDataExact = ...
    isequal(usable.accepted.sampleIndices,find(expectedMask)) && ...
    isequal(usable.accepted.q,usable.samples.q(expectedMask,:)) && ...
    isequal(usable.accepted.points,usable.samples.points(expectedMask,:));
result.collisionStatusHonest = ...
    usable.collision.status == "EVALUATED" && ...
    usable.collision.filterApplied && ...
    ~any(usable.masks.accepted & ~usable.masks.collisionFree);
result.success = result.sameSamples && result.maskExact && ...
    result.acceptedDataExact && result.collisionStatusHonest;

fprintf('Usable-workspace threshold filtering valid: %d\n',result.success);
end

function result = validateCollisionFiltering(reachable,collisionWorkspace)
result.sameSamples = isequal(reachable.samples.q,collisionWorkspace.samples.q) && ...
    isequal(reachable.samples.points,collisionWorkspace.samples.points);
expectedMask = collisionWorkspace.collision.isCollisionFree;
result.evaluated = collisionWorkspace.collision.status == "EVALUATED" && ...
    collisionWorkspace.collision.filterApplied;
result.maskExact = isequal(collisionWorkspace.masks.accepted,expectedMask) && ...
    isequal(collisionWorkspace.masks.collisionFree,expectedMask);
result.rejectionsRetained = isequal( ...
    collisionWorkspace.collision.rejectedSampleIndices,find(~expectedMask)) && ...
    isequal(collisionWorkspace.collision.rejectedConfigurations, ...
        collisionWorkspace.samples.q(~expectedMask,:));
result.success = result.sameSamples && result.evaluated && ...
    result.maskExact && result.rejectionsRetained;
fprintf('Collision workspace filtering valid: %d\n',result.success);
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

result.min = abs(workspace.radius.min-min(radius)) < 1e-12;
result.max = abs(workspace.radius.max-max(radius)) < 1e-12;
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

if ~isfield(robot.params,'geometry')
    result.skipped = true;
    result.reason = "URDF geometry is changed by editing/re-exporting the robot-specific URDF.";
    result.success = true;
    fprintf('Workspace geometry propagation skipped for URDF-backed model.\n');
    return;
end

params = robot.params;
params.geometry.l2 = params.geometry.l2 + 0.05;
robotB = loadRobot(robot.id,params);

workspaceA = workspaceAnalysis(robot,"reachable",options);
workspaceB = workspaceAnalysis(robotB,"reachable",options);

deltaZ = workspaceB.points(:,3) - workspaceA.points(:,3);

result.expectedDeltaZ = 0.05;
result.maxDeltaError = max(abs(deltaZ - result.expectedDeltaZ));
result.success = result.maxDeltaError < 1e-10;

fprintf('Workspace geometry propagation valid: %d\n',result.success);

end

function result = validateWorkspacePlotSmoke(robot)

options = struct();
options.numSamples = 25;
options.seed = 1;
options.showPlot = true;
options.saveFigure = false;
options.cacheEnabled = false;
options.manipulabilityThreshold = 0.40;

oldVisibility = get(0,'DefaultFigureVisible');
set(0,'DefaultFigureVisible','off');
cleanup = onCleanup(@() set(0,'DefaultFigureVisible',oldVisibility));

try
    analysisTypes = ["reachable","collision_free","manipulability","usable"];
    valid = false(size(analysisTypes));
    interactiveValid = false(size(analysisTypes));
    inspectionValid = false(size(analysisTypes));
    histogramValid = true;
    envelopeValid = true;
    for i = 1:numel(analysisTypes)
        workspace = workspaceAnalysis(robot,analysisTypes(i),options);
        valid(i) = isfield(workspace.figures,'main') && ...
            isfield(workspace.figures,'pointCloud') && ...
            isvalid(workspace.figures.main) && ...
            isvalid(workspace.figures.pointCloud);
        slider = workspace.figures.jointSliders(1);
        target = min(slider.Max,slider.Value+deg2rad(5));
        slider.Value = target;
        drawnow;
        movedConfiguration = getappdata( ...
            workspace.figures.main,'WorkspaceRobotConfiguration');
        homeCallback = workspace.figures.homeButton.Callback;
        homeCallback(workspace.figures.homeButton,[]);
        drawnow;
        homeConfiguration = getappdata( ...
            workspace.figures.main,'WorkspaceRobotConfiguration');
        interactiveValid(i) = isgraphics(workspace.figures.robotAxes) && ...
            numel(workspace.figures.jointSliders) == robot.structure.dof && ...
            abs(movedConfiguration(1)-target) < 1e-12 && ...
            norm(homeConfiguration-robot.params.joints.homePosition) < 1e-12;
        inspector = workspace.figures.inspector;
        if analysisTypes(i) == "reachable"
            inspector.selectPlotItem(1,1);
            expectedSample = 1;
            mapped = inspector.getState().clickedSampleIndex == expectedSample;
        elseif analysisTypes(i) == "collision_free"
            mapped = true;
            if any(~workspace.masks.collisionFree)
                inspector.selectPlotItem(1,1);
                mapped = mapped && inspector.getState().clickedSampleIndex == ...
                    find(~workspace.masks.collisionFree,1);
                inspector.selectCandidate(find(arrayfun(@(c) ...
                    c.sampleIndex == find(~workspace.masks.collisionFree,1), ...
                    inspector.getState().candidates),1));
                inspector.reach();
                mapped = mapped && contains( ...
                    inspector.getState().message,"Colliding pose shown");
            end
            if any(workspace.masks.collisionFree)
                inspector.selectPlotItem(2,1);
                mapped = mapped && inspector.getState().clickedSampleIndex == ...
                    find(workspace.masks.collisionFree,1);
            end
        else
            voxelIndices = workspace.manipulability.voxels.indices;
            if analysisTypes(i) == "usable"
                keep = workspace.manipulability.voxels.max >= ...
                    options.manipulabilityThreshold;
                voxelIndices = voxelIndices(keep,:);
            end
            if isempty(voxelIndices)
                mapped = true;
                inspector.selectSample(1);
            else
                inspector.selectPlotItem(1,1);
                mapped = isequal(inspector.getState().selectedVoxel, ...
                    voxelIndices(1,:));
            end
        end
        state = inspector.getState();
        grouped = all(arrayfun(@(c) c.source ~= "sampled" || ...
            isequal(workspace.samples.voxelCoordinates(c.sampleIndex,:), ...
                state.selectedVoxel),state.candidates));
        eligible = arrayfun(@(c) c.collisionFree == 1 && ...
            (analysisTypes(i) ~= "usable" || ...
                c.icn >= options.manipulabilityThreshold),state.candidates);
        ranked = ~any(eligible) || ...
            abs(state.candidates(state.selectedIndex).icn - ...
                max([state.candidates(eligible).icn])) < 1e-12;
        inspector.reach();
        state = inspector.getState();
        reached = max(abs(state.robotConfiguration- ...
            state.candidates(state.selectedIndex).q)) < 1e-12;
        inspectionValid(i) = mapped && grouped && ranked && reached;
        if analysisTypes(i)=="usable"
            histogramValid = histogramValid && validateJointHistograms(workspace,robot);
            envelopeValid = envelopeValid && validateEnvelope(workspace);
        else
            histogramValid = histogramValid && isempty(workspace.figures.jointHistogramsButton);
            envelopeValid = envelopeValid && isempty(fieldnames(workspace.figures.envelope));
        end
        close(workspace.figures.main);
    end
    multiOptions = options;
    multiOptions.voxelSize = 5;
    workspace = workspaceAnalysis(robot,"reachable",multiOptions);
    [voxels,~,groups] = unique(workspace.samples.voxelCoordinates,'rows');
    [largestCount,group] = max(accumarray(groups,1));
    workspace.figures.inspector.selectVoxel(voxels(group,:));
    multiState = workspace.figures.inspector.getState();
    multiVoxelValid = largestCount > 1 && ...
        numel(multiState.candidates) == largestCount;
    originalCount = numel(multiState.candidates);
    originalPoints = workspace.samples.points;
    workspace.figures.inspector.findAlternatives();
    ikState = workspace.figures.inspector.getState();
    target = multiState.candidates(multiState.selectedIndex).transform;
    ikValid = numel(ikState.candidates) >= originalCount && ...
        isequal(workspace.samples.points,originalPoints);
    for candidate = ikState.candidates
        if candidate.source == "new IK"
            ikValid = ikValid && ...
                norm(candidate.transform(1:3,4)-target(1:3,4)) <= 1e-5 && ...
                norm(candidate.transform(1:3,1:3)-target(1:3,1:3),'fro') < 1e-3;
        end
    end
    close(workspace.figures.main);
    multiOptions.manipulabilityThreshold = 0;
    workspace = workspaceAnalysis(robot,"usable",multiOptions);
    histogramValid = histogramValid && validateJointHistograms(workspace,robot);
    envelopeValid = envelopeValid && validateEnvelope(workspace);
    close(workspace.figures.main);
    multiOptions.manipulabilityThreshold = 1;
    workspace = workspaceAnalysis(robot,"usable",multiOptions);
    histogramValid = histogramValid && isempty(workspace.accepted.q) && ...
        validateJointHistograms(workspace,robot);
    envelopeValid = envelopeValid && validateEnvelope(workspace);
    close(workspace.figures.main);
    result.analysisTypes = analysisTypes;
    result.valid = valid;
    result.interactiveValid = interactiveValid;
    result.inspectionValid = inspectionValid;
    result.multiVoxelValid = multiVoxelValid;
    result.ikValid = ikValid;
    result.histogramValid = histogramValid;
    result.envelopeValid = envelopeValid;
    result.success = all(valid) && all(interactiveValid) && ...
        all(inspectionValid) && multiVoxelValid && ikValid && histogramValid && envelopeValid;
    result.message = "All workspace plot paths executed.";
catch ME
    close all;
    result.success = false;
    result.message = string(ME.message);
end

fprintf('Workspace plot smoke valid: %d\n',result.success);

end

function pass = validateEnvelope(workspace)
controls = workspace.figures.envelope;
before = controls.getState();
controls.selector.Value = 2;
controls.selector.Callback(controls.selector,[]);
state = controls.getState();
surface = controls.surface;
markers = findobj(surface.Parent,'Type','scatter');
p = unique(workspace.accepted.points,'rows');
pass = before.status=="NOT_BUILT" && state.visualApproximationOnly && ...
    state.inputSampleCount==size(workspace.accepted.q,1) && ...
    strcmp(surface.PickableParts,'none') && strcmp(surface.HitTest,'off');
if size(p,1)>=4 && rank(p-mean(p,1))==3
    reference = alphaShape(p,'HoleThreshold',0,'RegionThreshold',0);
    pass = pass && state.status=="READY" && state.faceCount>0 && ...
        abs(state.alphaRadius_m-reference.Alpha)<1e-12 && ...
        isfinite(state.alphaRadius_m) && strcmp(surface.Visible,'on') && ...
        all(ismember(surface.Vertices,p,'rows')) && ...
        all(arrayfun(@(h) strcmp(h.Visible,'off'),markers));
    controls.selector.Value=3;
    controls.selector.Callback(controls.selector,[]);
    pass = pass && strcmp(surface.Visible,'on') && ...
        all(arrayfun(@(h) strcmp(h.Visible,'on'),markers));
else
    pass = pass && state.status=="INSUFFICIENT_3D_SAMPLES" && ...
        controls.selector.Value==1 && strcmp(surface.Visible,'off');
end
controls.selector.Value=1;
controls.selector.Callback(controls.selector,[]);
pass = pass && strcmp(surface.Visible,'off') && ...
    all(arrayfun(@(h) strcmp(h.Visible,'on'),markers));
end

function pass = validateJointHistograms(workspace,robot)
button = workspace.figures.jointHistogramsButton;
button.Callback(button,[]);
fig = getappdata(workspace.figures.main,'WorkspaceJointHistograms');
cleanup = onCleanup(@() close(fig));
plots = findobj(fig,'Type','histogram');
selected = getappdata(fig,'HistogramSampleIndices');
pass = numel(plots)==robot.structure.dof && ...
    isequal(selected,find(workspace.masks.accepted));
pass = pass && checkHistogramCounts(plots,workspace.samples.q(selected,:),robot);
mode = findobj(fig,'Tag','HistogramPopulation');
mode.Value = 2;
mode.Callback(mode,[]);
selected = getappdata(fig,'HistogramSampleIndices');
voxels = workspace.samples.voxelCoordinates;
expected = unique(voxels(workspace.masks.accepted,:),'rows');
pass = pass && isequal(sortrows(voxels(selected,:)),expected) && ...
    all(workspace.masks.accepted(selected)) && ...
    checkHistogramCounts(plots,workspace.samples.q(selected,:),robot);
for index = selected.'
    group = all(voxels==voxels(index,:),2) & workspace.masks.accepted;
    pass = pass && workspace.manipulability.values(index)== ...
        max(workspace.manipulability.values(group));
end
button.Callback(button,[]);
pass = pass && isequal(fig,getappdata(workspace.figures.main,'WorkspaceJointHistograms'));
mode.Value = 1;
mode.Callback(mode,[]);
pass = pass && isequal(getappdata(fig,'HistogramSampleIndices'), ...
    find(workspace.masks.accepted));
end

function pass = checkHistogramCounts(plots,q,robot)
pass = true;
for h = plots.'
    expected = histcounts(h.Data,h.BinEdges);
    joint = find(robot.structure.jointNames==string(h.Parent.Title.String));
    pass = pass && isscalar(joint) && sum(h.Values)==size(q,1) && ...
        isequal(h.Values,expected) && ...
        isequal(h.Data(:),rad2deg(q(:,joint)));
end
end
