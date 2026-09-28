function dataset = prepareWorkspaceDataset(robot,settings,requirements)
% PREPAREWORKSPACEDATASET Build or reuse expensive workspace sample data.
% FK data and manipulability data are cached in stages. Plotting, voxel
% grouping, thresholds, and analysis-specific masks are intentionally not.

arguments
    robot (1,1) struct
    settings (1,1) struct
    requirements (1,1) struct
end

needsManipulability = isfield(requirements,'manipulability') && ...
    logical(requirements.manipulability);
cacheEnabled = logical(settings.cacheEnabled);
cacheFile = cacheFileFor(robot,settings);
kinematicSignature = createKinematicSignature(robot);
samplingSignature = createSamplingSignature(settings);
manipulabilitySignature = createManipulabilitySignature(settings);

status.enabled = cacheEnabled;
status.file = string(cacheFile);
status.coreHit = false;
status.coreBuilt = false;
status.manipulabilityRequested = needsManipulability;
status.manipulabilityHit = false;
status.manipulabilityBuilt = false;
status.saved = false;

cacheData = struct();
cacheDirty = false;
if cacheEnabled && isfile(cacheFile)
    try
        loaded = load(cacheFile,'cacheData');
        if isfield(loaded,'cacheData')
            cacheData = loaded.cacheData;
        end
    catch ME
        warning('workspace:CacheReadFailed', ...
            'Could not read workspace cache; rebuilding it. %s',ME.message);
        cacheData = struct();
    end
end

if hasMatchingCore(cacheData,kinematicSignature,samplingSignature)
    status.coreHit = true;
    fprintf('Workspace dataset: reusing cached configurations and FK.\n');
else
    fprintf('Workspace dataset: building configurations and FK.\n');
    [qSamples,sampling] = sampleConfigurations(robot,settings);
    [transforms,points] = evaluateForwardKinematics(robot,qSamples);
    cacheData.schemaVersion = 1;
    cacheData.kinematicSignature = kinematicSignature;
    cacheData.samplingSignature = samplingSignature;
    cacheData.core.q = qSamples;
    cacheData.core.transforms = transforms;
    cacheData.core.points = points;
    cacheData.core.sampling = sampling;
    cacheData.manipulability = struct();
    cacheData.manipulabilitySignature = struct();
    status.coreBuilt = true;
    cacheDirty = true;
end

if needsManipulability
    if hasMatchingManipulability(cacheData,manipulabilitySignature)
        status.manipulabilityHit = true;
        fprintf('Workspace dataset: reusing cached manipulability.\n');
    else
        fprintf('Workspace dataset: calculating manipulability.\n');
        cacheData.manipulability = evaluateManipulability( ...
            robot,cacheData.core.q,settings);
        cacheData.manipulabilitySignature = manipulabilitySignature;
        status.manipulabilityBuilt = true;
        cacheDirty = true;
    end
end

if cacheEnabled && cacheDirty
    status.saved = saveCache(cacheFile,cacheData);
end

dataset.q = cacheData.core.q;
dataset.transforms = cacheData.core.transforms;
dataset.points = cacheData.core.points;
dataset.sampling = cacheData.core.sampling;
if needsManipulability
    dataset.manipulability = cacheData.manipulability;
else
    dataset.manipulability = struct();
end
dataset.cache = status;

end

function tf = hasMatchingCore(cacheData,kinematicSignature,samplingSignature)
tf = isstruct(cacheData) && ...
    isfield(cacheData,'schemaVersion') && cacheData.schemaVersion == 1 && ...
    isfield(cacheData,'kinematicSignature') && ...
    isequaln(cacheData.kinematicSignature,kinematicSignature) && ...
    isfield(cacheData,'samplingSignature') && ...
    isequaln(cacheData.samplingSignature,samplingSignature) && ...
    isfield(cacheData,'core') && ...
    all(isfield(cacheData.core,{'q','transforms','points','sampling'})) && ...
    isequal(size(cacheData.core.q), ...
        [samplingSignature.numSamples kinematicSignature.dof]) && ...
    isequal(size(cacheData.core.points),[samplingSignature.numSamples 3]) && ...
    isequal(size(cacheData.core.transforms,1),4) && ...
    isequal(size(cacheData.core.transforms,2),4) && ...
    size(cacheData.core.transforms,3) == samplingSignature.numSamples && ...
    all(isfinite(cacheData.core.q(:))) && ...
    all(isfinite(cacheData.core.points(:))) && ...
    all(isfinite(cacheData.core.transforms(:)));
end

function tf = hasMatchingManipulability(cacheData,signature)
tf = isfield(cacheData,'manipulabilitySignature') && ...
    isequaln(cacheData.manipulabilitySignature,signature) && ...
    isfield(cacheData,'manipulability') && ...
    all(isfield(cacheData.manipulability, ...
        {'values','conditionNumber','singularValues'})) && ...
    numel(cacheData.manipulability.values) == size(cacheData.core.q,1) && ...
    size(cacheData.manipulability.singularValues,1) == ...
        size(cacheData.core.q,1) && ...
    all(isfinite(cacheData.manipulability.values)) && ...
    all(cacheData.manipulability.values >= 0 & ...
        cacheData.manipulability.values <= 1);
end

function signature = createSamplingSignature(settings)
signature.numSamples = settings.numSamples;
signature.samplingMethod = settings.samplingMethod;
signature.seed = settings.seed;
signature.algorithmVersion = 1;
end

function signature = createManipulabilitySignature(settings)
signature.index = settings.manipulabilityIndex;
signature.motion = settings.manipulabilityMotion;
signature.algorithmVersion = 1;
end

function signature = createKinematicSignature(robot)
limits = robot.params.joints.positionLimits;
dof = robot.structure.dof;
lower = limits(:,1).';
span = limits(:,2).'-lower;

fractions = zeros(4,dof);
fractions(1,:) = 0.5;
fractions(2,:) = 0.1+0.8*mod((1:dof)*0.173+0.11,1);
fractions(3,:) = 0.1+0.8*mod((1:dof)*0.317+0.23,1);
fractions(4,:) = 0.1+0.8*mod((1:dof)*0.419+0.37,1);
probeConfigurations = lower+fractions.*span;

bodyNames = string(robot.model.BodyNames);
probeTransforms = zeros(4,4,numel(bodyNames),size(probeConfigurations,1));
for p = 1:size(probeConfigurations,1)
    for b = 1:numel(bodyNames)
        probeTransforms(:,:,b,p) = getTransform( ...
            robot.model,probeConfigurations(p,:),char(bodyNames(b)));
    end
end

signature.version = 1;
signature.robotID = string(robot.id);
signature.dof = dof;
signature.positionLimits = limits;
signature.bodyNames = bodyNames;
signature.jointNames = string(robot.structure.jointNames);
signature.jointAxes = robot.structure.jointAxes;
signature.baseFrame = string(robot.structure.frames.base);
signature.endEffectorFrame = string(robot.structure.frames.endEffector);
signature.probeConfigurations = probeConfigurations;
signature.probeTransforms = probeTransforms;
end

function filePath = cacheFileFor(robot,settings)
robotID = regexprep(lower(char(string(robot.id))),'[^a-z0-9_-]','_');
method = regexprep(lower(char(settings.samplingMethod)),'[^a-z0-9_-]','_');
fileName = sprintf('%s_%s_n%d_seed%d.mat', ...
    robotID,method,settings.numSamples,settings.seed);
filePath = fullfile(char(settings.cacheFolder),fileName);
end

function success = saveCache(filePath,cacheData)
success = false;
folder = fileparts(filePath);
try
    if ~isfolder(folder)
        mkdir(folder);
    end
    save(filePath,'cacheData','-v7.3');
    success = true;
catch ME
    warning('workspace:CacheWriteFailed', ...
        'Workspace calculation succeeded, but its cache was not saved. %s', ...
        ME.message);
end
end

function [qSamples,metadata] = sampleConfigurations(robot,settings)
limits = robot.params.joints.positionLimits;
dof = robot.structure.dof;
span = limits(:,2).'-limits(:,1).';
lower = limits(:,1).';

metadata.requestedMethod = settings.samplingMethod;
metadata.actualMethod = settings.samplingMethod;
metadata.seed = settings.seed;
metadata.numSamples = settings.numSamples;

if settings.samplingMethod == "sobol" && exist('sobolset','file') == 2
    previousState = rng;
    cleanup = onCleanup(@() rng(previousState));
    rng(settings.seed,'twister');
    sequence = sobolset(dof,'Skip',settings.seed*1024);
    sequence = scramble(sequence,'MatousekAffineOwen');
    unitSamples = net(sequence,settings.numSamples);
elseif settings.samplingMethod == "sobol"
    warning('workspace:SobolUnavailable', ...
        'sobolset is unavailable; using seeded random sampling.');
    metadata.actualMethod = "random_fallback";
    unitSamples = seededRandom(settings.numSamples,dof,settings.seed);
else
    unitSamples = seededRandom(settings.numSamples,dof,settings.seed);
end

qSamples = lower+unitSamples.*span;
end

function values = seededRandom(rows,columns,seed)
previousState = rng;
cleanup = onCleanup(@() rng(previousState));
rng(seed,'twister');
values = rand(rows,columns);
end

function [transforms,points] = evaluateForwardKinematics(robot,qSamples)
sampleCount = size(qSamples,1);
transforms = zeros(4,4,sampleCount);
points = zeros(sampleCount,3);
for i = 1:sampleCount
    transforms(:,:,i) = forward_kinematics(robot,qSamples(i,:));
    points(i,:) = transforms(1:3,4,i).';
end
end

function result = evaluateManipulability(robot,qSamples,settings)
sampleCount = size(qSamples,1);
singularValueCount = min(3,robot.structure.dof);
values = zeros(sampleCount,1);
singularValues = zeros(sampleCount,singularValueCount);
endEffector = char(robot.structure.frames.endEffector);

for i = 1:sampleCount
    jacobian = geometricJacobian(robot.model,qSamples(i,:),endEffector);
    linearJacobian = jacobian(4:6,:);
    sigma = svd(linearJacobian,'econ');
    sigma = sigma(1:singularValueCount);
    singularValues(i,:) = sigma.';
    if isempty(sigma) || sigma(1) <= eps(max(1,sigma(1)))
        values(i) = 0;
    else
        values(i) = max(0,min(1,sigma(end)/sigma(1)));
    end
end

conditionNumber = inf(sampleCount,1);
nonzero = values > 0;
conditionNumber(nonzero) = 1./values(nonzero);
result.status = "EVALUATED";
result.index = settings.manipulabilityIndex;
result.motion = settings.manipulabilityMotion;
result.definition = "sigma_min(Jv) / sigma_max(Jv)";
result.values = values;
result.conditionNumber = conditionNumber;
result.singularValues = singularValues;
end
