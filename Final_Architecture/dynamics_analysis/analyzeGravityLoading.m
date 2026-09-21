function results = analyzeGravityLoading(robot,options)
% ANALYZEGRAVITYLOADING Evaluate explicit and/or sampled configurations.
% options.gravity is required. options.q is N-by-DOF; numSamples/seed optional.
if nargin < 2 || ~isstruct(options) || ~isfield(options,'gravity')
    error('dynamics:MissingGravity','options.gravity is required.');
end
if ~isfield(options,'includeCandidatePoses')
    options.includeCandidatePoses = true;
end
if ~islogical(options.includeCandidatePoses) || ~isscalar(options.includeCandidatePoses)
    error('dynamics:InvalidCandidateOption', ...
        'includeCandidatePoses must be a logical scalar.');
end
[model,validation] = prepareDynamicsModel(robot,options.gravity);
dof = robot.structure.dof;
q = zeros(0,dof);
if isfield(options,'q') && ~isempty(options.q)
    q = options.q;
end
poseNames = "user_" + string((1:size(q,1)).');
candidatePoseCount = 0;
if options.includeCandidatePoses && isfield(robot,'gravityPoses') && ...
        ~isempty(robot.gravityPoses)
    candidates = robot.gravityPoses;
    if ~isequal(size(candidates.q,2),dof) || ...
            numel(candidates.names) ~= size(candidates.q,1)
        error('dynamics:InvalidCandidatePoses', ...
            'Robot gravity poses must contain matching q and names.');
    end
    candidatePoseCount = size(candidates.q,1);
    q = [q; candidates.q];
    poseNames = [poseNames; string(candidates.names(:))];
end
if isfield(options,'numSamples')
    n = options.numSamples;
    if ~isnumeric(n) || ~isscalar(n) || ~isfinite(n) || n < 0 || n ~= floor(n)
        error('dynamics:InvalidSampleCount','numSamples must be a nonnegative integer.');
    end
else
    n = 0;
end
if n > 0
    if ~isfield(options,'seed'), options.seed = 1; end
    prior = rng;
    restore = onCleanup(@() rng(prior));
    rng(options.seed,'twister');
    limits = robot.params.joints.positionLimits;
    qRandom = limits(:,1).' + rand(n,dof).*(limits(:,2)-limits(:,1)).';
    q = [q; qRandom];
    poseNames = [poseNames; "random_" + string((1:n).')];
end
if isempty(q)
    error('dynamics:NoConfigurations','Provide options.q or numSamples.');
end
validateConfigurations(q,robot);
tau = zeros(size(q));
for k = 1:size(q,1)
    tau(k,:) = gravityTorque(model,q(k,:));
end
[absolutePeak,index] = max(abs(tau),[],1);
results.meta = dynamicsMetadata(robot,model.Gravity,validation,"gravity sampling");
results.q = q;
results.poseNames = poseNames;
results.candidatePoseCount = candidatePoseCount;
results.options = options;
results.torque = tau;
results.summaryTable = table(string(robot.structure.jointNames(:)), ...
    absolutePeak.',max(0,max(tau,[],1)).', ...
    min(0,min(tau,[],1)).',index.', ...
    'VariableNames',{'Joint','MaxObservedAbsGravityTorque_Nm', ...
    'PositiveMaximum_Nm','NegativeMinimum_Nm','SampleIndexAtAbsMax'});
results.qAtMaxAbs = q(index,:);
results.sampledMaximumIsGlobalBound = false;
end
