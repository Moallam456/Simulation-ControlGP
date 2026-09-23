function results = analyzeGravityLoading(robot,options)
% ANALYZEGRAVITYLOADING Search gravity peaks or evaluate stationary poses.
% "search" (default) requires Optimization Toolbox; "poses" does not.
if nargin < 2 || ~isstruct(options) || ~isfield(options,'gravity')
    error('dynamics:MissingGravity','options.gravity is required.');
end
if ~isfield(options,'mode'), options.mode = "search"; end
if ~isscalar(string(options.mode)) || ...
        ~any(string(options.mode) == ["search" "poses"])
    error('dynamics:InvalidGravityMode','mode must be "search" or "poses".');
end
mode = string(options.mode);
if ~isfield(options,'includeCandidatePoses')
    options.includeCandidatePoses = true;
end
if ~islogical(options.includeCandidatePoses) || ...
        ~isscalar(options.includeCandidatePoses)
    error('dynamics:InvalidCandidateOption', ...
        'includeCandidatePoses must be a logical scalar.');
end
if mode == "search" && exist('fmincon','file') ~= 2
    error('dynamics:MissingOptimizationToolbox', ...
        'Gravity peak search requires Optimization Toolbox (fmincon).');
end
if ~isfield(options,'numSamples')
    options.numSamples = 0;
    if mode == "search", options.numSamples = 1000; end
end
n = options.numSamples;
if ~isnumeric(n) || ~isscalar(n) || ~isfinite(n) || ...
        n < 0 || n ~= floor(n)
    error('dynamics:InvalidSampleCount','numSamples must be a nonnegative integer.');
end
if n > 0 && ~isfield(options,'seed'), options.seed = 1; end
[model,validation] = prepareDynamicsModel(robot,options.gravity);
dof = robot.structure.dof;
q = zeros(0,dof);
if isfield(options,'q') && ~isempty(options.q), q = options.q; end
if mode == "search" && isempty(q)
    q = robot.params.joints.homePosition(:).';
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
if n > 0
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
results.meta = dynamicsMetadata(robot,model.Gravity,validation, ...
    "stationary gravity poses");
results.options = options;
results.candidatePoseCount = candidatePoseCount;
if mode == "poses"
    results.q = q;
    results.poseNames = poseNames;
    results.torque = tau;
    return;
end

if ~isfield(options,'numStarts'), options.numStarts = 4; end
if ~isfield(options,'maxFunctionEvaluations')
    options.maxFunctionEvaluations = 1000;
end
if ~isnumeric(options.numStarts) || ~isscalar(options.numStarts) || ...
        ~isfinite(options.numStarts) || options.numStarts < 1 || ...
        options.numStarts ~= floor(options.numStarts)
    error('dynamics:InvalidStarts','numStarts must be a positive integer.');
end
if ~isnumeric(options.maxFunctionEvaluations) || ...
        ~isscalar(options.maxFunctionEvaluations) || ...
        ~isfinite(options.maxFunctionEvaluations) || ...
        options.maxFunctionEvaluations < 1 || ...
        options.maxFunctionEvaluations ~= floor(options.maxFunctionEvaluations)
    error('dynamics:InvalidEvaluationLimit', ...
        'maxFunctionEvaluations must be a positive integer.');
end
results.options = options;
results.meta.analysisType = "multistart gravity search";
results.meta.searchMethod = "seeded random sampling plus bounded multistart fmincon";
results.meta.searchDomain = ...
    "joint limits only; collision and workpiece constraints are not included";
results.meta.globalMaximumCertified = false;
results.numEvaluatedStartPoses = size(q,1);
limits = robot.params.joints.positionLimits;
lower = limits(:,1).';
upper = limits(:,2).';
settings = optimoptions('fmincon','Algorithm','sqp','Display','off', ...
    'MaxFunctionEvaluations',options.maxFunctionEvaluations, ...
    'OptimalityTolerance',1e-8,'StepTolerance',1e-10);
positive = zeros(1,dof);
negative = zeros(1,dof);
qPositive = zeros(dof,dof);
qNegative = zeros(dof,dof);
evaluationCount = zeros(dof,2);
nonpositiveExitCount = zeros(dof,2);
for j = 1:dof
    for side = 1:2
        direction = 3 - 2*side;
        [bestValue,bestIndex] = max(direction*tau(:,j));
        bestQ = q(bestIndex,:);
        seeds = selectStarts(q,direction*tau(:,j), ...
            options.numStarts,lower,upper);
        for s = 1:size(seeds,1)
            [candidate,~,exitflag,output] = fmincon( ...
                @(x) -direction*gravityComponent(model,x,j), ...
                seeds(s,:),[],[],[],[],lower,upper,[],settings);
            evaluationCount(j,side) = evaluationCount(j,side) + output.funcCount;
            if exitflag <= 0
                nonpositiveExitCount(j,side) = nonpositiveExitCount(j,side)+1;
            end
            value = direction*gravityComponent(model,candidate,j);
            if value > bestValue
                bestValue = value;
                bestQ = candidate(:).';
            end
        end
        if side == 1
            positive(j) = bestValue;
            qPositive(j,:) = bestQ;
        else
            negative(j) = -bestValue;
            qNegative(j,:) = bestQ;
        end
    end
end
choosePositive = abs(positive) >= abs(negative);
qAbs = qNegative;
qAbs(choosePositive,:) = qPositive(choosePositive,:);
signedPeak = negative;
signedPeak(choosePositive) = positive(choosePositive);
results.qAtMaxAbs = qAbs;
results.qAtPositiveMax = qPositive;
results.qAtNegativeMin = qNegative;
results.signedTorqueAtMaxAbs_Nm = signedPeak;
results.functionEvaluations = evaluationCount;
results.nonpositiveExitCount = nonpositiveExitCount;
results.q = qAbs;
results.poseNames = string(robot.structure.jointNames(:)) + " found gravity peak";
results.torque = zeros(dof,dof);
for j = 1:dof
    results.torque(j,:) = gravityTorque(model,qAbs(j,:));
end
results.summaryTable = table(string(robot.structure.jointNames(:)), ...
    max(abs(positive),abs(negative)).',max(0,positive).', ...
    min(0,negative).',signedPeak.', ...
    'VariableNames',{'Joint','MaxFoundAbsGravityTorque_Nm', ...
    'PositiveMaximumFound_Nm','NegativeMinimumFound_Nm', ...
    'SignedTorqueAtAbsMax_Nm'});
end

function torque = gravityComponent(model,q,jointIndex)
allTorque = gravityTorque(model,reshape(q,1,[]));
torque = allTorque(jointIndex);
end

function seeds = selectStarts(q,values,numStarts,lower,upper)
% Spread starts across joint-limit-normalized space when good samples cluster.
[~,order] = sort(values,'descend');
seeds = zeros(0,size(q,2));
span = upper-lower;
for k = 1:numel(order)
    candidate = q(order(k),:);
    if isempty(seeds) || all(vecnorm((seeds-candidate)./span,2,2) > 0.2)
        seeds(end+1,:) = candidate; %#ok<AGROW>
    end
    if size(seeds,1) == numStarts
        break;
    end
end
end
