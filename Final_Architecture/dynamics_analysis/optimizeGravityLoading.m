function results = optimizeGravityLoading(robot,options)
% OPTIMIZEGRAVITYLOADING Search separately for each joint's gravity extrema.
% Numerical multistart search improves sampled peaks; it is not a global proof.
if nargin < 2 || ~isstruct(options) || ~isfield(options,'gravity')
    error('dynamics:MissingGravity','options.gravity is required.');
end
if exist('fmincon','file') ~= 2
    error('dynamics:MissingOptimizationToolbox', ...
        'optimizeGravityLoading requires MATLAB Optimization Toolbox (fmincon).');
end
if ~isfield(options,'numSamples'), options.numSamples = 1000; end
if ~isfield(options,'numStarts'), options.numStarts = 4; end
if ~isfield(options,'seed'), options.seed = 1; end
if ~isfield(options,'includeCandidatePoses'), options.includeCandidatePoses = true; end
if ~isfield(options,'maxFunctionEvaluations'), options.maxFunctionEvaluations = 1000; end
if ~isscalar(options.numStarts) || ~isfinite(options.numStarts) || ...
        options.numStarts < 1 || options.numStarts ~= floor(options.numStarts)
    error('dynamics:InvalidStarts','numStarts must be a positive integer.');
end
if ~isscalar(options.maxFunctionEvaluations) || ...
        ~isfinite(options.maxFunctionEvaluations) || ...
        options.maxFunctionEvaluations < 1 || ...
        options.maxFunctionEvaluations ~= floor(options.maxFunctionEvaluations)
    error('dynamics:InvalidEvaluationLimit', ...
        'maxFunctionEvaluations must be a positive integer.');
end
[model,validation] = prepareDynamicsModel(robot,options.gravity);
sampleOptions = struct('gravity',options.gravity,'q', ...
    robot.params.joints.homePosition,'numSamples',options.numSamples, ...
    'seed',options.seed,'includeCandidatePoses',options.includeCandidatePoses);
if isfield(options,'q') && ~isempty(options.q)
    sampleOptions.q = options.q;
end
sampled = analyzeGravityLoading(robot,sampleOptions);
qSamples = sampled.q;
tauSamples = sampled.torque;
limits = robot.params.joints.positionLimits;
lower = limits(:,1).';
upper = limits(:,2).';
dof = robot.structure.dof;
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
        direction = 3 - 2*side; % +1 finds positive peak; -1 finds negative peak.
        [bestValue,bestIndex] = max(direction*tauSamples(:,j));
        bestQ = qSamples(bestIndex,:);
        seeds = selectStarts(qSamples,direction*tauSamples(:,j), ...
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
results.meta = dynamicsMetadata(robot,model.Gravity,validation,"multistart gravity search");
results.meta.searchMethod = "seeded random sampling plus bounded multistart fmincon";
results.meta.searchDomain = "joint limits only; collision and workpiece constraints are not included";
results.meta.globalMaximumCertified = false;
results.options = options;
results.sampled = sampled;
results.qAtMaxAbs = qAbs;
results.qAtPositiveMax = qPositive;
results.qAtNegativeMin = qNegative;
results.signedTorqueAtMaxAbs_Nm = signedPeak;
results.functionEvaluations = evaluationCount;
results.nonpositiveExitCount = nonpositiveExitCount;
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
