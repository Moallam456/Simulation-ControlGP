function results = analyzeGravityLoading(robot,options)
% ANALYZEGRAVITYLOADING Evaluate explicit and/or sampled configurations.
% options.gravity is required. options.q is N-by-DOF; numSamples/seed optional.
if nargin < 2 || ~isstruct(options) || ~isfield(options,'gravity')
    error('dynamics:MissingGravity','options.gravity is required.');
end
[model,validation] = prepareDynamicsModel(robot,options.gravity);
dof = robot.structure.dof;
q = zeros(0,dof);
if isfield(options,'q') && ~isempty(options.q)
    q = options.q;
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
results.torque = tau;
results.summaryTable = table(string(robot.structure.jointNames(:)), ...
    absolutePeak.',max(0,max(tau,[],1)).', ...
    min(0,min(tau,[],1)).',index.', ...
    'VariableNames',{'Joint','MaxObservedAbsGravityTorque_Nm', ...
    'PositiveMaximum_Nm','NegativeMinimum_Nm','SampleIndexAtAbsMax'});
results.qAtMaxAbs = q(index,:);
results.sampledMaximumIsGlobalBound = false;
end
