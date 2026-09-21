function solutions = solveInverseKinematics(targetPose,robot,options)
% SOLVEINVERSEKINEMATICS Return validated configurations for one TCP pose.
if nargin < 3 || isempty(options), options = struct(); end
validateTarget(targetPose);
limits = robot.params.joints.positionLimits;
dof = robot.structure.dof;
if isfield(options,'qSeed') && ~isempty(options.qSeed)
    qSeed = reshape(options.qSeed,1,[]);
else
    qSeed = reshape(robot.params.joints.homePosition,1,[]);
end
if numel(qSeed) ~= dof || any(~isfinite(qSeed))
    error('initial_trial:InvalidIKSeed','qSeed must have one finite angle per joint.');
end
qSeed = min(max(qSeed,limits(:,1).'),limits(:,2).');
positionTolerance = option(options,'positionTolerance',1e-5);
orientationTolerance = option(options,'orientationTolerance',1e-4);
requestedMethod = lower(string(option(options,'method',"auto")));
if ~ismember(requestedMethod,["auto","numerical"])
    error('initial_trial:InvalidIKMethod','method must be auto or numerical.');
end

[analytical,status] = deal([],"SKIPPED_BY_REQUEST");
if requestedMethod=="auto"
    [analytical,status] = analyticalIKFunction(robot);
end
method = "analytical";
iterations = 0;
if ~isempty(analytical)
    try
        raw = analytical(targetPose);
        candidates = expandRevoluteWraps(raw,limits);
    catch ME
        analytical = [];
        status = "EVALUATION_FAILED: " + string(ME.message);
    end
end
if isempty(analytical)
    method = "numerical_multistart";
    [candidates,iterations] = numericalCandidates(targetPose,robot,qSeed,options);
end
[q,positionErrors,orientationErrors] = filterCandidates( ...
    candidates,robot,targetPose,limits,positionTolerance,orientationTolerance);
if ~isempty(q)
    [~,order] = sort(vecnorm(q-qSeed,2,2));
    q = q(order,:);
    positionErrors = positionErrors(order);
    orientationErrors = orientationErrors(order);
end

solutions.q = q;
solutions.valid = ~isempty(q);
solutions.branch = strings(size(q,1),1);
solutions.info.success = solutions.valid;
solutions.info.numSolutions = size(q,1);
solutions.info.iterations = iterations;
solutions.info.method = method;
solutions.info.positionErrors = positionErrors;
solutions.info.orientationErrors = orientationErrors;
solutions.info.positionError = Inf;
solutions.info.orientationError = Inf;
if solutions.valid
    solutions.info.positionError = positionErrors(1);
    solutions.info.orientationError = orientationErrors(1);
    solutions.info.message = "Validated IK configurations found.";
else
    solutions.info.message = "No joint-limit-valid configuration reaches the target pose.";
end
solutions.info.analyticalStatus = status;
solutions.info.collisionStatus = "UNKNOWN: simplified collision shapes are not used to reject IK.";
end

function validateTarget(T)
if ~isequal(size(T),[4 4]) || any(~isfinite(T(:))) || ...
        norm(T(4,:)-[0 0 0 1])>1e-10 || ...
        norm(T(1:3,1:3).'*T(1:3,1:3)-eye(3),'fro')>1e-8 || ...
        det(T(1:3,1:3))<0
    error('initial_trial:InvalidTargetPose', ...
        'Target must be a finite, proper 4-by-4 rigid transform.');
end
end

function value = option(options,name,defaultValue)
if isfield(options,name) && ~isempty(options.(name))
    value = options.(name);
else
    value = defaultValue;
end
end

function expanded = expandRevoluteWraps(q,limits)
expanded = q;
for j = 1:size(q,2)
    next = zeros(0,size(q,2));
    for row = 1:size(expanded,1)
        low = ceil((limits(j,1)-expanded(row,j)-1e-10)/(2*pi));
        high = floor((limits(j,2)-expanded(row,j)+1e-10)/(2*pi));
        for winding = low:high
            candidate = expanded(row,:);
            candidate(j) = candidate(j)+2*pi*winding;
            next(end+1,:) = candidate; %#ok<AGROW>
        end
    end
    expanded = next;
end
end

function [q,iterations] = numericalCandidates(T,robot,qSeed,options)
ik = inverseKinematics('RigidBodyTree',robot.model);
ik.SolverParameters.AllowRandomRestart = false;
ik.SolverParameters.MaxIterations = option(options,'maxIterations',200);
ik.SolverParameters.SolutionTolerance = 1e-8;
limits = robot.params.joints.positionLimits;
dof = robot.structure.dof;
numStarts = option(options,'numStarts',16);
seed = option(options,'seed',1);
if ~isscalar(numStarts) || numStarts<1 || numStarts~=floor(numStarts)
    error('initial_trial:InvalidIKStarts','numStarts must be a positive integer.');
end
prior = rng;
restore = onCleanup(@() rng(prior)); %#ok<NASGU>
rng(seed,'twister');
seeds = [qSeed;robot.params.joints.homePosition; ...
    (limits(:,1).'+limits(:,2).')/2; ...
    limits(:,1).'+rand(numStarts,dof).*(limits(:,2)-limits(:,1)).'];
q = zeros(0,dof);
iterations = 0;
for i = 1:size(seeds,1)
    [candidate,info] = ik(char(robot.structure.frames.endEffector), ...
        T,ones(1,6),seeds(i,:));
    iterations = iterations+info.Iterations;
    if all(isfinite(candidate))
        q(end+1,:) = candidate; %#ok<AGROW>
    end
end
end

function [q,pError,rError] = filterCandidates( ...
        candidates,robot,target,limits,pTolerance,rTolerance)
q = zeros(0,robot.structure.dof);
pError = zeros(0,1);
rError = zeros(0,1);
for i = 1:size(candidates,1)
    candidate = candidates(i,:);
    if any(~isfinite(candidate)) || ...
            any(candidate<limits(:,1).'-1e-9 | candidate>limits(:,2).'+1e-9)
        continue;
    end
    achieved = forward_kinematics(robot,candidate);
    positionError = norm(achieved(1:3,4)-target(1:3,4));
    rotationError = target(1:3,1:3).'*achieved(1:3,1:3);
    orientationError = acos(max(-1,min(1,(trace(rotationError)-1)/2)));
    if positionError>pTolerance || orientationError>rTolerance
        continue;
    end
    if ~isempty(q) && any(vecnorm(q-candidate,2,2)<1e-6)
        continue;
    end
    q(end+1,:) = candidate; %#ok<AGROW>
    pError(end+1,1) = positionError; %#ok<AGROW>
    rError(end+1,1) = orientationError; %#ok<AGROW>
end
end
