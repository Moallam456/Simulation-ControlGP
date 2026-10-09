function solutions = solveInverseKinematics(targetPose,robot,options)
% SOLVEINVERSEKINEMATICS Numerical multistart IK for robot_ver2.

if nargin < 3 || isempty(options)
    options = struct();
end
validateTarget(targetPose);

dof = robot.structure.dof;
limits = robot.params.joints.positionLimits;
qSeed = reshape(option(options,'qSeed',robot.params.joints.homePosition),1,[]);
if numel(qSeed) ~= dof || any(~isfinite(qSeed))
    error('robot_ver2:InvalidIKSeed', ...
        'qSeed must contain %d finite angles.',dof);
end
qSeed = min(max(qSeed,limits(:,1).'),limits(:,2).');
numStarts = option(options,'numStarts',24);
if ~isscalar(numStarts) || numStarts < 1 || numStarts ~= floor(numStarts)
    error('robot_ver2:InvalidIKStarts','numStarts must be a positive integer.');
end

prior = rng;
cleanup = onCleanup(@() rng(prior));
rng(option(options,'seed',1),'twister');
seeds = [qSeed; robot.params.joints.homePosition; ...
    0.5*(limits(:,1).'+limits(:,2).'); ...
    limits(:,1).'+rand(numStarts,dof).*(limits(:,2)-limits(:,1)).'];

ik = inverseKinematics('RigidBodyTree',robot.model);
ik.SolverParameters.AllowRandomRestart = false;
ik.SolverParameters.MaxIterations = option(options,'maxIterations',300);
ik.SolverParameters.SolutionTolerance = 1e-9;
weights = option(options,'weights',ones(1,6));
positionTolerance = option(options,'positionTolerance',1e-5);
orientationTolerance = option(options,'orientationTolerance',1e-4);

candidates = zeros(0,dof);
iterations = 0;
for i = 1:size(seeds,1)
    [candidate,info] = ik(char(robot.structure.frames.endEffector), ...
        targetPose,weights,seeds(i,:));
    iterations = iterations + info.Iterations;
    if all(isfinite(candidate))
        candidates(end+1,:) = candidate; %#ok<AGROW>
    end
end

[q,pError,rError] = validatedCandidates(candidates,robot,targetPose, ...
    limits,positionTolerance,orientationTolerance);
if ~isempty(q)
    [~,order] = sort(vecnorm(q-qSeed,2,2));
    q = q(order,:);
    pError = pError(order);
    rError = rError(order);
end

solutions.q = q;
solutions.valid = ~isempty(q);
solutions.branch = repmat("numerical",size(q,1),1);
solutions.info.success = solutions.valid;
solutions.info.numSolutions = size(q,1);
solutions.info.iterations = iterations;
solutions.info.method = "numerical_multistart";
solutions.info.positionErrors = pError;
solutions.info.orientationErrors = rError;
solutions.info.positionError = Inf;
solutions.info.orientationError = Inf;
if solutions.valid
    solutions.info.positionError = pError(1);
    solutions.info.orientationError = rError(1);
    solutions.info.message = "Validated numerical IK configurations found.";
else
    solutions.info.message = "No limit-valid configuration reaches the exact target pose.";
end
solutions.info.analyticalStatus = "NOT_CONFIGURED_FOR_ROBOT_VER2";
solutions.info.collisionStatus = ...
    "NOT_FILTERED: mesh collisions require a separate collision check.";

end

function validateTarget(T)
if ~isequal(size(T),[4 4]) || any(~isfinite(T(:))) || ...
        norm(T(4,:)-[0 0 0 1]) > 1e-10 || ...
        norm(T(1:3,1:3).'*T(1:3,1:3)-eye(3),'fro') > 1e-8 || ...
        det(T(1:3,1:3)) < 0
    error('robot_ver2:InvalidTargetPose', ...
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

function [q,pError,rError] = validatedCandidates( ...
        candidates,robot,target,limits,pTolerance,rTolerance)
q = zeros(0,robot.structure.dof);
pError = zeros(0,1);
rError = zeros(0,1);
for i = 1:size(candidates,1)
    candidate = candidates(i,:);
    if any(candidate < limits(:,1).'-1e-9 | candidate > limits(:,2).'+1e-9)
        continue;
    end
    achieved = getTransform(robot.model,candidate, ...
        char(robot.structure.frames.endEffector));
    pe = norm(achieved(1:3,4)-target(1:3,4));
    relativeRotation = target(1:3,1:3).'*achieved(1:3,1:3);
    re = acos(max(-1,min(1,(trace(relativeRotation)-1)/2)));
    if pe > pTolerance || re > rTolerance
        continue;
    end
    wrappedDifference = atan2(sin(q-candidate),cos(q-candidate));
    if ~isempty(q) && any(vecnorm(wrappedDifference,2,2) < 1e-6)
        continue;
    end
    q(end+1,:) = candidate; %#ok<AGROW>
    pError(end+1,1) = pe; %#ok<AGROW>
    rError(end+1,1) = re; %#ok<AGROW>
end
end
