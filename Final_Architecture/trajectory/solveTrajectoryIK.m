function [q,report] = solveTrajectoryIK(robot,poses,qStart,options)
% SOLVETRAJECTORYIK Select a continuous branch across all Cartesian poses.
if nargin < 4, options = struct(); end
if ~isfield(options,'maxPositionError_m'), options.maxPositionError_m=2e-3; end
if ~isfield(options,'maxOrientationError_rad'), options.maxOrientationError_rad=5e-3; end
if ~isfield(options,'maxSpatialJointStepRad')
    options.maxSpatialJointStepRad=deg2rad(15);
end
n = size(poses,3);
dof = robot.structure.dof;
q = NaN(n,dof);
qStart = reshape(qStart,1,[]);
limits = robot.params.joints.positionLimits;
if numel(qStart)~=dof || any(~isfinite(qStart)) || ...
        any(qStart<limits(:,1).'-1e-10 | qStart>limits(:,2).'+1e-10)
    error('trajectory:InvalidStart','Start configuration is outside joint limits.');
end
report = struct('pass',false,'reason',"",'failedSample',NaN, ...
    'failedPose',[],'lastValidConfiguration',qStart, ...
    'maxPositionError_m',0,'maxOrientationError_rad',0, ...
    'maxJointStep_rad',0,'maxStepJoint',NaN,'maxStepSample',NaN, ...
    'candidateCounts',zeros(n,1));
candidates = cell(n,1);
costs = cell(n,1);
parents = cell(n,1);
for k = 1:n
    target = poses(:,:,k);
    if k==1 && poseWithinTolerance( ...
            forward_kinematics(robot,qStart),target,options)
        current = qStart;
    else
        ikOptions.qSeed = qStart;
        if isfield(options,'ikMaxIterations')
            ikOptions.maxIterations = options.ikMaxIterations;
        end
        solution = robot.solveIK(target,ikOptions);
        if ~solution.valid || isempty(solution.q)
            report.reason = "No IK configuration at spatial sample " + k + ".";
            report.failedSample = k;
            report.failedPose = target;
            if k>1
                [~,bestPrevious] = min(costs{k-1});
                report.lastValidConfiguration = candidates{k-1}(bestPrevious,:);
            end
            return;
        end
        current = solution.q;
    end
    valid = all(isfinite(current),2) & ...
        all(current>=limits(:,1).'-1e-10 & ...
            current<=limits(:,2).'+1e-10,2);
    for j = find(valid).'
        valid(j) = poseWithinTolerance( ...
            forward_kinematics(robot,current(j,:)),target,options);
    end
    current = current(valid,:);
    report.candidateCounts(k) = size(current,1);
    if isempty(current)
        report.reason = "All IK candidates failed limit or FK checks at spatial sample " + k + ".";
        report.failedSample = k;
        report.failedPose = target;
        if k>1
            [~,bestPrevious] = min(costs{k-1});
            report.lastValidConfiguration = candidates{k-1}(bestPrevious,:);
        end
        return;
    end
    candidates{k} = current;
    costs{k} = Inf(size(current,1),1);
    parents{k} = zeros(size(current,1),1);
    if k==1
        costs{k} = sum((current-qStart).^2,2);
    else
        previous = candidates{k-1};
        for j=1:size(current,1)
            delta = previous-current(j,:);
            allowed = all(abs(delta)<=options.maxSpatialJointStepRad,2);
            cost = costs{k-1}+sum(delta.^2,2);
            cost(~allowed) = Inf;
            [costs{k}(j),parents{k}(j)] = min(cost);
        end
        if all(~isfinite(costs{k}))
            [~,bestPrevious] = min(costs{k-1});
            report.lastValidConfiguration = previous(bestPrevious,:);
            report.reason = "No continuous IK branch at spatial sample " + k + ".";
            report.failedSample = k;
            report.failedPose = target;
            return;
        end
    end
end
[~,index] = min(costs{n});
for k=n:-1:1
    q(k,:) = candidates{k}(index,:);
    if k>1, index = parents{k}(index); end
end
report.lastValidConfiguration = q(end,:);
for k=1:n
    actual = forward_kinematics(robot,q(k,:));
    [pError,rError] = poseErrors(actual,poses(:,:,k));
    report.maxPositionError_m = max(report.maxPositionError_m,pError);
    report.maxOrientationError_rad = max(report.maxOrientationError_rad,rError);
    if k>1
        [step,joint] = max(abs(q(k,:)-q(k-1,:)));
        if step>report.maxJointStep_rad
            report.maxJointStep_rad=step;
            report.maxStepJoint=joint;
            report.maxStepSample=k;
        end
    end
end
report.pass = true;
end

function acceptable = poseWithinTolerance(actual,target,options)
[pError,rError] = poseErrors(actual,target);
acceptable = pError<=options.maxPositionError_m && ...
    rError<=options.maxOrientationError_rad;
end

function [positionError,orientationError] = poseErrors(actual,target)
positionError = norm(actual(1:3,4)-target(1:3,4));
R = target(1:3,1:3).'*actual(1:3,1:3);
orientationError = acos(max(-1,min(1,(trace(R)-1)/2)));
end
