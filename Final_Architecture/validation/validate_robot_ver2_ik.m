function report = validate_robot_ver2_ik()
% VALIDATE_ROBOT_VER2_IK FK, MATLAB IK, interface, and trajectory regression.
% Run without arguments. All project dependencies are in Final_Architecture.
% Timing includes the public robot.solveIK interface, not just trigonometry.
numRandom = 600;
numMatlabComparisons = 40;
numTimingTargets = 8;
root = fileparts(fileparts(mfilename('fullpath')));
oldPath = path;
oldRng = rng;
cleanup = onCleanup(@() restoreState(oldPath,oldRng));
addpath(genpath(root));
rng(73,'twister');
robot = loadRobot("robot_ver2");
home = robot.params.joints.homePosition;
limits = robot.params.joints.positionLimits;
frame = char(robot.structure.frames.endEffector);
pTolerance = 1e-5;
rTolerance = 1e-4;

% Keep boundary/singularity coverage separate from ordinary reachable poses.
ordinary = [home; limits(:,1).'+rand(numRandom,6).*(limits(:,2)-limits(:,1)).'];
edges = repmat([.2 -.3 .4 -.1 .5 .7],12,1);
for j = 1:6
    edges(2*j-1,j) = limits(j,1);
    edges(2*j,j) = limits(j,2);
end
wristAngles = [-pi/2 pi/2];
offsets = [0 -1e-8 1e-8 -1e-5 1e-5 -1e-3 1e-3];
singular = repmat([.2 -.3 .4 -.1 .5 .7],numel(wristAngles)*numel(offsets),1);
singular(:,5) = reshape(wristAngles(:)+offsets,[],1);
targetsQ = [ordinary; edges; singular];
n = size(targetsQ,1);
report.numTargets = n;
report.numBranches = 0;
report.missingTargets = zeros(0,1);
report.invalidBranches = 0;
report.contractPass = true;
report.maxPositionError_m = 0;
report.maxOrientationError_rad = 0;
report.maxGenericFKError = 0;
times = zeros(n,1);
targets = zeros(4,4,n);
solutions = cell(n,1);
fprintf('robot_ver2 IK: %d FK-generated targets; checking every returned branch.\n',n);
for k = 1:n
    targets(:,:,k) = getTransform(robot.model,targetsQ(k,:),frame);
    report.maxGenericFKError = max(report.maxGenericFKError, ...
        norm(forward_kinematics(robot,targetsQ(k,:))-targets(:,:,k),'fro'));
    t = tic;
    s = robot.solveIK(targets(:,:,k),struct('qSeed',home));
    times(k) = toc(t);
    solutions{k} = s;
    report.contractPass = report.contractPass && contractValid(s,home);
    if ~s.valid
        report.missingTargets(end+1,1) = k;
        continue;
    end
    report.numBranches = report.numBranches+size(s.q,1);
    for j = 1:size(s.q,1)
        [pe,re] = poseErrors(getTransform(robot.model,s.q(j,:),frame),targets(:,:,k));
        report.maxPositionError_m = max(report.maxPositionError_m,pe);
        report.maxOrientationError_rad = max(report.maxOrientationError_rad,re);
        valid = all(isfinite(s.q(j,:))) && ...
            all(s.q(j,:)>=limits(:,1).'-1e-9 & s.q(j,:)<=limits(:,2).'+1e-9) && ...
            pe<=pTolerance && re<=rTolerance;
        report.invalidBranches = report.invalidBranches+double(~valid);
    end
    if mod(k,100)==0, fprintf('  Checked %d/%d targets.\n',k,n); end
end
report.missingOrdinary = sum(report.missingTargets<=size(ordinary,1));
report.missingBoundary = sum(report.missingTargets>size(ordinary,1) & ...
    report.missingTargets<=size(ordinary,1)+size(edges,1));
report.missingNearSingular = numel(report.missingTargets)- ...
    report.missingOrdinary-report.missingBoundary;
report.missingConfigurations = targetsQ(report.missingTargets,:);
fprintf('FK stage: %d branches, %d missing, %d invalid; max %.9g m / %.9g rad.\n', ...
    report.numBranches,numel(report.missingTargets),report.invalidBranches, ...
    report.maxPositionError_m,report.maxOrientationError_rad);

% Independent numerical solving starts from home and perturbed configurations,
% never from an analytical answer. Both solvers are judged by the same FK test.
fprintf('Comparing with MATLAB inverseKinematics on %d targets.\n',numMatlabComparisons);
ik = inverseKinematics('RigidBodyTree',robot.model);
ik.SolverParameters.AllowRandomRestart = false;
ik.SolverParameters.MaxIterations = 300;
ik.SolverParameters.SolutionTolerance = 1e-9;
report.matlab = struct('numTargets',numMatlabComparisons,'solved',0, ...
    'analyticalSolved',0,'maxPositionError_m',0,'maxOrientationError_rad',0, ...
    'maxCrossPositionError_m',0,'maxCrossOrientationError_rad',0);
report.matlab.missedTargets = zeros(0,1);
singleTimes = zeros(numMatlabComparisons,1);
multiTimes = zeros(numTimingTargets,1);
multiSolved = false(numTimingTargets,1);
for k = 1:numMatlabComparisons
    index = k+1;
    T = targets(:,:,index);
    seeds = [home; max(limits(:,1).',min(limits(:,2).', ...
        targetsQ(index,:)+.15*randn(1,6))); ...
        limits(:,1).'+rand(2,6).*(limits(:,2)-limits(:,1)).'];
    found = false;
    for j = 1:size(seeds,1)
        t = tic;
        q = ik(frame,T,ones(1,6),seeds(j,:));
        elapsed = toc(t);
        if j==1, singleTimes(k) = elapsed; end
        [pe,re] = poseErrors(getTransform(robot.model,q,frame),T);
        if all(q>=limits(:,1).'-1e-9 & q<=limits(:,2).'+1e-9) && ...
                pe<=pTolerance && re<=rTolerance
            found = true;
            break;
        end
    end
    s = solutions{index};
    report.matlab.analyticalSolved = report.matlab.analyticalSolved+double(s.valid);
    if found
        report.matlab.solved = report.matlab.solved+1;
        report.matlab.maxPositionError_m = max(report.matlab.maxPositionError_m,pe);
        report.matlab.maxOrientationError_rad = max(report.matlab.maxOrientationError_rad,re);
        if s.valid
            [pe,re] = poseErrors(getTransform(robot.model,s.q(1,:),frame), ...
                getTransform(robot.model,q,frame));
            report.matlab.maxCrossPositionError_m = max(report.matlab.maxCrossPositionError_m,pe);
            report.matlab.maxCrossOrientationError_rad = max(report.matlab.maxCrossOrientationError_rad,re);
        end
    else
        report.matlab.missedTargets(end+1,1) = index;
    end
    if k<=numTimingTargets
        % Prior multistart workload: seed, home, midpoint, and 24 random starts.
        starts = [home; home; mean(limits,2).'; ...
            limits(:,1).'+rand(24,6).*(limits(:,2)-limits(:,1)).'];
        t = tic;
        for j = 1:size(starts,1)
            q = ik(frame,T,ones(1,6),starts(j,:));
            [pe,re] = poseErrors(getTransform(robot.model,q,frame),T);
            multiSolved(k) = multiSolved(k) || (pe<=pTolerance && re<=rTolerance && ...
                all(q>=limits(:,1).'-1e-9 & q<=limits(:,2).'+1e-9));
        end
        multiTimes(k) = toc(t);
    end
end
report.timing.analyticalMedian_s = median(times(2:numMatlabComparisons+1));
report.timing.matlabSingleStartMedian_s = median(singleTimes);
report.timing.matlab27StartsMedian_s = median(multiTimes);
report.timing.analyticalSameTargetsMedian_s = median(times(2:numTimingTargets+1));
report.timing.multistartSpeedRatio = median(multiTimes)/report.timing.analyticalSameTargetsMedian_s;
report.timing.multistartTargetsSolved = sum(multiSolved);

unreachable = eye(4); unreachable(1:3,4) = [10;10;10];
s = robot.solveIK(unreachable);
report.unreachableRejected = ~s.valid && isequal(size(s.q),[0 6]) && contractValid(s,home);
report.invalidInputRejected = false;
try
    robot.solveIK(zeros(4));
catch exception
    report.invalidInputRejected = strcmp(exception.identifier,'robot_ver2:InvalidTargetPose');
end
% Active limits must be taken from this robot instance, not nominal constants.
qLimited = [.2 -.3 .4 -.1 .5 .7];
% Retain the catalog's 135-degree start so loading its cases remains valid.
overrides.joints.positionLimits = repmat(deg2rad([-150 150]),6,1);
overrides.joints.homePosition = home;
limited = loadRobot('robot_ver2',overrides);
s = limited.solveIK(forward_kinematics(limited,qLimited));
report.limitOverridePass = s.valid && all(all( ...
    s.q>=overrides.joints.positionLimits(:,1).'-1e-9 & ...
    s.q<=overrides.joints.positionLimits(:,2).'+1e-9));

fprintf('Checking the existing THREE_LEG_SWEEP trajectory and rigid-body dynamics.\n');
testCase = robot.dynamicsCases(string({robot.dynamicsCases.id})=="THREE_LEG_SWEEP");
[trajectory,report.trajectory] = generateTrajectory(robot,testCase);
report.trajectory.numSamples = size(trajectory.q,1);
report.dynamicsFinite = report.trajectory.pass;
if report.trajectory.pass
    for k = 1:size(trajectory.q,1)
        torque = inverseDynamics(robot.model,trajectory.q(k,:),trajectory.qd(k,:),trajectory.qdd(k,:));
        report.dynamicsFinite = report.dynamicsFinite && all(isfinite(torque));
    end
end
report.otherRobots = struct();
otherRobots = cell(1,2);
robotIndex = 0;
for id = ["initial_trial" "robot_ver1"]
    other = loadRobot(id);
    robotIndex = robotIndex+1;
    otherRobots{robotIndex} = other;
    q = other.params.joints.homePosition;
    T = forward_kinematics(other,q);
    s = other.solveIK(T,struct('qSeed',q,'numStarts',2));
    valid = s.valid;
    if valid
        [pe,re] = poseErrors(forward_kinematics(other,s.q(1,:)),T);
        valid = pe<=pTolerance && re<=rTolerance;
    end
    report.otherRobots.(id) = valid;
end
% Reuse the original handle after loading other robot folders (path dispatch).
s = robot.solveIK(targets(:,:,2));
report.dispatchPass = s.valid && s.info.method=="analytical_closed_form";
for j = 1:numel(otherRobots)
    other = otherRobots{j};
    q = other.params.joints.homePosition;
    s = other.solveIK(forward_kinematics(other,q),struct('qSeed',q,'numStarts',2));
    report.dispatchPass = report.dispatchPass && s.valid && ...
        s.info.method~="analytical_closed_form";
    if other.id=="robot_ver1"
        report.dispatchPass = report.dispatchPass && s.info.method=="numerical_multistart";
    end
end
pathBeforeSolve = path;
robot.solveIK(targets(:,:,2));
report.dispatchPass = report.dispatchPass && strcmp(pathBeforeSolve,path);
report.pass = report.invalidBranches==0 && report.contractPass && ...
    isempty(report.missingTargets) && report.maxGenericFKError<1e-10 && ...
    report.matlab.solved==numMatlabComparisons && ...
    report.unreachableRejected && report.invalidInputRejected && report.limitOverridePass && ...
    report.trajectory.pass && report.dynamicsFinite && ...
    report.otherRobots.initial_trial && report.otherRobots.robot_ver1 && report.dispatchPass;
fprintf('FK: %d targets, %d branches, %d missing, %d invalid.\n', ...
    n,report.numBranches,numel(report.missingTargets),report.invalidBranches);
fprintf('Maximum analytical errors: %.9g m, %.9g rad.\n', ...
    report.maxPositionError_m,report.maxOrientationError_rad);
fprintf('MATLAB IK: %d/%d solved; analytical: %d/%d.\n', ...
    report.matlab.solved,numMatlabComparisons,report.matlab.analyticalSolved,numMatlabComparisons);
fprintf('Median time: analytical %.6g s, MATLAB single %.6g s, MATLAB 27 starts %.6g s.\n', ...
    report.timing.analyticalMedian_s,report.timing.matlabSingleStartMedian_s,report.timing.matlab27StartsMedian_s);
fprintf('THREE_LEG_SWEEP: %d (%d samples); finite dynamics: %d; overall pass: %d.\n', ...
    report.trajectory.pass,report.trajectory.numSamples,report.dynamicsFinite,report.pass);
if ~report.pass
    warning('robot_ver2:IKValidationFailed','Inspect the returned report for failed checks.');
end
end

function valid = contractValid(s,seed)
valid = all(isfield(s,{'q','valid','branch','info'})) && size(s.q,2)==6 && ...
    s.valid==~isempty(s.q) && s.info.success==s.valid && ...
    s.info.numSolutions==size(s.q,1) && s.info.iterations==0 && ...
    s.info.method=="analytical_closed_form" && ...
    numel(s.info.positionErrors)==size(s.q,1) && numel(s.info.orientationErrors)==size(s.q,1);
if ~valid || isempty(s.q), return; end
distances = sum(atan2(sin(s.q-seed),cos(s.q-seed)).^2,2);
valid = all(diff(distances)>=-1e-10) && all(s.branch=="analytical");
for j = 2:size(s.q,1)
    delta = atan2(sin(s.q(1:j-1,:)-s.q(j,:)),cos(s.q(1:j-1,:)-s.q(j,:)));
    valid = valid && all(vecnorm(delta,2,2)>=1e-6);
end
end

function [pe,re] = poseErrors(actual,target)
pe = norm(actual(1:3,4)-target(1:3,4));
R = target(1:3,1:3).'*actual(1:3,1:3);
re = acos(max(-1,min(1,(trace(R)-1)/2)));
end

function restoreState(oldPath,oldRng)
path(oldPath);
rng(oldRng);
end
