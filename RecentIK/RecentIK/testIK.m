function result = testIK()
% TESTIK Round-trip the control IK against robot_ver2's imported URDF FK.

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(repoRoot,'Final_Architecture'));
robot = loadRobot("robot_ver2");
P = robotParams();
home = robot.params.joints.homePosition;
rng(0);
jointSets = [home; ...
    deg2rad([20 35 -50 40 25 -30]); ...
    [.2 -.3 .4 -.1 pi/2 .7]; ...
    [.2 -.3 .4 -.1 -pi/2 .7]; ...
    -pi+2*pi*rand(500,6)];
positionTolerance = 1e-5;
orientationTolerance = 1e-4;
maxPositionError = 0;
maxOrientationError = 0;
noSolution = 0;
badBranch = 0;
branchCount = 0;

for k = 1:size(jointSets,1)
    target = getTransform(robot.model,jointSets(k,:),'End_Effector');
    [~,directBranches] = ikSolver(target,jointSets(k,:),P);
    resultIK = robot.solveIK(target,struct('qSeed',jointSets(k,:)));
    if isempty(directBranches) || ~resultIK.valid
        noSolution = noSolution+1;
        continue;
    end
    branchCount = branchCount+size(directBranches,1);
    for j = 1:size(directBranches,1)
        q = directBranches(j,:);
        actual = getTransform(robot.model,q,'End_Effector');
        pe = norm(actual(1:3,4)-target(1:3,4));
        re = acos(max(-1,min(1,(trace(target(1:3,1:3).'*actual(1:3,1:3))-1)/2)));
        maxPositionError = max(maxPositionError,pe);
        maxOrientationError = max(maxOrientationError,re);
        if pe > positionTolerance || re > orientationTolerance || ...
                any(q < P.qmin-1e-9 | q > P.qmax+1e-9)
            badBranch = badBranch+1;
        end
    end
end

unreachable = eye(4);
unreachable(1:3,4) = [10;10;10];
unreachableResult = robot.solveIK(unreachable);
result.numTargets = size(jointSets,1);
result.numBranches = branchCount;
result.noSolution = noSolution;
result.badBranch = badBranch;
result.maxPositionError_m = maxPositionError;
result.maxOrientationError_rad = maxOrientationError;
result.unreachableRejected = ~unreachableResult.valid;
result.pass = noSolution==0 && badBranch==0 && result.unreachableRejected;
fprintf('RecentIK versus robot_ver2 URDF: %d poses, %d branches, %d missing, %d invalid.\n', ...
    result.numTargets,branchCount,noSolution,badBranch);
fprintf('Max FK errors: %.3g m, %.3g rad. Unreachable pose rejected: %d. Pass: %d\n', ...
    maxPositionError,maxOrientationError,result.unreachableRejected,result.pass);
assert(result.pass,'RecentIK:ValidationFailed','Closed-form IK failed URDF FK validation.');
end
