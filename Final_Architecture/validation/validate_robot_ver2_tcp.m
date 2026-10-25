function report = validate_robot_ver2_tcp(cadReference)
% VALIDATE_ROBOT_VER2_TCP Test propagation separately from physical calibration.
% No arguments: test software and the user-supplied Joint-6-relative measurement.
% Optional independent CAD reference: q (1x6 rad), TBaseTCP (4x4, m), source.
% TBaseTCP must use the URDF base axes, not unconverted Inventor Y-up axes.
root = fileparts(fileparts(mfilename('fullpath')));
oldPath = path; oldRng = rng;
cleanup = onCleanup(@() restoreState(oldPath,oldRng));
addpath(genpath(root));
rng(37);
if nargin<1, cadReference = []; end
robot = loadRobot('robot_ver2');
% Independent test fixture transcribed from the user's CAD measurement,
% not read back from the configured URDF. It is not a hardware calibration.
measured = eye(4);
measured(1:3,4) = [-22.87;20.53;245.77]/1000;
report.measurement.source = "User Joint-6-relative CAD measurement, 2026-10-15";
report.measurement.transformError = norm(robot.params.tool.TFlangeTCP-measured,'fro');
report.measurement.pass = report.measurement.transformError<1e-12;
report.measurement.baseToTCPAtZero = forward_kinematics(robot,zeros(1,6));
report.measurement.zeroPoseCompositionError = norm(report.measurement.baseToTCPAtZero- ...
    getTransform(robot.model,zeros(1,6),'End_Effector')*measured,'fro');
report.controlLayoutPass = norm(reshape(robot.params.tool.control.T_flange_tcp,4,4).'- ...
    measured,'fro')<1e-12;

% Deliberately synthetic, nonzero translation AND rotation. Not a torch size.
T = eul2tform(deg2rad([30 -15 20]),'ZYX');
T(1:3,4) = [.031;-.042;.180];
overrides.tool.TFlangeTCP = T;
overrides.tool.version = "synthetic-regression-only";
probe = loadRobot('robot_ver2',overrides);
report.syntheticTransform = T;
report.maxFKError = 0;
report.maxJacobianError = 0;
report.ikPass = true;
report.massUnchanged = true;
report.dynamicsError = 0;
bare = copy(probe.model);
removeBody(bare,'tcp');
bare.Gravity = [0 0 -9.81];
model = copy(probe.model); model.Gravity = bare.Gravity;
qs = -2+4*rand(24,6);
h = 1e-6;
for k = 1:size(qs,1)
    q = qs(k,:);
    expected = getTransform(probe.model,q,'End_Effector')*T;
    actual = forward_kinematics(probe,q);
    report.maxFKError = max(report.maxFKError,norm(actual-expected,'fro'));
    J = geometricJacobian(probe.model,q,'tcp');
    finiteDifference = zeros(6,6);
    for j = 1:6
        plus = q; minus = q;
        plus(j)=plus(j)+h; minus(j)=minus(j)-h;
        dT = (forward_kinematics(probe,plus)-forward_kinematics(probe,minus))/(2*h);
        W = dT(1:3,1:3)*actual(1:3,1:3).';
        finiteDifference(:,j) = [W(3,2);W(1,3);W(2,1);dT(1:3,4)];
    end
    report.maxJacobianError = max(report.maxJacobianError, ...
        max(abs(J-finiteDifference),[],'all'));
    s = probe.solveIK(expected,struct('qSeed',q));
    report.ikPass = report.ikPass && s.valid;
    for j=1:size(s.q,1)
        achieved = forward_kinematics(probe,s.q(j,:));
        [pe,re] = poseErrors(achieved,expected);
        report.ikPass = report.ikPass && pe<1e-5 && re<1e-4;
    end
    report.dynamicsError = max([report.dynamicsError, ...
        norm(massMatrix(model,q)-massMatrix(bare,q),'fro'), ...
        norm(inverseDynamics(model,q,.1*ones(1,6),.2*ones(1,6))- ...
        inverseDynamics(bare,q,.1*ones(1,6),.2*ones(1,6)))]);
end
for name = robot.structure.bodyNames
    a = getBody(robot.model,char(name)); b = getBody(probe.model,char(name));
    report.massUnchanged = report.massUnchanged && a.Mass==b.Mass && ...
        isequal(a.CenterOfMass,b.CenterOfMass) && isequal(a.Inertia,b.Inertia) && ...
        numel(a.Visuals)==numel(b.Visuals) && numel(a.Collisions)==numel(b.Collisions);
end
body = getBody(probe.model,'tcp');
report.massUnchanged = report.massUnchanged && body.Mass==0 && ...
    all(body.Inertia==0) && isempty(body.Visuals) && isempty(body.Collisions);
collisionA = analyzeCollisions(robot,qs(1:4,:));
collisionB = analyzeCollisions(probe,qs(1:4,:));
report.collisionUnchanged = isequaln(collisionA.isSelfColliding,collisionB.isSelfColliding) && ...
    isequaln(collisionA.minimumClearance,collisionB.minimumClearance);

folder = tempname;
mkdir(folder);
cacheCleanup = onCleanup(@() removeTestCache(folder));
options = struct('numSamples',16,'showPlot',false,'saveFigure',false, ...
    'cacheEnabled',true,'cacheFolder',folder,'seed',37);
w1 = workspaceAnalysis(robot,'manipulability',options);
w2 = workspaceAnalysis(robot,'manipulability',options);
w3 = workspaceAnalysis(probe,'manipulability',options);
report.cache.initialReuse = w2.cache.coreHit && w2.cache.manipulabilityHit && w2.cache.collisionHit;
report.cache.offsetInvalidates = w3.cache.coreBuilt && w3.cache.manipulabilityBuilt && ...
    w3.cache.collisionBuilt && ~w3.cache.coreHit && ~w3.cache.manipulabilityHit && ...
    ~w3.cache.collisionHit && isequal(w1.samples.q,w3.samples.q) && ...
    max(abs(w1.points-w3.points),[],'all')>1e-3;
report.workspaceFKError = 0;
for k=1:size(w3.samples.q,1)
    expected = getTransform(probe.model,w3.samples.q(k,:),'End_Effector')*T;
    report.workspaceFKError = max(report.workspaceFKError, ...
        norm(w3.points(k,:).'-expected(1:3,4)));
end
changed = overrides;
changed.tool.TFlangeTCP(1:3,1:3) = eye(3);
rotated = loadRobot('robot_ver2',changed);
w4 = workspaceAnalysis(rotated,'manipulability',options);
report.cache.rotationInvalidates = w4.cache.coreBuilt && w4.cache.manipulabilityBuilt && ...
    max(abs(w4.samples.transforms-w3.samples.transforms),[],'all')>1e-3;
changed.tool.version = "synthetic-version-change";
versioned = loadRobot('robot_ver2',changed);
w5 = workspaceAnalysis(versioned,'manipulability',options);
report.cache.toolVersionInvalidates = w5.cache.coreBuilt && w5.cache.manipulabilityBuilt;
versioned.params.source.modelVersion = "synthetic-model-revision";
w6 = workspaceAnalysis(versioned,'manipulability',options);
report.cache.modelVersionInvalidates = w6.cache.coreBuilt && w6.cache.manipulabilityBuilt;

q = probe.dynamicsCases(2).startConfiguration;
start = forward_kinematics(probe,q);
testCase = probe.dynamicsCases(2);
testCase.startConfiguration = q;
testCase.path = struct('type',"line",'endPosition',start(1:3,4)+[0;.01;0], ...
    'orientationMode',"constant");
[trajectory,report.trajectory] = generateTrajectory(probe,testCase);
report.trajectory.numSamples = size(trajectory.q,1);
report.softwarePass = report.maxFKError<1e-10 && report.maxJacobianError<1e-6 && ...
    report.ikPass && report.massUnchanged && report.dynamicsError<1e-10 && ...
    report.collisionUnchanged && report.workspaceFKError<1e-10 && ...
    all(structfun(@(value) logical(value),report.cache)) && report.trajectory.pass && ...
    report.controlLayoutPass;

report.cad.provided = ~isempty(cadReference);
report.cad.pass = false;
report.cad.status = "MISSING_INDEPENDENT_MEASUREMENT";
if report.cad.provided
    if ~isstruct(cadReference) || ~all(isfield(cadReference,{'q','TBaseTCP','source'}))
        error('tcp:InvalidReference','CAD reference needs q, TBaseTCP, and source.');
    end
    measured = cadReference.TBaseTCP;
    if ~isequal(size(measured),[4 4]) || any(~isfinite(measured(:))) || ...
            norm(measured(4,:)-[0 0 0 1])>1e-10 || ...
            norm(measured(1:3,1:3).'*measured(1:3,1:3)-eye(3),'fro')>1e-8 || ...
            abs(det(measured(1:3,1:3))-1)>1e-8
        error('tcp:InvalidReference','CAD TBaseTCP must be a rigid transform.');
    end
    [report.cad.positionError_m,report.cad.orientationError_rad] = ...
        poseErrors(forward_kinematics(robot,cadReference.q),measured);
    report.cad.source = string(cadReference.source);
    report.cad.pass = robot.params.tool.status~="UNCONFIGURED" && ...
        report.cad.positionError_m<1e-4 && report.cad.orientationError_rad<1e-3;
    report.cad.status = "MEASUREMENT_CHECKED";
end
report.measurementValidated = report.softwarePass && report.measurement.pass && ...
    report.measurement.zeroPoseCompositionError<1e-12;
report.independentBasePoseValidated = report.softwarePass && report.cad.pass;
fprintf('TCP software propagation: %d; FK %.3g; Jacobian %.3g; dynamics %.3g.\n', ...
    report.softwarePass,report.maxFKError,report.maxJacobianError,report.dynamicsError);
fprintf('Supplied local CAD measurement matches: %d; additional base-frame measurement: %s.\n', ...
    report.measurementValidated,report.cad.status);
end

function [p,r] = poseErrors(actual,expected)
p = norm(actual(1:3,4)-expected(1:3,4));
R = expected(1:3,1:3).'*actual(1:3,1:3);
r = acos(max(-1,min(1,(trace(R)-1)/2)));
end

function removeTestCache(folder)
if isfolder(folder), rmdir(folder,'s'); end
end

function restoreState(oldPath,oldRng)
path(oldPath); rng(oldRng);
end
