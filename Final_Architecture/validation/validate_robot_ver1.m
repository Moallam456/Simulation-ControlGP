function report = validate_robot_ver1(robot)
% VALIDATE_ROBOT_VER1 Acceptance checks for the six-axis URDF-backed model.

if nargin < 1 || isempty(robot)
    robot = loadRobot("robot_ver1");
end
rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'kinematics'), ...
    fullfile(rootDir,'dynamics_analysis'), ...
    fullfile(rootDir,'actuator_sizing_cases'));
if string(robot.id) ~= "robot_ver1"
    error('validation:WrongRobot','Expected robot_ver1, received %s.',robot.id);
end

expectedBodies = ["Link_1" "Link_2" "Link_3" "Wrist_1" "Wrist_2" "End_Effector"];
expectedJoints = ["Joint_1" "Joint_2" "Joint_3" "Joint_4" "Joint_5" "Joint_6"];
report.topology = robot.structure.dof == 6 && ...
    string(robot.model.BaseName) == "Base_Link" && ...
    isequal(robot.structure.bodyNames,expectedBodies) && ...
    isequal(robot.structure.jointNames,expectedJoints) && ...
    string(robot.structure.frames.endEffector) == "End_Effector";

report.meshes = validateMeshes(robot);
report.massProperties = validateMassProperties(robot);
report.fk = validateFK(robot,30);
report.ik = validateIKRoundTrip(robot);
report.dynamics = validateDynamics(robot);
report.rendering = validateRendering(robot);

qHome = robot.params.joints.homePosition;
[isColliding,~] = checkCollision(robot.model,qHome, ...
    'SkippedSelfCollisions','parent','Exhaustive','on');
report.collision.homeCollisionDetected = any(isColliding(:));
report.collision.reportedPair = robot.collision.knownHomeCollision;
report.collision.status = "UNRESOLVED";
report.collision.note = robot.collision.approximation;

report.readiness = isfield(robot,'readiness') && ...
    ~robot.readiness.joint6Missing && ...
    robot.readiness.jointLimitsProvisional && ...
    robot.readiness.tcpProvisional && ...
    robot.readiness.massPropertiesPreliminary && ...
    ~robot.readiness.actuatorSizingAllowed;
report.actuatorSizingBlocked = sizingIsBlocked(robot);
report.initialTrialRegression = initialTrialLoads();

report.pass = report.topology && report.meshes.pass && ...
    report.massProperties.pass && report.fk.pass && report.ik.pass && ...
    report.dynamics.pass && report.rendering.pass && ...
    report.collision.homeCollisionDetected && report.readiness && ...
    report.actuatorSizingBlocked && report.initialTrialRegression;

fprintf('\nrobot_ver1 validation\n');
fprintf('  Topology and six-axis endpoint:  %d\n',report.topology);
fprintf('  URDF visual/collision meshes:    %d\n',report.meshes.pass);
fprintf('  CAD mass properties:             %d\n',report.massProperties.pass);
fprintf('  FK / IK / dynamics:              %d / %d / %d\n', ...
    report.fk.pass,report.ik.pass,report.dynamics.pass);
fprintf('  Known home collision:            %s <-> %s (%s)\n', ...
    report.collision.reportedPair(1),report.collision.reportedPair(2), ...
    report.collision.status);
fprintf('  Actuator sizing blocked:         %d\n',report.actuatorSizingBlocked);
fprintf('  Overall pass:                    %d\n',report.pass);

end

function result = validateMeshes(robot)
allBodies = [{robot.model.Base} robot.model.Bodies];
result.visualCounts = cellfun(@(body) numel(body.Visuals),allBodies);
result.collisionCounts = cellfun(@(body) numel(body.Collisions),allBodies);
result.filesExist = all(isfile(robot.collision.meshFiles));

document = xmlread(char(robot.params.source.urdf));
nodes = document.getElementsByTagName('mesh');
result.uris = strings(nodes.getLength,1);
result.urisValid = nodes.getLength == 14;
for i = 0:nodes.getLength-1
    uri = string(nodes.item(i).getAttribute('filename'));
    result.uris(i+1) = uri;
    result.urisValid = result.urisValid && startsWith(uri,"package://robot_ver1/meshes/");
end
result.pass = all(result.visualCounts == 1) && ...
    all(result.collisionCounts == 1) && result.filesExist && ...
    result.urisValid && validBinarySTL(robot.collision.meshFiles(end));
end

function result = validateMassProperties(robot)
expectedMass = [2.200 3.500 2.350 0.575 0.575 0.300];
expectedCOM = [
    -0.106451 -0.056736 0.101055148
     0.007556557 -0.323779148 -0.034318388
    -0.182291207 -0.065672927 0.003309351
    -0.066487521 0.010831776 -0.091501332
     0.069619270 -0.003019655 0.095127911
    -0.048946282 -0.059068151 -0.077557237];
expectedInertia = cat(3, ...
    symmetricTensor(0.018750698642421,0.014482698642421,0.024953768389588, ...
        -0.006955004483746,0.004506874542284,0.002532529082077), ...
    symmetricTensor(0.100855169714518,0.029141985319516,0.079335186630093, ...
        -0.000541360713197,-0.000736745555217,-0.026839527938637), ...
    symmetricTensor(0.005502104072070,0.017624423356653,0.019910268294615, ...
        -0.004695656408395,-0.002023265007920,-0.001539982979145), ...
    symmetricTensor(0.000390875617420,0.001583278285557,0.001382169277839, ...
        0.000020665893054,-0.000234483224917,0.000036951515987), ...
    symmetricTensor(0.000408850231533,0.001571028947078,0.001376444128997, ...
        -0.000146149911797,-0.000232875887541,-0.000034819514884), ...
    symmetricTensor(0.000396719978944,0.000059726463615, ...
        0.000384368311539,0.000013490534120, ...
        0.000004279170339,-0.000010178183054));
result.maxMassError = 0;
result.maxCOMError = 0;
result.maxInertiaError = 0;
result.minimumCOMEigenvalue = Inf;
for i = 1:robot.structure.dof
    body = robot.model.Bodies{i};
    result.maxMassError = max(result.maxMassError,abs(body.Mass-expectedMass(i)));
    result.maxCOMError = max(result.maxCOMError, ...
        norm(body.CenterOfMass-expectedCOM(i,:),Inf));
    result.maxInertiaError = max(result.maxInertiaError, ...
        norm(robot.params.links(i).inertia-expectedInertia(:,:,i),Inf));
    result.minimumCOMEigenvalue = min(result.minimumCOMEigenvalue, ...
        min(eig(robot.params.links(i).inertia)));
end
[baseMass,baseCOM,baseInertia] = readBaseInertia(robot.params.source.urdf);
result.baseURDFValid = abs(baseMass-4.000)<1e-12 && ...
    norm(baseCOM-[0.019014 0.084231 0.157204],Inf)<1e-12 && ...
    norm(baseInertia-symmetricTensor(0.083300383493810,0.042615866299863, ...
        0.077791810729023,-0.009690760751949,0.000134894085282, ...
        -0.000807819899129),Inf)<1e-12;
result.pass = result.maxMassError < 1e-12 && ...
    result.maxCOMError < 1e-12 && result.maxInertiaError < 1e-12 && ...
    result.minimumCOMEigenvalue > 0 && result.baseURDFValid;
end

function I = symmetricTensor(ixx,iyy,izz,ixy,ixz,iyz)
I = [ixx ixy ixz; ixy iyy iyz; ixz iyz izz];
end

function [mass,com,I] = readBaseInertia(urdfPath)
document = xmlread(char(urdfPath));
links = document.getElementsByTagName('link');
base = [];
for i = 0:links.getLength-1
    candidate = links.item(i);
    if string(candidate.getAttribute('name')) == "Base_Link"
        base = candidate;
        break;
    end
end
if isempty(base)
    error('validation:MissingBaseLink','Base_Link was not found in the URDF.');
end
inertial = base.getElementsByTagName('inertial').item(0);
mass = str2double(string(inertial.getElementsByTagName('mass').item(0).getAttribute('value')));
com = sscanf(char(inertial.getElementsByTagName('origin').item(0).getAttribute('xyz')),'%f').';
node = inertial.getElementsByTagName('inertia').item(0);
value = @(name) str2double(string(node.getAttribute(name)));
I = symmetricTensor(value('ixx'),value('iyy'),value('izz'), ...
    value('ixy'),value('ixz'),value('iyz'));
end

function result = validateFK(robot,numTests)
prior = rng;
cleanup = onCleanup(@() rng(prior)); %#ok<NASGU>
rng(8,'twister');
limits = robot.params.joints.positionLimits;
result.maxTransformError = 0;
for i = 1:numTests
    q = limits(:,1).'+rand(1,robot.structure.dof).*(limits(:,2)-limits(:,1)).';
    custom = forward_kinematics(robot,q);
    toolbox = getTransform(robot.model,q,char(robot.structure.frames.endEffector));
    result.maxTransformError = max(result.maxTransformError,norm(custom-toolbox,'fro'));
end
result.pass = result.maxTransformError < 1e-12;
end

function result = validateIKRoundTrip(robot)
qTarget = [0.20 -0.30 0.25 0.10 -0.20 0.15];
target = forward_kinematics(robot,qTarget);
solution = robot.solveIK(target,struct( ...
    'qSeed',qTarget+[0.02 -0.02 0.02 -0.02 0.02 -0.02], ...
    'numStarts',6,'seed',4));
result.valid = solution.valid;
result.positionError = solution.info.positionError;
result.orientationError = solution.info.orientationError;
result.pass = result.valid && result.positionError < 1e-5 && ...
    result.orientationError < 1e-4;
end

function result = validateDynamics(robot)
model = copy(robot.model);
model.Gravity = [0 0 -9.81];
q = [0.10 -0.20 0.15 0.05 -0.10 0.08];
qd = [0.05 -0.03 0.02 0.01 -0.02 0.01];
qdd = [0.10 -0.08 0.06 0.04 -0.02 0.03];
result.gravityTorque = gravityTorque(model,q);
result.massMatrix = massMatrix(model,q);
result.inverseDynamics = inverseDynamics(model,q,qd,qdd);
result.minimumEigenvalue = min(eig(0.5*(result.massMatrix+result.massMatrix.')));
result.pass = all(isfinite(result.gravityTorque)) && ...
    all(isfinite(result.massMatrix(:))) && ...
    all(isfinite(result.inverseDynamics)) && result.minimumEigenvalue > 0;
end

function valid = validBinarySTL(filePath)

info = dir(filePath);
valid = numel(info) == 1 && info.bytes >= 84;
if ~valid
    return;
end
file = fopen(filePath,'rb');
if file < 0
    valid = false;
    return;
end
cleanup = onCleanup(@() fclose(file)); %#ok<NASGU>
fseek(file,80,'bof');
triangleCount = fread(file,1,'uint32=>double','ieee-le');
valid = isscalar(triangleCount) && ...
    info.bytes == 84+50*triangleCount;

end

function result = validateRendering(robot)
oldVisibility = get(0,'DefaultFigureVisible');
set(0,'DefaultFigureVisible','off');
cleanup = onCleanup(@() set(0,'DefaultFigureVisible',oldVisibility)); %#ok<NASGU>
try
    figure;
    show(robot.model,robot.params.joints.homePosition, ...
        'Visuals','on','Collisions','off','Frames','on');
    close(gcf);
    figure;
    show(robot.model,robot.params.joints.homePosition, ...
        'Visuals','off','Collisions','on','Frames','off');
    close(gcf);
    result.pass = true;
    result.message = "Visual and collision rendering completed.";
catch ME
    close all;
    result.pass = false;
    result.message = string(ME.message);
end
end

function blocked = sizingIsBlocked(robot)
blocked = isfield(robot,'readiness') && ...
    isfield(robot.readiness,'actuatorSizingAllowed') && ...
    ~robot.readiness.actuatorSizingAllowed;
end

function pass = initialTrialLoads()
try
    oldRobot = loadRobot("initial_trial");
    pass = oldRobot.structure.dof == 6 && isa(oldRobot.model,'rigidBodyTree');
catch
    pass = false;
end
end
