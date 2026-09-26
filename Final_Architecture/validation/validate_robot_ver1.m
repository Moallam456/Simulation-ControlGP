function report = validate_robot_ver1(robot)
% VALIDATE_ROBOT_VER1 Acceptance checks for the incomplete URDF-backed model.

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

expectedBodies = ["Link_1" "Link_2" "Link_3" "Wrist_1" "Wrist_2"];
expectedJoints = ["Joint_1" "Joint_2" "Joint_3" "Joint_4" "Joint_5"];
report.topology = robot.structure.dof == 5 && ...
    string(robot.model.BaseName) == "Base_Link" && ...
    isequal(robot.structure.bodyNames,expectedBodies) && ...
    isequal(robot.structure.jointNames,expectedJoints) && ...
    string(robot.structure.frames.endEffector) == "Wrist_2";

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
    robot.readiness.joint6Missing && ...
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
fprintf('  Topology and five-axis endpoint: %d\n',report.topology);
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
result.urisValid = nodes.getLength == 12;
for i = 0:nodes.getLength-1
    uri = string(nodes.item(i).getAttribute('filename'));
    result.uris(i+1) = uri;
    result.urisValid = result.urisValid && startsWith(uri,"package://robot_ver1/meshes/");
end
result.pass = all(result.visualCounts == 1) && ...
    all(result.collisionCounts == 1) && result.filesExist && result.urisValid;
end

function result = validateMassProperties(robot)
expectedMass = [16.058 9.843 7.576 4.535 4.535];
expectedCOM = [
    -0.106451 -0.056736 0.101055148
     0.007556557 -0.323779148 -0.034318388
    -0.182291207 -0.065672927 0.003309351
    -0.066487521 0.010831776 -0.091501332
     0.069619270 -0.003019655 0.095127911];
expectedInertia = cat(3, ...
    symmetricTensor(0.136863054,0.105710534,0.182139824, ...
        -0.050765210,0.032896087,0.018485160), ...
    symmetricTensor(0.283633553,0.081955589,0.223113212, ...
        -0.001522461,-0.002071939,-0.075480421), ...
    symmetricTensor(0.017737847,0.056818141,0.064187316, ...
        -0.015137997,-0.006522662,-0.004964643), ...
    symmetricTensor(0.003082819,0.012487247,0.010901109, ...
        0.000162991,-0.001849359,0.000291435), ...
    symmetricTensor(0.003224584,0.012390637,0.010855955, ...
        -0.001152678,-0.001836682,-0.000274620));
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
result.baseURDFValid = abs(baseMass-21.810)<1e-12 && ...
    norm(baseCOM-[0.019014 0.084231 0.157204],Inf)<1e-12 && ...
    norm(baseInertia-symmetricTensor(0.454195341,0.232363011,0.424159848, ...
        -0.052838873,0.000735510,-0.004404638),Inf)<1e-12;
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
qTarget = [0.20 -0.30 0.25 0.10 -0.20];
target = forward_kinematics(robot,qTarget);
solution = robot.solveIK(target,struct( ...
    'qSeed',qTarget+[0.02 -0.02 0.02 -0.02 0.02], ...
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
q = [0.10 -0.20 0.15 0.05 -0.10];
qd = [0.05 -0.03 0.02 0.01 -0.02];
qdd = [0.10 -0.08 0.06 0.04 -0.02];
result.gravityTorque = gravityTorque(model,q);
result.massMatrix = massMatrix(model,q);
result.inverseDynamics = inverseDynamics(model,q,qd,qdd);
result.minimumEigenvalue = min(eig(0.5*(result.massMatrix+result.massMatrix.')));
result.pass = all(isfinite(result.gravityTorque)) && ...
    all(isfinite(result.massMatrix(:))) && ...
    all(isfinite(result.inverseDynamics)) && result.minimumEigenvalue > 0;
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
blocked = false;
try
    createActuatorSizingCases(robot);
catch ME
    blocked = strcmp(ME.identifier,'sizing:RobotNotReady');
end
end

function pass = initialTrialLoads()
try
    oldRobot = loadRobot("initial_trial");
    pass = oldRobot.structure.dof == 6 && isa(oldRobot.model,'rigidBodyTree');
catch
    pass = false;
end
end
