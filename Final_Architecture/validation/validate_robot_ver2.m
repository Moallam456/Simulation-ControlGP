function report = validate_robot_ver2(robot)
% VALIDATE_ROBOT_VER2 Check the imported model without certifying CAD data.
if nargin < 1 || isempty(robot)
    robot = loadRobot("robot_ver2");
end
if string(robot.id) ~= "robot_ver2"
    error('validation:WrongRobot','Expected robot_ver2.');
end

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'kinematics'), ...
    fullfile(rootDir,'dynamics_analysis'),fullfile(rootDir,'collision_analysis'));
names = ["Link_1" "Link_2" "Link_3" "Link_4" "Link_5" "End_Effector"];
joints = "Joint_" + string(1:6);
report.topology = robot.structure.dof == 6 && ...
    string(robot.model.BaseName) == "Base_Link" && ...
    isequal(robot.structure.bodyNames,names) && ...
    isequal(robot.structure.jointNames,joints) && ...
    string(robot.structure.frames.endEffector) == "End_Effector";

allBodies = [{robot.model.Base} robot.model.Bodies];
report.meshes = all(cellfun(@(b) numel(b.Visuals)==1 && ...
    numel(b.Collisions)==1,allBodies)) && all(isfile(robot.collision.meshFiles));
document = xmlread(char(robot.params.source.urdf));
meshNodes = document.getElementsByTagName('mesh');
report.meshes = report.meshes && meshNodes.getLength == 14;
for i = 0:meshNodes.getLength-1
    uri = string(meshNodes.item(i).getAttribute('filename'));
    report.meshes = report.meshes && ...
        startsWith(uri,"package://URDF_Assembly_Robot_ver2/meshes/robot_ver2_");
end
for i = 1:numel(allBodies)
    meshName = "robot_ver2_" + string(allBodies{i}.Name) + ".STL";
    report.meshes = report.meshes && ...
        contains(string(allBodies{i}.Visuals{1}),meshName) && ...
        contains(string(allBodies{i}.Collisions{1}),meshName);
end
report.renderedMeshes = validateRenderedMeshes(robot);

report.massProperties = validateDynamicModel(robot);
report.inertialPlacement = validateInertialPlacement(robot);
q = [0.2 -0.3 0.25 0.1 -0.2 0.15];
T = forward_kinematics(robot,q);
report.fk = norm(T-getTransform(robot.model,q,'End_Effector'),'fro') < 1e-12;
solution = robot.solveIK(T,struct('qSeed',q,'numStarts',3));
report.ik = solution.valid && solution.info.positionError < 1e-5 && ...
    solution.info.orientationError < 1e-4 && ...
    all(solution.q(:) >= -pi-1e-9) && all(solution.q(:) <= pi+1e-9);

model = copy(robot.model);
model.Gravity = [0 0 -9.81];
M = massMatrix(model,q);
tau = inverseDynamics(model,q,0.1*ones(1,6),0.2*ones(1,6));
report.dynamics = all(isfinite(M(:))) && all(isfinite(tau)) && ...
    min(eig(0.5*(M+M.'))) > 0;

collisions = analyzeCollisions(robot,robot.params.joints.homePosition);
report.collision.status = "NONE_DETECTED_AT_HOME";
report.collision.pairs = robot.collision.knownHomeCollision;
report.collision.detected = collisions.isSelfColliding(1);
report.collision.pairsMatch = isempty(report.collision.pairs) && ...
    ~any(collisions.collisionPairCounts(:));
report.readiness = ~robot.readiness.actuatorSizingAllowed && ...
    robot.readiness.jointLimitsProvisional && ...
    robot.readiness.massPropertiesPreliminary;
report.pass = report.topology && report.meshes && report.renderedMeshes && ...
    report.massProperties.pass && report.inertialPlacement.pass && ...
    report.fk && report.ik && ...
    report.dynamics && ~report.collision.detected && ...
    report.collision.pairsMatch && report.readiness;
fprintf('robot_ver2: topology %d, meshes %d, rendered STLs %d, COM placement %d, FK %d, IK %d, dynamics %d\n', ...
    report.topology,report.meshes,report.renderedMeshes, ...
    report.inertialPlacement.pass,report.fk,report.ik,report.dynamics);
fprintf('Home collision detected: %d (%s); final actuator sizing blocked: %d\n', ...
    report.collision.detected,report.collision.status,report.readiness);
fprintf('robot_ver2 validation pass: %d\n',report.pass);
end

function result = validateInertialPlacement(robot)
% A link-local COM must be close to its own STL, not its CAD-world position.
names = string(robot.structure.bodyNames);
margin = NaN(size(names));
for i = 1:numel(names)
    meshPath = robot.collision.meshFiles(robot.collision.bodyNames == names(i));
    mesh = stlread(char(meshPath));
    vertices = mesh.Points;
    com = robot.model.Bodies{i}.CenterOfMass;
    margin(i) = min([com-min(vertices,[],1), max(vertices,[],1)-com]);
end
result.bodyNames = names;
result.minimumBoxMargin_m = margin;
result.pass = all(isfinite(margin)) && all(margin >= -1e-3);
end

function pass = validateRenderedMeshes(robot)
% Catch filename-only resolution loading a different robot's same-named STL.
pass = false;
figureHandle = figure('Visible','off');
cleanup = onCleanup(@() close(figureHandle)); %#ok<NASGU>
show(robot.model,robot.params.joints.homePosition, ...
    'Visuals','on','Collisions','off','Frames','off');
patches = findobj(gca,'Type','Patch');
renderedFaces = arrayfun(@(p) size(p.Faces,1),patches);
expectedFaces = zeros(1,numel(robot.collision.meshFiles));
for i = 1:numel(expectedFaces)
    mesh = stlread(char(robot.collision.meshFiles(i)));
    expectedFaces(i) = size(mesh.ConnectivityList,1);
end
for count = unique(expectedFaces)
    if sum(renderedFaces==count) < sum(expectedFaces==count)
        return;
    end
end
pass = true;
end
