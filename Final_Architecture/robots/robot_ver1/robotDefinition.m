function definition = robotDefinition(overrides)
% ROBOTDEFINITION Import robot_ver1 and expose the common robot interface.
% The URDF is authoritative for topology, transforms, meshes, and mass data.

if nargin < 1
    overrides = [];
end

robotFolder = fileparts(mfilename('fullpath'));
packageFolder = fullfile(robotFolder,'CAD','robot_ver1');
urdfPath = fullfile(packageFolder,'urdf','robot_ver1.urdf');
if ~isfile(urdfPath)
    error('robot_ver1:MissingURDF','URDF file not found: %s',urdfPath);
end

model = importrobot(urdfPath,'DataFormat','row');
movingBodies = model.Bodies(cellfun( ...
    @(body) ~strcmpi(body.Joint.Type,'fixed'),model.Bodies));
dof = numel(movingBodies);
if dof ~= 6
    error('robot_ver1:UnexpectedDOF', ...
        'robot_ver1 must contain six moving joints; found %d.',dof);
end

limits = repmat([-pi pi],dof,1);
home = zeros(1,dof);
if ~isempty(overrides)
    [limits,home] = applyJointOverrides(overrides,limits,home,dof);
end
for i = 1:dof
    movingBodies{i}.Joint.PositionLimits = limits(i,:);
    movingBodies{i}.Joint.HomePosition = home(i);
end

bodyNames = string(cellfun(@(body) body.Name,movingBodies, ...
    'UniformOutput',false));
jointNames = string(cellfun(@(body) body.Joint.Name,movingBodies, ...
    'UniformOutput',false));
jointAxes = cell2mat(cellfun(@(body) body.Joint.JointAxis,movingBodies, ...
    'UniformOutput',false).');

params.id = "robot_ver1";
params.displayName = "robot_ver1 (preliminary six-axis CAD model)";
params.source.type = "URDF";
params.source.urdf = string(urdfPath);
params.source.packageFolder = string(packageFolder);
params.source.massPropertyAccuracy = "Inventor Low";
params.source.joint6Geometry = ...
    "URDF_Files(ver2): Joint_6 transform, axis, and End_Effector mesh";
params.source.endEffectorDynamics = ...
    "Inventor screenshots converted from world coordinates and kg*mm^2 to the End_Effector body frame in SI units";
params.joints.positionLimits = limits;
params.joints.homePosition = home;
params.joints.limitStatus = "PROVISIONAL";
params.joints.limitSource = "Temporary MATLAB metadata; not the zero limits in the CAD export";
params.links = bodyParameterArray(movingBodies);
params.base.source = "Base inertial properties remain authoritative in the URDF";
params.base.note = "MATLAB does not expose inertial properties on rigidBodyTree.Base.";
params.tool.name = "temporary End_Effector TCP";
params.tool.bodyName = "End_Effector";
params.tool.TFlangeTCP = eye(4);
params.tool.status = "PROVISIONAL";

structure.name = "robot_ver1";
structure.model = "URDF-backed preliminary six-axis CAD model";
structure.dof = dof;
structure.bodyNames = bodyNames;
structure.jointNames = jointNames;
structure.jointTypes = repmat("revolute",1,dof);
structure.jointAxes = jointAxes;
structure.frames.base = string(model.BaseName);
structure.frames.endEffector = "End_Effector";
structure.frames.flange = "End_Effector";
structure.hasSphericalWrist = false;

chain.convention = "URDF parent-to-child joint origins";
chain.source = string(urdfPath);
chain.homeFrames = homeFrames(model,home,structure);
chain.usesRigidBodyTreeFK = true;

readiness.status = "PRELIMINARY_SIX_AXIS_MODEL";
readiness.warnings = [
    "All six joint limits are provisional [-pi, pi]."
    "The TCP is temporarily the End_Effector body-frame origin."
    "Inventor mass properties were exported at Low accuracy."
    "The Link_3/Wrist_2 home collision is unresolved; MATLAB uses convex mesh collision geometry."
    "Final actuator sizing is not allowed for this model."];
readiness.joint6Missing = false;
readiness.jointLimitsProvisional = true;
readiness.tcpProvisional = true;
readiness.massPropertiesPreliminary = true;
readiness.unresolvedHomeCollision = ["Link_3" "Wrist_2"];
readiness.actuatorSizingAllowed = false;
readiness.allowedUses = ["FK" "IK" "visualization" "workspace" ...
    "collision inspection" "trajectory generation" ...
    "preliminary rigid-body dynamics"];

definition.params = params;
definition.structure = structure;
definition.chain = chain;
definition.model = model;
definition.readiness = readiness;

end

function [limits,home] = applyJointOverrides(overrides,limits,home,dof)

if ~isstruct(overrides) || ~isfield(overrides,'joints')
    error('robot_ver1:InvalidOverrides', ...
        'robot_ver1 accepts only a params struct containing joint metadata overrides.');
end
if isfield(overrides.joints,'positionLimits')
    candidate = overrides.joints.positionLimits;
    if ~isequal(size(candidate),[dof 2]) || any(~isfinite(candidate(:))) || ...
            any(candidate(:,1) >= candidate(:,2))
        error('robot_ver1:InvalidLimits', ...
            'Joint limits must be a finite %d-by-2 array.',dof);
    end
    limits = candidate;
end
if isfield(overrides.joints,'homePosition')
    candidate = reshape(overrides.joints.homePosition,1,[]);
    if numel(candidate) ~= dof || any(~isfinite(candidate)) || ...
            any(candidate < limits(:,1).' | candidate > limits(:,2).')
        error('robot_ver1:InvalidHome', ...
            'Home position must contain %d limit-valid angles.',dof);
    end
    home = candidate;
end

end

function links = bodyParameterArray(bodies)

links = repmat(bodyParameters(bodies{1},string(bodies{1}.Name)),1,numel(bodies));
for i = 1:numel(bodies)
    links(i) = bodyParameters(bodies{i},string(bodies{i}.Name));
end

end

function data = bodyParameters(body,name)

mass = body.Mass;
r = reshape(body.CenterOfMass,3,1);
v = body.Inertia;
Iorigin = [v(1) v(6) v(5); v(6) v(2) v(4); v(5) v(4) v(3)];
Icom = Iorigin - mass*((r.'*r)*eye(3)-r*r.');
Icom = 0.5*(Icom+Icom.');
data.name = string(name);
data.mass = mass;
data.centerOfMass = reshape(r,1,3);
data.inertia = Icom;
data.massPropertySource = "robot_ver1 URDF corrected from Inventor screenshots";
data.dynamics.status = "estimated";

end

function frames = homeFrames(model,q,structure)

frames = cell(1,structure.dof+2);
frames{1} = eye(4);
for i = 1:structure.dof
    frames{i+1} = getTransform(model,q,char(structure.bodyNames(i)));
end
frames{end} = getTransform(model,q,char(structure.frames.endEffector));

end
