function definition = robotDefinition(overrides)
% ROBOTDEFINITION Import the ver2 CAD export into the common robot interface.

if nargin < 1
    overrides = [];
end
robotFolder = fileparts(mfilename('fullpath'));
packageFolder = fullfile(robotFolder,'CAD','URDF_Assembly_Robot_ver2');
urdfPath = fullfile(packageFolder,'urdf','URDF_Assembly_Robot_ver2.urdf');
if ~isfile(urdfPath)
    error('robot_ver2:MissingURDF','URDF file not found: %s',urdfPath);
end
model = importrobot(urdfPath,'DataFormat','row');
movingBodies = model.Bodies(cellfun(@(b) ~strcmpi(b.Joint.Type,'fixed'),model.Bodies));
dof = numel(movingBodies);
if dof ~= 6 || numel(model.Bodies) ~= 6
    error('robot_ver2:UnexpectedTopology', ...
        'Expected exactly six moving bodies and joints; found %d bodies, %d joints.', ...
        numel(model.Bodies),dof);
end

% The source export has zero-width limits. Keep it unchanged and record the
% temporary MATLAB analysis range separately.
limits = repmat([-pi pi],dof,1);
home = zeros(1,dof);
if ~isempty(overrides)
    [limits,home] = jointOverrides(overrides,limits,home,dof);
end
for i = 1:dof
    movingBodies{i}.Joint.PositionLimits = limits(i,:);
    movingBodies{i}.Joint.HomePosition = home(i);
end
bodyNames = string(cellfun(@(b) b.Name,movingBodies,'UniformOutput',false));
jointNames = string(cellfun(@(b) b.Joint.Name,movingBodies,'UniformOutput',false));
jointAxes = cell2mat(cellfun(@(b) b.Joint.JointAxis,movingBodies, ...
    'UniformOutput',false).');

params.id = "robot_ver2";
params.displayName = "robot_ver2 (preliminary CAD export)";
params.source.type = "URDF";
params.source.urdf = string(urdfPath);
params.source.packageFolder = string(packageFolder);
params.source.massPropertyAccuracy = "LOW_ACCURACY_CAD_FRAME_CONVERTED";
params.source.inertiaAssumption = ...
    "Inventor Y-up assembly COMs and COM tensors converted to URDF link frames at q=0; verify CAD accuracy before sizing";
params.joints.positionLimits = limits;
params.joints.homePosition = home;
params.joints.limitStatus = "PROVISIONAL";
params.joints.limitSource = "Temporary MATLAB range; source URDF has zero-width limits";
params.links = repmat(bodyParameters(movingBodies{1}),1,dof);
for i = 1:dof
    params.links(i) = bodyParameters(movingBodies{i});
end
params.base.source = "Base inertial properties are stored in the URDF";
params.base.note = "MATLAB does not expose base inertia through rigidBodyTree.Base.";
params.tool.name = "temporary End_Effector TCP";
params.tool.bodyName = "End_Effector";
params.tool.TFlangeTCP = eye(4);
params.tool.status = "PROVISIONAL";

structure.name = "robot_ver2";
structure.model = "URDF-backed preliminary six-axis CAD model";
structure.dof = dof;
structure.bodyNames = bodyNames;
structure.jointNames = jointNames;
structure.jointTypes = repmat("revolute",1,dof);
structure.jointAxes = jointAxes;
structure.frames.base = string(model.BaseName);
structure.frames.endEffector = bodyNames(end);
structure.frames.flange = bodyNames(end);
structure.hasSphericalWrist = false;

chain.convention = "URDF parent-to-child joint origins";
chain.source = string(urdfPath);
chain.homeFrames = cell(1,dof+2);
chain.homeFrames{1} = eye(4);
for i = 1:dof
    chain.homeFrames{i+1} = getTransform(model,home,char(bodyNames(i)));
end
chain.homeFrames{end} = chain.homeFrames{dof+1};
chain.usesRigidBodyTreeFK = true;

readiness.status = "PRELIMINARY_SIX_AXIS_MODEL";
readiness.warnings = [ ...
    "URDF joint limits are zero-width; analysis uses provisional [-pi, pi] limits.";
    "The TCP is temporarily the End_Effector body-frame origin.";
    "Moving-link inertials were converted from Inventor assembly coordinates; CAD Low accuracy and frame alignment need independent confirmation.";
    "Copied ver2 mesh filenames are unique to avoid MATLAB resolving same-named ver1 STL files.";
    "The corrected home pose has no detected self-collision; sampled poses can still collide.";
    "CAD mass properties, collision clearances, and motor ratings are not validated.";
    "Final actuator sizing is not allowed for this model."];
readiness.joint6Missing = false;
readiness.jointLimitsProvisional = true;
readiness.tcpProvisional = true;
readiness.massPropertiesPreliminary = true;
readiness.unresolvedHomeCollision = strings(0,2);
readiness.actuatorSizingAllowed = false;
readiness.allowedUses = ["FK" "IK" "visualization" "workspace" ...
    "collision inspection" "trajectory generation" "preliminary rigid-body dynamics"];

definition.params = params;
definition.structure = structure;
definition.chain = chain;
definition.model = model;
definition.readiness = readiness;
end

function [limits,home] = jointOverrides(overrides,limits,home,dof)
if ~isstruct(overrides) || ~isfield(overrides,'joints')
    error('robot_ver2:InvalidOverrides', ...
        'Overrides must be a params struct containing joints metadata.');
end
if isfield(overrides.joints,'positionLimits')
    candidate = overrides.joints.positionLimits;
    if ~isequal(size(candidate),[dof 2]) || any(~isfinite(candidate(:))) || ...
            any(candidate(:,1) >= candidate(:,2))
        error('robot_ver2:InvalidLimits','Joint limits must be finite %d-by-2.',dof);
    end
    limits = candidate;
end
if isfield(overrides.joints,'homePosition')
    candidate = reshape(overrides.joints.homePosition,1,[]);
    if numel(candidate) ~= dof || any(~isfinite(candidate)) || ...
            any(candidate < limits(:,1).' | candidate > limits(:,2).')
        error('robot_ver2:InvalidHome','Home must contain %d limit-valid angles.',dof);
    end
    home = candidate;
end
end

function data = bodyParameters(body)
mass = body.Mass;
r = body.CenterOfMass(:);
v = body.Inertia;
Iorigin = [v(1) v(6) v(5);v(6) v(2) v(4);v(5) v(4) v(3)];
Icom = Iorigin-mass*((r.'*r)*eye(3)-r*r.');
data.name = string(body.Name);
data.mass = mass;
data.centerOfMass = reshape(r,1,3);
data.inertia = 0.5*(Icom+Icom.');
data.massPropertySource = "robot_ver2 URDF; converted from Inventor assembly frame";
data.dynamics.status = "estimated";
end
