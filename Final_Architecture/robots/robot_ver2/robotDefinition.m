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
if dof ~= 6 || numel(model.Bodies) ~= 7 || ~ismember('tcp',model.BodyNames)
    error('robot_ver2:UnexpectedTopology', ...
        'Expected six moving bodies and one fixed tcp frame; found %d bodies, %d joints.', ...
        numel(model.Bodies),dof);
end

% These ranges are provisional even though the URDF now carries them too.
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
params.source.modelVersion = "robot_ver2_6axis_tcp_v1";
params.source.massPropertyAccuracy = "LOW_ACCURACY_CAD_FRAME_CONVERTED";
params.source.inertiaAssumption = ...
    "Inventor Y-up assembly COMs and COM tensors converted to URDF link frames at q=0; verify CAD accuracy before sizing";
params.joints.positionLimits = limits;
params.joints.homePosition = home;
params.joints.limitStatus = "PROVISIONAL";
params.joints.limitSource = "Provisional [-pi, pi] ranges in the URDF; confirm against mechanical stops";
params.links = repmat(bodyParameters(movingBodies{1}),1,dof);
for i = 1:dof
    params.links(i) = bodyParameters(movingBodies{i});
end
params.base.source = "Base inertial properties are stored in the URDF";
params.base.note = "MATLAB does not expose base inertia through rigidBodyTree.Base.";
params.tool = toolParameters(model,home,overrides);

% Practical search seeds for this geometry, not manufacturer joint settings.
% Negative J3 folds the forearm back; positive J3 raises/extends it.
params.park.seedConfigurations = deg2rad([
    0 0 -120 60 0 0
    0 0 -120 120 0 0
    0 0 -150 0 0 0
    0 10 -110 60 45 0
    0 -10 -110 60 -45 0
    0 0 -100 60 90 0
    0 0 -100 60 -90 0
    0 15 130 0 90 0
    0 15 140 0 90 0
    0 15 150 60 90 0]);
params.park.localSpread_deg = [20 12 25 100 100 120];
params.park.maximumHeight_m = 1.35;
params.park.maximumRadius_m = .65;
params.park.seedDescription = "Upright shoulder, folded-back forearm, tucked wrist; verified using this robot's frames";
% Historical near-zero holding-load pose, for display only (not a search seed).
params.park.uprightComparisonConfiguration = [ ...
    -0.142666694229407 -3.50615582789260e-7 1.57050091431967 ...
    1.27352568611370 -0.0743839171890253 0.141423219387215];

structure.name = "robot_ver2";
structure.model = "URDF-backed preliminary six-axis CAD model";
structure.dof = dof;
structure.bodyNames = bodyNames;
structure.jointNames = jointNames;
structure.jointTypes = repmat("revolute",1,dof);
structure.jointAxes = jointAxes;
structure.frames.base = string(model.BaseName);
structure.frames.endEffector = "tcp";
structure.frames.tcp = "tcp";
structure.frames.flange = bodyNames(end);
structure.hasSphericalWrist = false;

chain.convention = "URDF parent-to-child joint origins";
chain.source = string(urdfPath);
chain.homeFrames = cell(1,dof+2);
chain.homeFrames{1} = eye(4);
for i = 1:dof
    chain.homeFrames{i+1} = getTransform(model,home,char(bodyNames(i)));
end
chain.homeFrames{end} = getTransform(model,home,'tcp');
chain.usesRigidBodyTreeFK = true;

readiness.status = "PRELIMINARY_SIX_AXIS_MODEL";
readiness.warnings = [ ...
    "URDF and MATLAB use provisional [-pi, pi] joint limits; mechanical stops are not yet verified.";
    "The tcp fixed frame uses the user-supplied Joint-6-relative measurement (2026-10-15).";
    "Moving-link inertials were converted from Inventor assembly coordinates; CAD Low accuracy and frame alignment need independent confirmation.";
    "Copied ver2 mesh filenames are unique to avoid MATLAB resolving same-named ver1 STL files.";
    "The corrected home pose has no detected self-collision; sampled poses can still collide.";
    "CAD mass properties, collision clearances, and motor ratings are not validated.";
    "Final actuator sizing is not allowed for this model."];
readiness.joint6Missing = false;
readiness.jointLimitsProvisional = true;
readiness.tcpProvisional = params.tool.status~="CONFIGURED";
readiness.tcpStatus = params.tool.status;
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
if ~isstruct(overrides)
    error('robot_ver2:InvalidOverrides', ...
        'Overrides must be a params struct.');
end
if ~isfield(overrides,'joints'), return; end
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

function tool = toolParameters(model,home,overrides)
% The URDF tcp_fixed origin is authoritative. Overrides are in-memory studies.
body = getBody(model,'tcp');
if ~strcmp(body.Joint.Type,'fixed') || ~strcmp(body.Parent.Name,'End_Effector')
    error('robot_ver2:InvalidTCP','tcp must be fixed directly to End_Effector.');
end
if body.Mass~=0 || any(body.Inertia~=0) || ...
        ~isempty(body.Visuals) || ~isempty(body.Collisions)
    error('robot_ver2:DuplicateTool','tcp must be a massless, geometry-free frame.');
end
tool.name = "welding torch TCP";
tool.bodyName = "tcp";
tool.referenceFrame = "End_Effector";
tool.version = "torch-tcp-2026-10-15";
tool.source = "URDF tcp_fixed origin (metres, radians)";
tool.TFlangeTCP = getTransform(model,home,'tcp','End_Effector');
tool.status = "CONFIGURED";
if norm(tool.TFlangeTCP-eye(4),'fro')<1e-12
    tool.status = "UNCONFIGURED";
end
if ~isempty(overrides) && isfield(overrides,'tool')
    input = overrides.tool;
    if isfield(input,'TFlangeTCP')
        T = input.TFlangeTCP;
        if ~isequal(size(T),[4 4]) || ~isreal(T) || any(~isfinite(T(:))) || ...
                norm(T(4,:)-[0 0 0 1])>1e-10 || ...
                norm(T(1:3,1:3).'*T(1:3,1:3)-eye(3),'fro')>1e-8 || ...
                abs(det(T(1:3,1:3))-1)>1e-8
            error('robot_ver2:InvalidToolTransform','TFlangeTCP must be a proper rigid transform.');
        end
        joint = rigidBodyJoint('tcp_fixed','fixed');
        setFixedTransform(joint,T);
        replaceJoint(model,'tcp',joint);
        tool.TFlangeTCP = getTransform(model,home,'tcp','End_Effector');
        tool.source = "Explicit in-memory override; URDF file is unchanged";
        tool.status = "OVERRIDE_UNVERIFIED";
    end
    if isfield(input,'version'), tool.version = string(input.version); end
end
tool.mass = 0;
tool.centerOfMass = [0 0 0];
tool.inertia = zeros(3);
tool.massPropertySource = "Frame only; torch mass and geometry are already on End_Effector";
tool.dynamics.status = "known";
% Row-major layout expected by the control team's T_flange_tcp[16] interface.
tool.control.T_flange_tcp = reshape(tool.TFlangeTCP.',1,16);
tool.control.parentFrame = tool.referenceFrame;
tool.control.childFrame = tool.bodyName;
tool.control.units = "metres";
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
