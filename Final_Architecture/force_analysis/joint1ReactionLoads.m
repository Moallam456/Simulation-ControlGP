function result = joint1ReactionLoads(model,q,qd,qdd)
% JOINT1REACTIONLOADS Joint-1 support-on-arm wrench for force analysis.
% Inputs are N-by-DOF in SI units. Zero speed/acceleration gives static loads.
% No external contact loads. This is not the load in each individual bearing.
if ~isa(model,'rigidBodyTree') || ~strcmp(model.DataFormat,'row')
    error('jointLoads:Model','Expected a row-format rigidBodyTree.');
end
moving = find(cellfun(@(b) ~strcmp(b.Joint.Type,'fixed'),model.Bodies));
dof = numel(moving);
if nargin < 3 || isempty(qd), qd = zeros(size(q)); end
if nargin < 4 || isempty(qdd), qdd = zeros(size(q)); end
if dof == 0 || ~isnumeric(q) || ~isnumeric(qd) || ~isnumeric(qdd) || ~isreal(q) || isempty(q) || ...
        size(q,2) ~= dof || ~isequal(size(q),size(qd),size(qdd)) || ...
        ~isreal(qd) || ~isreal(qdd) || any(~isfinite([q(:);qd(:);qdd(:)]))
    error('jointLoads:State','Expected finite real N-by-DOF q, qd and qdd.');
end
first = model.Bodies{moving(1)};
if ~strcmp(first.Joint.Type,'revolute')
    error('jointLoads:JointType','This bearing analysis requires a revolute first joint.');
end

% Retain only inertia downstream of Joint 1, including fixed tools but not
% the pedestal. Never modify the caller's model or add mass for the TCP.
arm = copy(model);
supported = false(1,model.NumBodies);
for k = 1:model.NumBodies
    ancestor = model.Bodies{k};
    while ~isempty(ancestor)
        if strcmp(ancestor.Name,first.Name)
            supported(k) = true;
            break;
        end
        if strcmp(ancestor.Name,model.BaseName), break; end
        ancestor = ancestor.Parent;
    end
    if ~supported(k)
        arm.Bodies{k}.Mass = 0;
        arm.Bodies{k}.CenterOfMass = [0 0 0];
        arm.Bodies{k}.Inertia = zeros(1,6);
    end
end

% Six massless, stationary virtual joints form a computational load cell.
% Their generalized forces are Fx,Fy,Fz,Mx,My,Mz at the original base origin.
% Physical joint coordinates and the robot returned to the user are unchanged.
sensor = rigidBodyTree('DataFormat','row');
sensor.Gravity = model.Gravity;
prefix = 'reaction_sensor_';
while any(startsWith(string([model.BodyNames {model.BaseName}]),prefix))
    prefix = [prefix '_']; %#ok<AGROW>
end
parent = sensor.BaseName;
for k = 1:6
    body = rigidBody(sprintf('%s%d',prefix,k));
    kind = 'prismatic';
    if k > 3, kind = 'revolute'; end
    joint = rigidBodyJoint(sprintf('%sj%d',prefix,k),kind);
    direction = zeros(1,3);
    direction(mod(k-1,3)+1) = 1;
    joint.JointAxis = direction;
    body.Joint = joint;
    body.Mass = 0;
    body.CenterOfMass = [0 0 0];
    body.Inertia = zeros(1,6);
    addBody(sensor,body,parent);
    parent = body.Name;
end
addSubtree(sensor,parent,arm);

n = size(q,1);
force = zeros(n,3);
moment = zeros(n,3);
forceLocal = zeros(n,3);
momentLocal = zeros(n,3);
jointOrigin = zeros(n,3);
jointAxis = zeros(n,3);
torques = zeros(n,dof);
for k = 1:n
    wrench = inverseDynamics(sensor,[zeros(1,6) q(k,:)], ...
        [zeros(1,6) qd(k,:)],[zeros(1,6) qdd(k,:)]);
    if strcmp(first.Parent.Name,model.BaseName)
        Tparent = eye(4);
    else
        Tparent = getTransform(model,q(k,:),first.Parent.Name);
    end
    Tjoint = Tparent*first.Joint.JointToParentTransform;
    R = Tjoint(1:3,1:3);
    p = Tjoint(1:3,4).';
    force(k,:) = wrench(1:3);
    moment(k,:) = wrench(4:6)-cross(p,force(k,:));
    forceLocal(k,:) = force(k,:)*R;
    momentLocal(k,:) = moment(k,:)*R;
    jointOrigin(k,:) = p;
    jointAxis(k,:) = first.Joint.JointAxis*R.';
    torques(k,:) = wrench(7:end);
end
axialSigned = sum(force.*jointAxis,2);
torsionSigned = sum(moment.*jointAxis,2);
result.jointName = string(first.Joint.Name);
result.supportedBodyNames = string(model.BodyNames(supported));
result.supportedMass_kg = sum(cellfun(@(b) b.Mass,model.Bodies(supported)));
result.q = q;
result.qd = qd;
result.qdd = qdd;
result.forceBase_N = force;
result.momentAtJointBase_Nm = moment;
result.forceJoint_N = forceLocal;
result.momentJoint_Nm = momentLocal;
result.jointOriginBase_m = jointOrigin;
result.jointAxisBase = jointAxis;
result.axialSigned_N = axialSigned;
result.axial_N = abs(axialSigned);
result.radial_N = vecnorm(force-axialSigned.*jointAxis,2,2);
result.bending_Nm = vecnorm(moment-torsionSigned.*jointAxis,2,2);
result.torsionSigned_Nm = torsionSigned;
result.torsion_Nm = abs(torsionSigned);
result.physicalJointTorques_Nm = torques;
result.gravity_m_s2 = model.Gravity;
result.convention = "Support-on-arm; arm-on-support is the negative vector. Joint axes are the stationary pre-motion joint frame.";
result.scope = "Ideal rigid-body resultant, not individual bearing reactions. No process/cable/contact loads, friction, preload, compliance, or safety factors.";
end
