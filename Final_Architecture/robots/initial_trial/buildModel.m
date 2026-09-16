function model = buildModel(params,structure,dh)

% BUILDMODEL Build the initial_trial rigidBodyTree.
%
% This function assembles supplied data only. It does not own geometry or
% call robotParameters internally, which keeps parameter sweeps simple.

model = rigidBodyTree( ...
    'DataFormat','row', ...
    'MaxNumBodies',structure.dof + 1);

parentName = structure.frames.base;

for i = 1:structure.dof

    body = rigidBody(char(structure.bodyNames(i)));
    joint = rigidBodyJoint( ...
        char(structure.jointNames(i)), ...
        char(structure.jointTypes(i)));

    setFixedTransform(joint,dh.fixedTransforms{i});
    joint.JointAxis = structure.jointAxes(i,:);
    joint.HomePosition = params.joints.homePosition(i);
    joint.PositionLimits = params.joints.positionLimits(i,:);

    body.Joint = joint;

    if isfield(params,'links') && numel(params.links) >= i
        body = assignDynamicsIfKnown(body,params.links(i));
    end

    addBody(model,body,char(parentName));
    parentName = structure.bodyNames(i);

end

tcpBody = rigidBody(char(structure.frames.endEffector));
tcpJoint = rigidBodyJoint('tcp_fixed','fixed');
setFixedTransform(tcpJoint,params.tool.TFlangeTCP);
tcpBody.Joint = tcpJoint;
addBody(model,tcpBody,char(structure.frames.flange));

end

function body = assignDynamicsIfKnown(body,linkParams)

if isfield(linkParams,'mass') && isfinite(linkParams.mass)
    body.Mass = linkParams.mass;
end

if isfield(linkParams,'centerOfMass') && ...
   numel(linkParams.centerOfMass) == 3 && ...
   all(isfinite(linkParams.centerOfMass))
    body.CenterOfMass = linkParams.centerOfMass;
end

if isfield(linkParams,'inertia') && isequal(size(linkParams.inertia),[3 3]) && ...
   all(isfinite(linkParams.inertia(:)))
    I = linkParams.inertia;
    body.Inertia = [I(1,1) I(2,2) I(3,3) I(2,3) I(1,3) I(1,2)];
end

end
