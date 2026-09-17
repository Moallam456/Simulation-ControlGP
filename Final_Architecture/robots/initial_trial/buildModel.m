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
        body = assignMassProperties(body,params.links(i),dh,i);
        body = addSimplifiedGeometry(body,params.links(i),dh,i);
    end

    addBody(model,body,char(parentName));
    parentName = structure.bodyNames(i);

end

tcpBody = rigidBody(char(structure.frames.endEffector));
tcpJoint = rigidBodyJoint('tcp_fixed','fixed');
setFixedTransform(tcpJoint,params.tool.TFlangeTCP);
tcpBody.Joint = tcpJoint;
tcpBody = assignToolMassProperties(tcpBody,params.tool);
tcpBody = addToolGeometry(tcpBody,params.tool);
addBody(model,tcpBody,char(structure.frames.flange));

end

function body = assignMassProperties(body,linkParams,dh,linkIndex)

if ~isfield(linkParams,'mass') || ~isscalar(linkParams.mass) || ...
        ~isfinite(linkParams.mass) || linkParams.mass <= 0
    error('buildModel:MissingLinkMass','Link %d needs a positive mass.',linkIndex);
end
body.Mass = linkParams.mass;
if ~isfield(linkParams,'structuralMass') || ...
        ~isfield(linkParams,'jointModuleMass') || ...
        abs(linkParams.mass - linkParams.structuralMass - linkParams.jointModuleMass) > 1e-10
    error('buildModel:MassMismatch', ...
        'Link %d mass must equal structuralMass + jointModuleMass.',linkIndex);
end

if hasKnownMassProperties(linkParams)
    body.CenterOfMass = linkParams.centerOfMass;
    I = linkParams.inertia;
    if norm(I-I.','fro') > 1e-10*max(1,norm(I,'fro'))
        error('buildModel:AsymmetricInertia','Link %d inertia must be symmetric.',linkIndex);
    end
    body.Inertia = [I(1,1) I(2,2) I(3,3) I(2,3) I(1,3) I(1,2)];
    return;
end

if isfield(linkParams,'radius') && isfinite(linkParams.radius) && ...
        linkParams.radius > 0

    components = massComponentsInBodyFrame(dh,linkParams,linkIndex);
    [com,inertia] = estimateMassProperties(components,linkParams.radius);

    body.CenterOfMass = com;
    body.Inertia = [
        inertia(1,1) ...
        inertia(2,2) ...
        inertia(3,3) ...
        inertia(2,3) ...
        inertia(1,3) ...
        inertia(1,2)];
else
    error('buildModel:MissingLinkRadius', ...
        'Link %d needs an estimate radius or explicit COM and inertia.',linkIndex);
end
if ~all(isfinite(body.CenterOfMass)) || ~all(isfinite(body.Inertia))
    error('buildModel:MissingLinkInertia', ...
        'Link %d needs COM and inertia or valid estimate geometry.',linkIndex);
end

end

function known = hasKnownMassProperties(linkParams)

known = isfield(linkParams,'centerOfMass') && ...
    isfield(linkParams,'inertia') && ...
    numel(linkParams.centerOfMass) == 3 && ...
    isequal(size(linkParams.inertia),[3 3]) && ...
    all(isfinite(linkParams.centerOfMass)) && ...
    all(isfinite(linkParams.inertia(:)));

end

function body = assignToolMassProperties(body,toolParams)

if ~isfield(toolParams,'mass') || ~isfinite(toolParams.mass) || toolParams.mass <= 0 || ...
        ~isfield(toolParams,'segmentMass') || ~isfinite(toolParams.segmentMass) || ...
        toolParams.segmentMass < 0
    error('buildModel:MissingToolMass','Tool and J6-to-EE masses must be explicit.');
end

toolMass = toolParams.mass;
segmentMass = toolParams.segmentMass;
body.Mass = toolMass + segmentMass;

if isfield(toolParams,'centerOfMass') && numel(toolParams.centerOfMass) == 3 && ...
   all(isfinite(toolParams.centerOfMass))
    toolCOM = toolParams.centerOfMass;
else
    error('buildModel:MissingToolCOM','Tool COM must be specified in TCP frame.');
end

if isfield(toolParams,'radius') && isfinite(toolParams.radius) && toolParams.radius > 0
    radius = toolParams.radius;
else
    error('buildModel:MissingToolRadius','Tool estimate radius must be specified.');
end

length = norm(toolParams.TFlangeTCP(1:3,4));
if length <= 0 && segmentMass > 0
    error('buildModel:MissingToolSegmentLength','J6-to-EE segment length is required.');
end
segmentCOM = [0 0 -length/2];
body.CenterOfMass = (toolMass*toolCOM + segmentMass*segmentCOM)/body.Mass;
if isfield(toolParams,'inertia') && isequal(size(toolParams.inertia),[3 3]) && ...
        all(isfinite(toolParams.inertia(:)))
    I = toolParams.inertia;
    if norm(I-I.','fro') > 1e-10*max(1,norm(I,'fro'))
        error('buildModel:AsymmetricToolInertia','Tool inertia must be symmetric.');
    end
else
    I = sphereInertia(toolMass,radius);
end
if segmentMass > 0
    I = I + cylinderInertiaAboutCenter(segmentMass,radius,length,[0 0 1]);
end
d = toolCOM - body.CenterOfMass;
I = I + toolMass*((d*d.')*eye(3) - d.'*d);
d = segmentCOM - body.CenterOfMass;
I = I + segmentMass*((d*d.')*eye(3) - d.'*d);
body.Inertia = [I(1,1) I(2,2) I(3,3) I(2,3) I(1,3) I(1,2)];

end

function body = addToolGeometry(body,toolParams)

if ~isfield(toolParams,'TFlangeTCP')
    return;
end

length = abs(toolParams.TFlangeTCP(3,4));

if length < 1e-9
    return;
end

if isfield(toolParams,'radius') && isfinite(toolParams.radius) && toolParams.radius > 0
    radius = toolParams.radius;
else
    radius = 0.025;
end

% The TCP body frame is at the tool tip, so the simplified tool cylinder
% extends backward along local -Z toward the flange.
T = cylinderTransform([0 0 -length],[0 0 0]);
body = tryAddVisual(body,"Cylinder",[radius length],T,[0.05 0.05 0.05]);
body = tryAddCollision(body,"Cylinder",[radius length],T);
body = tryAddVisual(body,"Sphere",1.4*radius,eye(4),[0.8 0.1 0.1]);
body = tryAddCollision(body,"Sphere",1.4*radius,eye(4));

end

function body = addSimplifiedGeometry(body,linkParams,dh,linkIndex)

pointsBody = visualPointsInBodyFrame(dh,linkIndex);

if isfield(linkParams,'radius') && isfinite(linkParams.radius)
    radius = linkParams.radius;
else
    radius = 0.04;
end

if isfield(linkParams,'jointRadius') && isfinite(linkParams.jointRadius)
    jointRadius = linkParams.jointRadius;
else
    jointRadius = 1.4*radius;
end

color = [0.8 0.8 0.8];

if isfield(linkParams,'color') && numel(linkParams.color) == 3
    color = linkParams.color;
end

for i = 1:size(pointsBody,1)-1
    p1 = pointsBody(i,:);
    p2 = pointsBody(i+1,:);
    length = norm(p2 - p1);

    if length < 1e-9
        continue;
    end

    T = cylinderTransform(p1,p2);
    body = tryAddVisual(body,"Cylinder",[radius length],T,color);
    body = tryAddCollision(body,"Cylinder",[radius length],T);
end

jointT = eye(4);
jointT(1:3,4) = pointsBody(end,:).';
body = tryAddVisual(body,"Sphere",jointRadius,jointT,color);
body = tryAddCollision(body,"Sphere",jointRadius,jointT);

end

function body = tryAddVisual(body,shape,dimensions,T,color)

try
    addVisual(body,char(shape),dimensions,T,'Color',color);
catch
    try
        addVisual(body,char(shape),dimensions,T);
    catch
    end
end

end

function body = tryAddCollision(body,shape,dimensions,T)

try
    addCollision(body,char(shape),dimensions,T);
catch
end

end

function pointsBody = visualPointsInBodyFrame(dh,linkIndex)

if ~isfield(dh,'visualSegments') || numel(dh.visualSegments) < linkIndex
    pointsBody = [0 0 0];
    return;
end

pointsParent = dh.visualSegments{linkIndex};
TParentBody = dh.fixedTransforms{linkIndex};
pointsHomogeneous = [pointsParent ones(size(pointsParent,1),1)];
pointsBodyHomogeneous = (TParentBody \ pointsHomogeneous.').';
pointsBody = pointsBodyHomogeneous(:,1:3);

end

function components = massComponentsInBodyFrame(dh,linkParams,linkIndex)

components = struct('type',{},'mass',{},'radius',{},'pointA',{},'pointB',{},'center',{});
radius = linkParams.radius;

points = visualPointsInBodyFrame(dh,linkIndex);

if isfield(linkParams,'structuralMass') && linkParams.structuralMass > 0
    segments = pointsToSegments(points);
    lengths = vecnorm(segments(:,4:6) - segments(:,1:3),2,2);
    totalLength = sum(lengths);

    if totalLength > 1e-9
        for i = 1:size(segments,1)
            componentMass = linkParams.structuralMass * lengths(i)/totalLength;
            components(end+1).type = "cylinder"; %#ok<AGROW>
            components(end).mass = componentMass;
            components(end).radius = radius;
            components(end).pointA = segments(i,1:3);
            components(end).pointB = segments(i,4:6);
            components(end).center = 0.5*(segments(i,1:3) + segments(i,4:6));
        end
    end
end

if isfield(linkParams,'jointModuleMass') && linkParams.jointModuleMass > 0
    components(end+1).type = "sphere";
    components(end).mass = linkParams.jointModuleMass;
    components(end).radius = max(radius,linkParams.jointRadius);
    components(end).pointA = [NaN NaN NaN];
    components(end).pointB = [NaN NaN NaN];
    components(end).center = points(end,:);
end

if isempty(components)
    components(1).type = "sphere";
    components(1).mass = linkParams.mass;
    components(1).radius = max(radius,linkParams.jointRadius);
    components(1).pointA = [NaN NaN NaN];
    components(1).pointB = [NaN NaN NaN];
    components(1).center = points(end,:);
end

end

function [com,inertia] = estimateMassProperties(components,defaultRadius)

masses = [components.mass].';
mass = sum(masses);

if mass <= 0
    com = [0 0 0];
    inertia = sphereInertia(1,defaultRadius);
    return;
end

centers = vertcat(components.center);
com = sum(centers.*masses,1)/mass;

inertia = zeros(3,3);

for i = 1:numel(components)
    component = components(i);

    if component.type == "cylinder"
        p1 = component.pointA;
        p2 = component.pointB;
        length = norm(p2 - p1);
        direction = (p2 - p1)/length;
        inertiaCenter = cylinderInertiaAboutCenter( ...
            component.mass, ...
            component.radius, ...
            length, ...
            direction);
    else
        inertiaCenter = sphereInertia(component.mass,component.radius);
    end

    offset = component.center - com;
    inertia = inertia + inertiaCenter + ...
        component.mass * ((offset*offset.')*eye(3) - offset.'*offset);
end

end

function segments = pointsToSegments(points)

segments = zeros(0,6);

for i = 1:size(points,1)-1
    if norm(points(i+1,:) - points(i,:)) > 1e-9
        segments(end+1,:) = [points(i,:) points(i+1,:)]; %#ok<AGROW>
    end
end

end

function inertia = cylinderInertiaAboutCenter(mass,radius,length,direction)

Iaxis = 0.5*mass*radius^2;
Iperp = (1/12)*mass*(3*radius^2 + length^2);
localInertia = diag([Iperp Iperp Iaxis]);
R = frameWithZ(direction(:));
inertia = R*localInertia*R.';

end

function inertia = sphereInertia(mass,radius)

value = (2/5)*mass*radius^2;
inertia = value*eye(3);

end

function T = cylinderTransform(p1,p2)

p1 = p1(:);
p2 = p2(:);
direction = p2 - p1;
length = norm(direction);
center = 0.5*(p1 + p2);

T = eye(4);
T(1:3,1:3) = frameWithZ(direction/length);
T(1:3,4) = center;

end

function R = frameWithZ(zAxis)

zAxis = zAxis/norm(zAxis);

if abs(dot(zAxis,[0;0;1])) < 0.9
    xGuess = [0;0;1];
else
    xGuess = [0;1;0];
end

yAxis = cross(zAxis,xGuess);
yAxis = yAxis/norm(yAxis);
xAxis = cross(yAxis,zAxis);
xAxis = xAxis/norm(xAxis);

R = [xAxis yAxis zAxis];

end
