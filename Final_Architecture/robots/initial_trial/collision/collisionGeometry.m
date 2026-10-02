function collision = collisionGeometry(params,structure,chain)

% COLLISIONGEOMETRY Authoritative collision model for initial_trial.
%
% Every transform is expressed from the owning rigid-body frame to the
% collision shape. buildModel attaches these exact definitions.

collision.available = true;
collision.schemaVersion = 1;
collision.source = "estimated cylinders and joint spheres";
collision.status = "PRELIMINARY";
collision.minimumClearance = 0.010; % m, preliminary design margin
collision.bodyNames = [structure.frames.baseStructure;structure.bodyNames; ...
    structure.frames.endEffector];
collision.homeFrames = chain.homeFrames;
collision.geometry = emptyGeometry();

collision.geometry(end+1) = makeCylinder( ...
    structure.frames.baseStructure, ...
    params.base.radius, ...
    chain.baseSegment(1,:), ...
    chain.baseSegment(end,:), ...
    "base");

for i = 1:structure.dof
    link = params.links(i);
    radius = valueOrDefault(link,'radius',0.04);
    jointRadius = valueOrDefault(link,'jointRadius',1.4*radius);
    pointsBody = chain.bodySegments{i};

    for j = 1:size(pointsBody,1)-1
        if norm(pointsBody(j+1,:) - pointsBody(j,:)) < 1e-9
            continue;
        end
        collision.geometry(end+1) = makeCylinder( ...
            structure.bodyNames(i),radius,pointsBody(j,:),pointsBody(j+1,:), ...
            "link segment");
    end

    collision.geometry(end+1) = makeSphere( ...
        structure.bodyNames(i),jointRadius,pointsBody(end,:),"joint housing");
end

toolLength = abs(params.tool.TFlangeTCP(3,4));
toolRadius = valueOrDefault(params.tool,'radius',0.025);
if toolLength > 1e-9
    collision.geometry(end+1) = makeCylinder( ...
        structure.frames.endEffector,toolRadius, ...
        [0 0 -toolLength],[0 0 0],"tool");
    collision.geometry(end+1) = makeSphere( ...
        structure.frames.endEffector,1.4*toolRadius,[0 0 0],"TCP housing");
end

% Adjacent bodies are skipped automatically by the generic collision
% analyser. This extra pair is an intentional overlap in the preliminary
% simplified model: link_5's J6 housing and the TCP tool geometry represent
% connected hardware at the same location.
collision.ignoredSelfCollisionPairs = ["link_5" "tcp"];
collision.notes = [string(params.notes.missingData(:)); ...
    "Collision shapes are preliminary approximations, not CAD envelopes."; ...
    "The 10 mm clearance is a provisional analysis margin."];

validateGeometry(collision,structure);

end

function geometry = emptyGeometry()

geometry = struct( ...
    'bodyName',{}, ...
    'type',{}, ...
    'dimensions',{}, ...
    'transform',{}, ...
    'role',{});

end

function shape = makeCylinder(bodyName,radius,p1,p2,role)

length = norm(p2-p1);
if ~isfinite(radius) || radius <= 0 || ~isfinite(length) || length <= 0
    error('collisionGeometry:InvalidCylinder', ...
        'Cylinder on %s needs positive finite dimensions.',bodyName);
end

shape.bodyName = string(bodyName);
shape.type = "Cylinder";
shape.dimensions = [radius length];
shape.transform = cylinderTransform(p1,p2);
shape.role = string(role);

end

function shape = makeSphere(bodyName,radius,center,role)

if ~isfinite(radius) || radius <= 0
    error('collisionGeometry:InvalidSphere', ...
        'Sphere on %s needs a positive finite radius.',bodyName);
end

T = eye(4);
T(1:3,4) = center(:);
shape.bodyName = string(bodyName);
shape.type = "Sphere";
shape.dimensions = radius;
shape.transform = T;
shape.role = string(role);

end

function value = valueOrDefault(source,fieldName,defaultValue)

if isfield(source,fieldName) && isfinite(source.(fieldName)) && ...
        source.(fieldName) > 0
    value = source.(fieldName);
else
    value = defaultValue;
end

end

function T = cylinderTransform(p1,p2)

p1 = p1(:);
p2 = p2(:);
direction = p2-p1;
length = norm(direction);
center = 0.5*(p1+p2);

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
R = [xAxis/norm(xAxis) yAxis zAxis];

end

function validateGeometry(collision,structure)

knownBodies = [structure.frames.baseStructure;structure.bodyNames; ...
    structure.frames.endEffector];
for i = 1:numel(collision.geometry)
    shape = collision.geometry(i);
    if ~ismember(shape.bodyName,knownBodies)
        error('collisionGeometry:UnknownBody', ...
            'Collision shape %d names unknown body %s.',i,shape.bodyName);
    end
    if ~isequal(size(shape.transform),[4 4]) || ...
            any(~isfinite(shape.transform(:))) || ...
            norm(shape.transform(4,:) - [0 0 0 1]) > 1e-12
        error('collisionGeometry:InvalidTransform', ...
            'Collision shape %d has an invalid transform.',i);
    end
end

end
