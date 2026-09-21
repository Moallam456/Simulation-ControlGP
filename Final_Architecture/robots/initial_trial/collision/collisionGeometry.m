function collision = collisionGeometry(params,structure,chain)

% COLLISIONGEOMETRY Simplified collision geometry for initial_trial.
%
% Geometry is intentionally approximate: cylinders along link segments and
% spheres at joint centers. It is suitable for early simulation plumbing,
% not final collision clearance decisions.

collision.available = true;
collision.source = "estimated cylinders and joint spheres";
collision.bodyNames = [structure.frames.baseStructure;structure.bodyNames];
collision.homeFrames = chain.homeFrames;
collision.geometry = struct([]);

index = 0;

index = index + 1;
collision.geometry(index).bodyName = structure.frames.baseStructure;
collision.geometry(index).type = "cylinder";
collision.geometry(index).radius = params.base.radius;
collision.geometry(index).pointA = chain.baseSegment(1,:);
collision.geometry(index).pointB = chain.baseSegment(end,:);
collision.geometry(index).frame = "body";

for i = 1:structure.dof
    link = params.links(i);

    if isfield(link,'radius') && isfinite(link.radius)
        radius = link.radius;
    else
        radius = 0.04;
    end

    if isfield(link,'jointRadius') && isfinite(link.jointRadius)
        jointRadius = link.jointRadius;
    else
        jointRadius = 1.4*radius;
    end

    pointsBody = chain.bodySegments{i};

    for j = 1:size(pointsBody,1)-1
        p1 = pointsBody(j,:);
        p2 = pointsBody(j+1,:);

        if norm(p2 - p1) < 1e-9
            continue;
        end

        index = index + 1;
        collision.geometry(index).bodyName = structure.bodyNames(i);
        collision.geometry(index).type = "cylinder";
        collision.geometry(index).radius = radius;
        collision.geometry(index).pointA = p1;
        collision.geometry(index).pointB = p2;
        collision.geometry(index).frame = "body";
    end

    index = index + 1;
    collision.geometry(index).bodyName = structure.bodyNames(i);
    collision.geometry(index).type = "sphere";
    collision.geometry(index).radius = jointRadius;
    collision.geometry(index).center = pointsBody(end,:);
    collision.geometry(index).frame = "body";
end

if isfield(params,'tool') && isfield(params.tool,'TFlangeTCP')
    toolLength = abs(params.tool.TFlangeTCP(3,4));

    if isfield(params.tool,'radius') && isfinite(params.tool.radius)
        toolRadius = params.tool.radius;
    else
        toolRadius = 0.025;
    end

    if toolLength > 1e-9
        index = index + 1;
        collision.geometry(index).bodyName = structure.frames.endEffector;
        collision.geometry(index).type = "cylinder";
        collision.geometry(index).radius = toolRadius;
        collision.geometry(index).pointA = [0 0 -toolLength];
        collision.geometry(index).pointB = [0 0 0];
        collision.geometry(index).frame = "body";

        index = index + 1;
        collision.geometry(index).bodyName = structure.frames.endEffector;
        collision.geometry(index).type = "sphere";
        collision.geometry(index).radius = 1.4*toolRadius;
        collision.geometry(index).center = [0 0 0];
        collision.geometry(index).frame = "body";
    end
end

collision.notes = params.notes.missingData;

end
