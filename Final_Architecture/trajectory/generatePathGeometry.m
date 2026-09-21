function geometry = generatePathGeometry(path)
% GENERATEPATHGEOMETRY Resolve one line, three-point arc, or full circle.
type = lower(string(path.type));
geometry.type = type;
geometry.startPosition = point(path.startPosition);
geometry.startOrientation = rotation(path.startOrientation);
geometry.orientationMode = "constant";
if isfield(path,'orientationMode')
    geometry.orientationMode = lower(string(path.orientationMode));
end
geometry.endOrientation = geometry.startOrientation;
if isfield(path,'endOrientation') && ~isempty(path.endOrientation)
    geometry.endOrientation = rotation(path.endOrientation);
end
switch type
    case "line"
        geometry.endPosition = point(path.endPosition);
        geometry.length = norm(geometry.endPosition-geometry.startPosition);
    case {"arc","fullcircle"}
        if type == "arc"
            p2 = point(path.midPosition);
            p3 = point(path.endPosition);
        else
            p2 = point(path.point2);
            p3 = point(path.point3);
        end
        p1 = geometry.startPosition;
        normal = cross(p2-p1,p3-p1);
        if norm(normal) < 1e-10
            error('trajectory:DegenerateCircle','Three circle points must be noncollinear.');
        end
        axis = normal/norm(normal);
        A = [(p2-p1).';(p3-p1).';axis.'];
        rhs = [0.5*(p2.'*p2-p1.'*p1); ...
            0.5*(p3.'*p3-p1.'*p1);axis.'*p1];
        center = A\rhs;
        radius = norm(p1-center);
        e1 = (p1-center)/radius;
        e2 = cross(axis,e1);
        if type == "arc"
            angleMid = mod(atan2(dot(p2-center,e2),dot(p2-center,e1)),2*pi);
            angleEnd = mod(atan2(dot(p3-center,e2),dot(p3-center,e1)),2*pi);
            if angleMid < 1e-10 || angleMid > 2*pi-1e-10 || ...
                    angleEnd < 1e-10 || abs(angleMid-angleEnd) < 1e-10
                error('trajectory:DegenerateArc','Arc start, mid, and end must be distinct.');
            end
            if angleMid < angleEnd
                sweep = angleEnd;
            else
                sweep = angleEnd-2*pi;
            end
        else
            if ~isfield(path,'direction') || ~ismember(path.direction,[-1 1])
                error('trajectory:InvalidCircleDirection','direction must be +1 or -1.');
            end
            sweep = path.direction*2*pi;
        end
        geometry.center = center;
        geometry.radius = radius;
        geometry.axis = axis;
        geometry.e1 = e1;
        geometry.e2 = e2;
        geometry.sweep = sweep;
        geometry.length = radius*abs(sweep);
        geometry.endPosition = center+radius*(cos(sweep)*e1+sin(sweep)*e2);
    otherwise
        error('trajectory:UnknownPath','Unsupported path type: %s',type);
end
if geometry.length <= 1e-9
    error('trajectory:ZeroLength','Path length must be positive.');
end
if ~ismember(geometry.orientationMode,["constant","slerp","followarc"])
    error('trajectory:InvalidOrientationMode','Unknown orientation mode.');
end
if geometry.orientationMode=="constant" && ...
        norm(geometry.endOrientation-geometry.startOrientation,'fro')>1e-8
    error('trajectory:ConflictingOrientation', ...
        'constant orientation cannot have a different end orientation.');
end
if type=="fullcircle" && ...
        norm(geometry.endOrientation-geometry.startOrientation,'fro')>1e-8
    error('trajectory:OpenCircleOrientation', ...
        'Full-circle orientation must close at the starting orientation.');
end
end

function p = point(value)
if ~isnumeric(value) || numel(value) ~= 3 || any(~isfinite(value))
    error('trajectory:InvalidPoint','Path points must contain three finite meters.');
end
p = value(:);
end

function R = rotation(value)
if ~isequal(size(value),[3 3]) || any(~isfinite(value(:))) || ...
        norm(value.'*value-eye(3),'fro') > 1e-8 || det(value) < 0
    error('trajectory:InvalidOrientation','Orientation must be a proper 3-by-3 rotation.');
end
R = value;
end
