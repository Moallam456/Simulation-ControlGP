function [poses,positions] = evaluatePathGeometry(geometry,s)
% EVALUATEPATHGEOMETRY Evaluate an arc-length-normalized Cartesian path.
s = s(:);
if any(~isfinite(s)) || any(s < -1e-10 | s > 1+1e-10)
    error('trajectory:InvalidProgress','Path progress must lie in [0,1].');
end
s = min(max(s,0),1);
n = numel(s);
poses = repmat(eye(4),1,1,n);
positions = zeros(n,3);
R0 = geometry.startOrientation;
switch geometry.orientationMode
    case "slerp"
        leftover = geometry.endOrientation*R0.';
    case "followarc"
        if ~ismember(geometry.type,["arc","fullcircle"])
            error('trajectory:FollowArcOnLine','followArc requires an arc or circle.');
        end
        predictedEnd = axang2rotm([geometry.axis.' geometry.sweep])*R0;
        leftover = geometry.endOrientation*predictedEnd.';
    otherwise
        leftover = eye(3);
end
leftoverAxisAngle = rotm2axang(leftover);
for k = 1:n
    switch geometry.type
        case "line"
            p = geometry.startPosition + s(k)*( ...
                geometry.endPosition-geometry.startPosition);
            pathRotation = R0;
        otherwise
            theta = geometry.sweep*s(k);
            p = geometry.center+geometry.radius*( ...
                cos(theta)*geometry.e1+sin(theta)*geometry.e2);
            pathRotation = axang2rotm([geometry.axis.' theta])*R0;
    end
    if geometry.orientationMode == "constant"
        R = R0;
    else
        correction = axang2rotm([leftoverAxisAngle(1:3) ...
            s(k)*leftoverAxisAngle(4)]);
        R = correction*pathRotation;
    end
    poses(1:3,1:3,k) = R;
    poses(1:3,4,k) = p;
    positions(k,:) = p.';
end
end
