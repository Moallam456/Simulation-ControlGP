function poses = pathProgressPose(segment, progress)
%PATHPROGRESSPOSE Evaluate the selected Cartesian geometry and original orientation.
% This planning adapter may use MATLAB geometry helpers. The cubic core is
% independent of all Cartesian and robot data.

if isfield(segment,'exactCircle')
    positions = pathProgressCircleGeometry(segment.exactCircle,progress);
else
    positions = interp1(segment.arcLength,segment.arcGeometry.', ...
        segment.length*progress,'linear').';
end
poses = repmat(eye(4),1,1,numel(progress));
poses(1:3,4,:) = reshape(positions,3,1,[]);
for i = 1:numel(progress)
    s = progress(i);
    if segment.orientationSweep ~= 0
        % Same composition as the original arc's local quaternion helper.
        % Apply the path rotation, then SLERP the remaining base-frame twist.
        pathRotation = axang2rotm([segment.axis,segment.orientationSweep*s]);
        fullRotation = axang2rotm([segment.axis,segment.orientationSweep]);
        leftover = segment.endRotation*(fullRotation*segment.startRotation).';
        rotation = slerpOrientation(eye(3),leftover,s)*pathRotation*segment.startRotation;
    elseif isequal(segment.startRotation,segment.endRotation)
        rotation = segment.startRotation;
    else
        rotation = slerpOrientation(segment.startRotation,segment.endRotation,s);
    end
    poses(1:3,1:3,i) = rotation;
end
end
