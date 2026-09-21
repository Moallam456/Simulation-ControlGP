function limits = robotPoseBounds(robot,q)
% ROBOTPOSEBOUNDS Stable cubic bounds enclosing all supplied robot poses.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'kinematics'));
frameCount = robot.structure.dof+2;
points = zeros(size(q,1)*frameCount,3);
for sample = 1:size(q,1)
    [~,frames] = forward_kinematics(robot,q(sample,:));
    origins = cellfun(@(T) T(1:3,4).',frames,'UniformOutput',false);
    rows = (sample-1)*frameCount+(1:frameCount);
    points(rows,:) = vertcat(origins{:});
end
low = min(points,[],1);
high = max(points,[],1);
center = (low+high)/2;
halfWidth = max(0.35,max(high-low)/2+0.12);
limits = [center(1)+[-halfWidth halfWidth], ...
    center(2)+[-halfWidth halfWidth], ...
    center(3)+[-halfWidth halfWidth]];
end
