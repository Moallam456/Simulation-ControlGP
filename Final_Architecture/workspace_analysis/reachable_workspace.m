function workspace = reachable_workspace(robot,numSamples)

% REACHABLE_WORKSPACE Monte-Carlo TCP workspace for any loaded robot.
%
% Preferred:
%   robot = loadRobot();
%   workspace = reachable_workspace(robot, 50000);
%
% Convenience:
%   workspace = reachable_workspace();
%   workspace = reachable_workspace(10000);

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'kinematics'),fullfile(rootDir,'validation'));

if nargin < 1 || isempty(robot)
    robot = loadRobot();
    numSamples = 50000;
elseif isnumeric(robot)
    numSamples = robot;
    robot = loadRobot();
elseif nargin < 2 || isempty(numSamples)
    numSamples = 50000;
end

points = zeros(numSamples,3);

fprintf('Generating reachable workspace for %s...\n',robot.id);

for i = 1:numSamples
    q = random_configuration(robot);
    T = forward_kinematics(robot,q);
    points(i,:) = T(1:3,4).';
end

fprintf('Workspace generation complete.\n');

workspace.robotID = robot.id;
workspace.points = points;
workspace.numSamples = numSamples;

radius = sqrt(sum(points.^2,2));
workspace.maxReach = max(radius);
workspace.minReach = min(radius);

figure('Name',sprintf('Reachable Workspace - %s',robot.id));

scatter3( ...
    points(:,1), ...
    points(:,2), ...
    points(:,3), ...
    4, ...
    'filled');

grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(sprintf('Reachable Workspace: %s (%d Samples)',robot.id,numSamples), ...
    'Interpreter','none');
view(135,25);

fprintf('Estimated maximum reach = %.3f m\n',workspace.maxReach);
fprintf('Estimated minimum reach = %.3f m\n',workspace.minReach);

end
