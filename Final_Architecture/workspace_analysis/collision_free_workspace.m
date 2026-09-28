function varargout = collision_free_workspace()
% COLLISION_FREE_WORKSPACE Plot the collision-workspace pipeline stage.
%
% Collision filtering is intentionally a pass-through until the collision
% analysis is implemented. Results are marked NOT_EVALUATED and must not be
% interpreted as verified collision-free configurations.
%
% Command Window:
%   collision_free_workspace

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'kinematics'), ...
    fullfile(rootDir,'workspace_analysis'));

config = projectConfig();
robot = loadRobot(config.robotID);
assertConfiguredRobot(robot,config.robotID);
result = workspaceAnalysis(robot,"collision_free");

if nargout > 0
    varargout{1} = result;
end

function assertConfiguredRobot(robot,configuredID)

if string(robot.id) ~= string(configuredID)
    error('workspace:ConfiguredRobotMismatch', ...
        ['projectConfig selected "%s", but loadRobot returned "%s". ', ...
         'Clear stale MATLAB paths/functions and try again.'], ...
        string(configuredID),string(robot.id));
end

end

end
