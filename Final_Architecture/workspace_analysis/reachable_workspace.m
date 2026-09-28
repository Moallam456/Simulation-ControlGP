function varargout = reachable_workspace()
% REACHABLE_WORKSPACE Run and plot the configured joint-limited workspace.
%
% Command Window:
%   reachable_workspace
%
% The selected robot comes from projectConfig.m. Workspace settings are
% owned by workspaceAnalysis.m.

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'kinematics'), ...
    fullfile(rootDir,'workspace_analysis'));

config = projectConfig();
robot = loadRobot(config.robotID);
assertConfiguredRobot(robot,config.robotID);
result = workspaceAnalysis(robot,"reachable");

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
