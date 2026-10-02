function varargout = manipulability_workspace()
% MANIPULABILITY_WORKSPACE Plot the configured workspace by manipulability.
%
% Command Window:
%   manipulability_workspace
%
% The selected robot comes from projectConfig.m.

% Manipulability settings. Keep these here because this file owns this
% analysis. The current implementation supports linear inverse condition.
settings.manipulabilityIndex = "inverse-condition";
settings.manipulabilityMotion = "linear";

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'kinematics'), ...
    fullfile(rootDir,'workspace_analysis'), ...
    fullfile(rootDir,'analysis_cache'), ...
    fullfile(rootDir,'collision_analysis'));

config = projectConfig();
robot = loadRobot(config.robotID);
assertConfiguredRobot(robot,config.robotID);
result = workspaceAnalysis(robot,"manipulability",settings);

if nargout > 0
    varargout{1} = result;
end

end

function assertConfiguredRobot(robot,configuredID)

if string(robot.id) ~= string(configuredID)
    error('workspace:ConfiguredRobotMismatch', ...
        ['projectConfig selected "%s", but loadRobot returned "%s". ', ...
         'Clear stale MATLAB paths/functions and try again.'], ...
        string(configuredID),string(robot.id));
end

end
