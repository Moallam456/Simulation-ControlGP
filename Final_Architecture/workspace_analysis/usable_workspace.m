function varargout = usable_workspace()
% USABLE_WORKSPACE Plot points above the configured manipulability threshold.
%
% Command Window:
%   usable_workspace
%
% Self-colliding configurations are removed before applying the threshold.

% Usable-workspace setting. Change this value to keep more or fewer points.
manipulabilityThreshold = 0.40;

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'kinematics'), ...
    fullfile(rootDir,'workspace_analysis'), ...
    fullfile(rootDir,'analysis_cache'), ...
    fullfile(rootDir,'collision_analysis'));

config = projectConfig();
robot = loadRobot(config.robotID);
assertConfiguredRobot(robot,config.robotID);
settings.manipulabilityThreshold = manipulabilityThreshold;
result = workspaceAnalysis(robot,"usable",settings);

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
