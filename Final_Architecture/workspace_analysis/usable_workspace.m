function varargout = usable_workspace()
% USABLE_WORKSPACE Plot points above the configured manipulability threshold.
%
% Command Window:
%   usable_workspace
%
% Self-colliding configurations are removed before applying the threshold.

% Usable-workspace setting. Change this value to keep more or fewer points.
manipulabilityThreshold = 0.20;
histogramBinWidthDeg = 10; % Joint-angle histogram bin width [deg].
envelopeAlphaRadius_m = NaN; % NaN: minimum all-points radius; larger can erase cavities.
envelopeOpacity = 0.22;     % 0: transparent; 1: opaque. Visual envelope only.

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'kinematics'), ...
    fullfile(rootDir,'workspace_analysis'), ...
    fullfile(rootDir,'analysis_cache'), ...
    fullfile(rootDir,'collision_analysis'));

config = projectConfig();
robot = loadRobot(config.robotID);
assertConfiguredRobot(robot,config.robotID);
settings.manipulabilityThreshold = manipulabilityThreshold;
settings.histogramBinWidthDeg = histogramBinWidthDeg;
settings.envelopeAlphaRadius_m = envelopeAlphaRadius_m;
settings.envelopeOpacity = envelopeOpacity;
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
