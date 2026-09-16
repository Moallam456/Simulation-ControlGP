function robot = loadRobot(robotID, params)

% LOADROBOT Assemble the selected robot into one common interface.
%
% Usage:
%   robot = loadRobot()
%   robot = loadRobot("initial_trial")
%   robot = loadRobot("initial_trial", modifiedParams)
%
% Robot-specific code lives under:
%   robots/<robotID>/

rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir,fullfile(rootDir,'kinematics'));

if nargin < 1 || isempty(robotID)
    config = projectConfig();
    robotID = config.robotID;
end

if nargin < 2
    params = [];
end

robotID = lower(string(robotID));
robotFolder = robotFolderFor(rootDir,robotID);

if isempty(params)
    params = callInRobotFolder(robotFolder,"robotParameters");
end

structure = callInRobotFolder(robotFolder,"robotStructure");
dh = callInRobotFolder(robotFolder,"dhParameters",params,structure);
model = callInRobotFolder(robotFolder,"buildModel",params,structure,dh);

actuation = callInFolderIfFileExists( ...
    fullfile(robotFolder,'actuation'), ...
    'actuatorParameters.m', ...
    "actuatorParameters", ...
    structure);

collision = callInFolderIfFileExists( ...
    fullfile(robotFolder,'collision'), ...
    'collisionGeometry.m', ...
    "collisionGeometry", ...
    params,structure,dh);

robot.id = robotID;
robot.paths.root = rootDir;
robot.paths.robotFolder = robotFolder;
robot.params = params;
robot.structure = structure;
robot.dh = dh;
robot.model = model;
robot.actuation = actuation;
robot.collision = collision;
robot.solveIK = @(targetPose,varargin) solveIK( ...
    robotFolder,targetPose,robot,varargin{:});

end

function robotFolder = robotFolderFor(rootDir,robotID)

switch robotID
    case "initial_trial"
        robotFolder = fullfile(rootDir,'robots','initial_trial');
    otherwise
        error('loadRobot:UnknownRobot', ...
            'Unknown robot ID "%s".', robotID);
end

if ~isfolder(robotFolder)
    error('loadRobot:MissingRobotFolder', ...
        'Robot folder does not exist: %s', robotFolder);
end

end

function output = callInRobotFolder(robotFolder,functionName,varargin)

output = withTemporaryPath(robotFolder,@() callByName(functionName,varargin{:}));

end

function output = callInFolderIfFileExists(folder,fileName,functionName,varargin)

if ~isfile(fullfile(folder,fileName))
    output = [];
    return;
end

output = withTemporaryPath(folder,@() callByName(functionName,varargin{:}));

end

function output = withTemporaryPath(folder,callback)

addpath(folder,'-begin');
cleanup = onCleanup(@() rmpath(folder));
output = callback();

end

function output = callByName(functionName,varargin)

functionHandle = str2func(char(functionName));
output = functionHandle(varargin{:});

end

function solutions = solveIK(robotFolder,targetPose,robot,varargin)

if isempty(varargin)
    options = struct();
elseif numel(varargin) == 1
    options = varargin{1};
else
    error('loadRobot:InvalidIKOptions', ...
        'IK accepts at most one options structure.');
end

solutions = callInRobotFolder( ...
    robotFolder, ...
    "inverseKinematics", ...
    targetPose,robot,options);

end
