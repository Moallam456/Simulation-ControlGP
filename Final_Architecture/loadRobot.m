function robot = loadRobot(robotID, params)

% LOADROBOT Assemble the selected robot into one common interface.
%
% Usage:
%   robot = loadRobot()
%   robot = loadRobot("robot_ver1")
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

definitionFile = fullfile(robotFolder,'robotDefinition.m');
if isfile(definitionFile)
    definition = callInRobotFolder(robotFolder,"robotDefinition",params);
    params = definition.params;
    structure = definition.structure;
    chain = definition.chain;
    model = definition.model;
    readiness = definition.readiness;
    collision = callInFolderIfFileExists( ...
        fullfile(robotFolder,'collision'), ...
        'collisionGeometry.m', ...
        "collisionGeometry", ...
        params,structure,chain);
else
    if isempty(params)
        params = callInRobotFolder(robotFolder,"robotParameters");
    end
    structure = callInRobotFolder(robotFolder,"robotStructure");
    chain = callInRobotFolder(robotFolder,"kinematicChain",params,structure);
    collision = callInFolderIfFileExists( ...
        fullfile(robotFolder,'collision'), ...
        'collisionGeometry.m', ...
        "collisionGeometry", ...
        params,structure,chain);
    model = callInRobotFolder( ...
        robotFolder,"buildModel",params,structure,chain,collision);
    readiness = defaultReadiness();
end

actuation = callInFolderIfFileExists( ...
    fullfile(robotFolder,'actuation'), ...
    'actuatorParameters.m', ...
    "actuatorParameters", ...
    structure);

gravityPoses = callInFolderIfFileExists( ...
    robotFolder,'gravityCandidatePoses.m', ...
    "gravityCandidatePoses",params,structure);

robot.id = robotID;
robot.paths.root = rootDir;
robot.paths.robotFolder = robotFolder;
robot.params = params;
robot.structure = structure;
robot.chain = chain;
robot.model = model;
robot.readiness = readiness;
robot.actuation = actuation;
robot.collision = collision;
robot.gravityPoses = gravityPoses;
robot.solveIK = @(targetPose,varargin) solveIK( ...
    robotFolder,targetPose,robot,varargin{:});

end

function robotFolder = robotFolderFor(rootDir,robotID)

switch robotID
    case "robot_ver1"
        robotFolder = fullfile(rootDir,'robots','robot_ver1');
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

function readiness = defaultReadiness()

readiness.status = "READY_FOR_PRELIMINARY_ANALYSIS";
readiness.warnings = strings(0,1);
readiness.joint6Missing = false;
readiness.jointLimitsProvisional = false;
readiness.tcpProvisional = false;
readiness.massPropertiesPreliminary = false;
readiness.actuatorSizingAllowed = true;

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
    "solveInverseKinematics", ...
    targetPose,robot,options);

end
