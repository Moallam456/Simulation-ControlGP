function results = run_all_validation(numTests)

% RUN_ALL_VALIDATION One-command validation for Final_Architecture.
%
% Usage:
%   results = run_all_validation();
%   results = run_all_validation(200);

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath( ...
    rootDir, ...
    fullfile(rootDir,'kinematics'), ...
    fullfile(rootDir,'validation'), ...
    fullfile(rootDir,'visualization'), ...
    fullfile(rootDir,'workspace_analysis'), ...
    fullfile(rootDir,'dynamics_analysis'));

if nargin < 1 || isempty(numTests)
    numTests = 100;
end

fprintf('\n=====================================\n');
fprintf('FINAL ARCHITECTURE VALIDATION SUITE\n');
fprintf('=====================================\n');

robot = loadRobot();

results.loadRobot = validateLoadRobot(robot);
results.kinematics = validate_kinematics(robot,numTests);
results.workspace = validate_workspace_analysis(robot);
results.dynamics = validate_dynamics_analysis(robot);
results.visualGeometry = validate_robot_visuals(robot);
results.plotSmoke = validatePlotSmoke(robot);

results.success = ...
    results.loadRobot.success && ...
    results.kinematics.success && ...
    results.workspace.success && ...
    results.dynamics.pass && ...
    results.visualGeometry.pass && ...
    results.plotSmoke.success;

fprintf('\n=====================================\n');
fprintf('VALIDATION SUITE SUCCESS: %d\n',results.success);
fprintf('=====================================\n');

if ~results.success
    error('run_all_validation:Failed', ...
        'At least one validation check failed.');
end

end

function result = validateLoadRobot(robot)

config = projectConfig();

result.expectedRobotID = config.robotID;
result.actualRobotID = robot.id;
result.hasParams = isfield(robot,'params');
result.hasStructure = isfield(robot,'structure');
result.hasChain = isfield(robot,'chain');
result.hasRigidBodyTree = isa(robot.model,'rigidBodyTree');
result.hasIK = isa(robot.solveIK,'function_handle');

result.success = result.actualRobotID == result.expectedRobotID && ...
    result.hasParams && ...
    result.hasStructure && ...
    result.hasChain && ...
    result.hasRigidBodyTree && ...
    result.hasIK;

fprintf('loadRobot interface valid: %d\n',result.success);

end

function result = validatePlotSmoke(robot)

oldVisibility = get(0,'DefaultFigureVisible');
set(0,'DefaultFigureVisible','off');
cleanup = onCleanup(@() set(0,'DefaultFigureVisible',oldVisibility));

try
    q = robot.params.joints.homePosition;
    q(2) = q(2) + deg2rad(45);
    plot_robot(robot,q);
    close(gcf);
    result.success = true;
    result.message = "plot_robot executed.";
catch ME
    close all;
    result.success = false;
    result.message = string(ME.message);
end

fprintf('Plot smoke valid: %d\n',result.success);

end
