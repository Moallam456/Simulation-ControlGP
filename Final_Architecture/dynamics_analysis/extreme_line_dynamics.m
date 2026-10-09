function session = extreme_line_dynamics()
% EXTREME_LINE_DYNAMICS Run the configured robot's outer-reach line viewer.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
robot = loadRobot();

if ~isfield(robot,'dynamicsCases') || isempty(robot.dynamicsCases)
    error('dynamics:NoRobotCases', ...
        ['The robot selected in projectConfig.m does not define ', ...
         'a robot-specific dynamicsTestCases.m file.']);
end
caseIndex = find(string({robot.dynamicsCases.id}) == "EXTREME_LONG_LINE",1);
if isempty(caseIndex)
    error('dynamics:MissingExtremeLine', ...
        'The selected robot does not define an EXTREME_LONG_LINE case.');
end
testCase = robot.dynamicsCases(caseIndex);

fprintf('\n%s\n',testCase.name);
fprintf('Robot: %s | Status: %s | Collision checking: %s\n', ...
    robot.id,testCase.dataStatus,testCase.collisionStatus);
fprintf(['This is preliminary ideal rigid-body inverse dynamics. ', ...
    'The known mesh-collision issue is not used to reject this motion.\n']);

[trajectory,trajectoryReport] = generateTrajectory(robot,testCase);
if ~trajectoryReport.pass
    error('dynamics:ExtremeLineFailed', ...
        'The outer-reach trajectory failed validation: %s',trajectoryReport.reason);
end

scenario.gravity = [0 0 -9.81];
dynamics = analyzeTrajectoryDynamics(robot,trajectory,scenario);
viewerOptions = struct('joint',min(2,robot.structure.dof), ...
    'playbackRate',1,'autoplay',true,'visible',true);
viewer = playTrajectoryDynamics(robot,trajectory,dynamics,viewerOptions);

positions = trajectory.cartesian.position;
horizontalRadius = hypot(positions(:,1),positions(:,2));
fprintf('Path length: %.3f m | Duration: %.3f s\n', ...
    trajectory.meta.pathLength,trajectory.time(end));
fprintf('Horizontal TCP radius: %.3f to %.3f m | Z: %.3f m\n', ...
    min(horizontalRadius),max(horizontalRadius),positions(1,3));
disp(dynamics.summaryTable);

session.robot = robot;
session.case = testCase;
session.trajectory = trajectory;
session.trajectoryReport = trajectoryReport;
session.dynamics = dynamics;
session.viewer = viewer;
session.status = "PRELIMINARY_COLLISION_NOT_CHECKED";

end
