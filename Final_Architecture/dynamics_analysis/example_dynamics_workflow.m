function output = example_dynamics_workflow()
% EXAMPLE_DYNAMICS_WORKFLOW Run this after adding Final_Architecture to path.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'dynamics_analysis'));
robot = loadRobot();
scenario.gravity = [0 0 -9.81];
output.validation = validateDynamicModel(robot);
if ~output.validation.pass
    error('example:InvalidModel','%s',strjoin(output.validation.errors,newline));
end
gravityOptions.gravity = scenario.gravity;
gravityOptions.q = robot.params.joints.homePosition;
gravityOptions.numSamples = 100;
gravityOptions.seed = 1;
output.gravity = analyzeGravityLoading(robot,gravityOptions);

% 81 time samples: J2 moves from 0 to +20 degrees in four seconds;
% J1 and J3-J6 stay at home. The control team can supply its own arrays.
time = linspace(0,4,81).';
s = time/4;
blend = 10*s.^3 - 15*s.^4 + 6*s.^5;
blendD = (30*s.^2 - 60*s.^3 + 30*s.^4)/4;
blendDD = (60*s - 180*s.^2 + 120*s.^3)/16;
qStart = robot.params.joints.homePosition;
qFinish = qStart;
qFinish(2) = qFinish(2) + deg2rad(20);
delta = qFinish-qStart;
trajectory.time = time;
trajectory.q = qStart + blend*delta;
trajectory.qd = blendD*delta;
trajectory.qdd = blendDD*delta;
output.inputTrajectory = trajectory;
output.trajectory = analyzeTrajectoryDynamics(robot,trajectory,scenario);
disp(output.trajectory.meta.label);
disp(output.trajectory.summaryTable);
plotDynamicsResults(output.trajectory,struct('joint',2));
end
