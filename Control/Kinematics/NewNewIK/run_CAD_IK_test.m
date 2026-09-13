%% run_CAD_IK_test.m
% Minimal launcher for the CAD robot model and FK/IK test.

clear
clc

% Build the temporary robot and editable length structure.
[robot, L] = buildRobotCAD();

% Show the model used by controlFK.
disp(robot)
disp(L)

% Quick FK sanity check at q = 0.
q0 = zeros(robot.dof,1);
T0 = controlFK(robot,q0);

fprintf('\nT_B_TCP at q = zeros(6,1):\n');
disp(T0)

% Run the previously created IK validation script/function.
%
% If your test file is still a SCRIPT version:
test_analyticalIK_CAD

% If you later convert it to a function version, use:
% results = test_analyticalIK_CAD(robot,L);
