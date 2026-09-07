function crossValidateFK()
%% crossValidateFK.m
% Simulation Team - FK Cross-Validation
%
% PURPOSE
% Compare the Control team's controlFK() against an independently built
% Robotics System Toolbox rigidBodyTree using the SAME shared robot
% parameters in config.UR5().
%
% This script tests the same five joint configurations used by the Control
% team in validateControlIK.m.
%
% PASS means:
%   Control FK and the independent Simulation reference produce the same
%   Base-to-TCP position and orientation for all tested configurations.
%
% Required on MATLAB path:
%   +config/UR5.m
%   controlFK.m
%   dhTransform.m
%
% Requires:
%   Robotics System Toolbox

clear;
clc;
close all;

fprintf('\n');
fprintf('============================================================\n');
fprintf('       SIMULATION TEAM - FK CROSS-VALIDATION\n');
fprintf('============================================================\n\n');

%% 1. Load the shared welding-robot configuration

robot = config.UR5();

n = robot.dof;

assert(n == 6, 'Expected a 6-DOF robot.');

a           = robot.kinematics.a(:);
d           = robot.kinematics.d(:);
alpha       = robot.kinematics.alpha(:);
thetaOffset = robot.kinematics.thetaOffset(:);

assert(numel(a) == n, 'robot.kinematics.a must have DOF entries.');
assert(numel(d) == n, 'robot.kinematics.d must have DOF entries.');
assert(numel(alpha) == n, 'robot.kinematics.alpha must have DOF entries.');
assert(numel(thetaOffset) == n, ...
    'robot.kinematics.thetaOffset must have DOF entries.');

assert(isequal(size(robot.tool.T_F_TCP), [4 4]), ...
    'robot.tool.T_F_TCP must be a 4x4 transform.');

fprintf('[1] Shared configuration loaded: PASS\n');

%% 2. Build an INDEPENDENT Simulation reference model
%
% MATLAB Standard-DH order:
%       [a alpha d theta]
%
% For a revolute rigidBodyJoint, MATLAB ignores the theta value passed to
% setFixedTransform(...,"dh") because theta comes from the joint
% configuration. Therefore each reference configuration will be:
%
%       qReference = q + thetaOffset
%
% This matches Control's:
%
%       theta = q(i) + thetaOffset(i)

referenceRobot = rigidBodyTree( ...
    'DataFormat', 'column', ...
    'MaxNumBodies', n + 1);

parentName = referenceRobot.BaseName;

for i = 1:n

    body = rigidBody(sprintf('link%d', i));
    joint = rigidBodyJoint(sprintf('joint%d', i), 'revolute');

    dhFixed = [ ...
        a(i), ...
        alpha(i), ...
        d(i), ...
        0 ...
    ];

    setFixedTransform(joint, dhFixed, 'dh');

    body.Joint = joint;
    addBody(referenceRobot, body, parentName);

    parentName = body.Name;
end

% Add TCP as a fixed child of the final flange/link frame.
tcpBody = rigidBody('TCP');
tcpJoint = rigidBodyJoint('TCP_fixed', 'fixed');

setFixedTransform(tcpJoint, robot.tool.T_F_TCP);

tcpBody.Joint = tcpJoint;
addBody(referenceRobot, tcpBody, parentName);

fprintf('[2] Independent rigidBodyTree reference built: PASS\n\n');

%% 3. Use the SAME five configurations used by Control

qTestsDeg = [
     30   -45    60    20   -30    45
    -30   -60    80   -20    40    60
     60   -30    40    90   -45   -30
    -60   -75    70    30    45   -60
     15   -90    90     0    30    90
];

qTests = deg2rad(qTestsDeg);

numberOfTests = size(qTests,1);

%% 4. Acceptance tolerances
%
% Both models use identical robot dimensions but independent FK
% implementations, so numerical errors should be extremely small.

positionTolerance    = 1e-9;   % metres
orientationTolerance = 1e-9;   % radians

%% 5. Storage

positionError = zeros(numberOfTests,1);
orientationError = zeros(numberOfTests,1);
maxMatrixDifference = zeros(numberOfTests,1);
passed = false(numberOfTests,1);

TControlStore = zeros(4,4,numberOfTests);
TReferenceStore = zeros(4,4,numberOfTests);

%% 6. FK cross-validation

for k = 1:numberOfTests

    q = qTests(k,:).';

    % ---------------------------------------------------------
    % A) Control-team FK
    % ---------------------------------------------------------
    TControl = controlFK(robot, q);

    % ---------------------------------------------------------
    % B) Independent Simulation reference FK
    % ---------------------------------------------------------
    qReference = q + thetaOffset;

    TReference = getTransform( ...
        referenceRobot, ...
        qReference, ...
        'TCP');

    % Store transformations for later inspection
    TControlStore(:,:,k) = TControl;
    TReferenceStore(:,:,k) = TReference;

    % ---------------------------------------------------------
    % C) Position error
    % ---------------------------------------------------------
    pControl = TControl(1:3,4);
    pReference = TReference(1:3,4);

    positionError(k) = norm(pControl - pReference);

    % ---------------------------------------------------------
    % D) TRUE orientation error
    % ---------------------------------------------------------
    RControl = TControl(1:3,1:3);
    RReference = TReference(1:3,1:3);

    RRelative = RControl * RReference';

    cosTheta = (trace(RRelative) - 1) / 2;
    cosTheta = max(-1, min(1, cosTheta));

    orientationError(k) = acos(cosTheta);

    % ---------------------------------------------------------
    % E) Raw homogeneous-matrix difference
    % ---------------------------------------------------------
    maxMatrixDifference(k) = max( ...
        abs(TControl - TReference), ...
        [], ...
        'all');

    % ---------------------------------------------------------
    % F) PASS / FAIL
    % ---------------------------------------------------------
    passed(k) = ...
        positionError(k) < positionTolerance && ...
        orientationError(k) < orientationTolerance;

end

%% 7. Display compact results table

Test = (1:numberOfTests).';

resultsTable = table( ...
    Test, ...
    positionError, ...
    orientationError, ...
    maxMatrixDifference, ...
    passed, ...
    'VariableNames', { ...
        'Test', ...
        'PositionError_m', ...
        'OrientationError_rad', ...
        'MaxTransformDifference', ...
        'Pass' ...
    });

fprintf('FK CROSS-VALIDATION RESULTS\n');
fprintf('------------------------------------------------------------\n');
disp(resultsTable);

%% 8. Print the first test in detail
%
% This lets us visually confirm that Base->TCP transforms have the same
% translation and rotation, not just rely on the summary table.

fprintf('\n============================================================\n');
fprintf(' DETAILED CHECK - TEST 1\n');
fprintf(' q = [30 -45 60 20 -30 45] deg\n');
fprintf('============================================================\n\n');

fprintf('Control-team T_B_TCP:\n');
disp(TControlStore(:,:,1));

fprintf('Simulation-reference T_B_TCP:\n');
disp(TReferenceStore(:,:,1));

fprintf('Difference (Control - Simulation):\n');
disp(TControlStore(:,:,1) - TReferenceStore(:,:,1));

%% 9. Overall result

fprintf('\n============================================================\n');
fprintf(' FK CROSS-VALIDATION SUMMARY\n');
fprintf('============================================================\n');

fprintf('Tests passed: %d / %d\n', sum(passed), numberOfTests);
fprintf('Worst position error: %.3e m\n', max(positionError));
fprintf('Worst orientation error: %.3e rad\n', max(orientationError));
fprintf('Worst transform-element difference: %.3e\n', ...
    max(maxMatrixDifference));

if all(passed)
    fprintf('\nOVERALL RESULT: PASS\n');
    fprintf(['Control FK and Simulation reference FK agree for all ', ...
             'five configurations.\n']);
else
    fprintf('\nOVERALL RESULT: FAIL\n');
    fprintf('At least one FK result does not match the Simulation model.\n');

    failedTests = find(~passed);
    fprintf('Failed test number(s): ');
    fprintf('%d ', failedTests);
    fprintf('\n');
end

fprintf('============================================================\n\n');
end