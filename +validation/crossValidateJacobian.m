function crossValidateJacobian()
% validation.crossValidateJacobian
%
% Simulation Team - Jacobian Cross-Validation
%
% PURPOSE
% Validate the Control team's controlJacobian() against:
%
%   1) MATLAB geometricJacobian() evaluated on the independently built
%      Simulation rigidBodyTree.
%
%   2) A finite-difference Jacobian obtained from actual TCP motion of the
%      Simulation rigidBodyTree.
%
% Control-team convention:
%
%       [v_B; omega_B] = J_control * qdot
%
% MATLAB geometricJacobian convention:
%
%       [omega_B; v_B] = J_MATLAB * qdot
%
% Therefore MATLAB's Jacobian is reordered before comparison.
%
% Required on MATLAB path:
%   +config/UR5.m
%   controlJacobian.m
%   controlFK.m (not required for the reference calculation, but normally
%                    part of the Control kinematics package)
%
% Requires:
%   Robotics System Toolbox

clc;

fprintf('\n');
fprintf('============================================================\n');
fprintf('     SIMULATION TEAM - JACOBIAN CROSS-VALIDATION\n');
fprintf('============================================================\n\n');

%% 1. Load shared robot configuration

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

%% 2. Build independent Simulation rigidBodyTree

referenceRobot = rigidBodyTree( ...
    'DataFormat', 'column', ...
    'MaxNumBodies', n + 1);

parentName = referenceRobot.BaseName;

for i = 1:n

    body = rigidBody(sprintf('link%d', i));
    joint = rigidBodyJoint(sprintf('joint%d', i), 'revolute');

    % Standard DH order in MATLAB: [a alpha d theta]
    % Joint theta is supplied through the robot configuration.
    setFixedTransform( ...
        joint, ...
        [a(i), alpha(i), d(i), 0], ...
        'dh');

    body.Joint = joint;
    addBody(referenceRobot, body, parentName);

    parentName = body.Name;
end

% Fixed flange-to-TCP transform
tcpBody = rigidBody('TCP');
tcpJoint = rigidBodyJoint('TCP_fixed', 'fixed');

setFixedTransform(tcpJoint, robot.tool.T_F_TCP);

tcpBody.Joint = tcpJoint;
addBody(referenceRobot, tcpBody, parentName);

fprintf('[2] Independent rigidBodyTree reference built: PASS\n\n');

%% 3. Same five configurations already used by Control and FK validation

qTestsDeg = [
     30   -45    60    20   -30    45
    -30   -60    80   -20    40    60
     60   -30    40    90   -45   -30
    -60   -75    70    30    45   -60
     15   -90    90     0    30    90
];

qTests = deg2rad(qTestsDeg);

numberOfTests = size(qTests,1);

%% 4. Tolerances

% Direct analytic comparison should agree to near machine precision.
analyticTolerance = 1e-9;

% Finite differences introduce numerical truncation/roundoff error.
finiteDifferenceTolerance = 1e-6;

% Small perturbation used for central differences.
h = 1e-7;  % rad

%% 5. Storage

analyticMaxError = zeros(numberOfTests,1);
finiteDifferenceMaxError = zeros(numberOfTests,1);

analyticPass = false(numberOfTests,1);
finiteDifferencePass = false(numberOfTests,1);

JControlStore = zeros(6,n,numberOfTests);
JReferenceStore = zeros(6,n,numberOfTests);
JFiniteDifferenceStore = zeros(6,n,numberOfTests);

%% 6. Run Jacobian cross-validation

for k = 1:numberOfTests

    q = qTests(k,:).';

    % =========================================================
    % A) CONTROL-TEAM JACOBIAN
    % =========================================================

    JControl = controlJacobian(robot, q);

    assert(isequal(size(JControl), [6 n]), ...
        'controlJacobian must return a 6xDOF matrix.');

    % =========================================================
    % B) INDEPENDENT MATLAB GEOMETRIC JACOBIAN
    % =========================================================

    qReference = q + thetaOffset;

    % MATLAB returns:
    %
    %       [omega; v]
    %
    % Control returns:
    %
    %       [v; omega]
    %
    JMatlab = geometricJacobian( ...
        referenceRobot, ...
        qReference, ...
        'TCP');

    JReference = [
        JMatlab(4:6,:)
        JMatlab(1:3,:)
    ];

    % =========================================================
    % C) ANALYTIC COMPARISON
    % =========================================================

    analyticMaxError(k) = max( ...
        abs(JControl - JReference), ...
        [], ...
        'all');

    analyticPass(k) = analyticMaxError(k) < analyticTolerance;

    % =========================================================
    % D) FINITE-DIFFERENCE JACOBIAN FROM SIMULATION TCP MOTION
    % =========================================================
    %
    % Each joint is perturbed by +/-h.
    %
    % Linear velocity column:
    %
    %   dp/dq_i ~= (p(q+h)-p(q-h))/(2h)
    %
    % Angular velocity column:
    %
    %   Rplus*Rminus' ~= I + 2h*[omega]_x
    %
    % so
    %
    %   omega ~= vee(Rdelta-Rdelta')/(4h)
    %
    % The resulting finite-difference Jacobian uses the same convention:
    %
    %       [v; omega]

    JFD = zeros(6,n);

    for j = 1:n

        qPlus = q;
        qMinus = q;

        qPlus(j) = qPlus(j) + h;
        qMinus(j) = qMinus(j) - h;

        TPlus = getTransform( ...
            referenceRobot, ...
            qPlus + thetaOffset, ...
            'TCP');

        TMinus = getTransform( ...
            referenceRobot, ...
            qMinus + thetaOffset, ...
            'TCP');

        % ----- Linear part -----
        pPlus = TPlus(1:3,4);
        pMinus = TMinus(1:3,4);

        Jv = (pPlus - pMinus) / (2*h);

        % ----- Angular part -----
        RPlus = TPlus(1:3,1:3);
        RMinus = TMinus(1:3,1:3);

        RDelta = RPlus * RMinus';

        Jw = [
            RDelta(3,2) - RDelta(2,3)
            RDelta(1,3) - RDelta(3,1)
            RDelta(2,1) - RDelta(1,2)
        ] / (4*h);

        JFD(:,j) = [
            Jv
            Jw
        ];
    end

    finiteDifferenceMaxError(k) = max( ...
        abs(JControl - JFD), ...
        [], ...
        'all');

    finiteDifferencePass(k) = ...
        finiteDifferenceMaxError(k) < finiteDifferenceTolerance;

    % Store for detailed inspection
    JControlStore(:,:,k) = JControl;
    JReferenceStore(:,:,k) = JReference;
    JFiniteDifferenceStore(:,:,k) = JFD;

end

%% 7. Results table

Test = (1:numberOfTests).';

OverallPass = analyticPass & finiteDifferencePass;

resultsTable = table( ...
    Test, ...
    analyticMaxError, ...
    analyticPass, ...
    finiteDifferenceMaxError, ...
    finiteDifferencePass, ...
    OverallPass, ...
    'VariableNames', { ...
        'Test', ...
        'AnalyticMaxError', ...
        'AnalyticPass', ...
        'FiniteDifferenceMaxError', ...
        'FiniteDifferencePass', ...
        'OverallPass' ...
    });

fprintf('JACOBIAN CROSS-VALIDATION RESULTS\n');
fprintf('------------------------------------------------------------\n');
disp(resultsTable);

%% 8. Detailed inspection of Test 1

fprintf('\n============================================================\n');
fprintf(' DETAILED CHECK - TEST 1\n');
fprintf(' q = [30 -45 60 20 -30 45] deg\n');
fprintf('============================================================\n\n');

fprintf('Control-team Jacobian [v; omega]:\n');
disp(JControlStore(:,:,1));

fprintf('Simulation/MATLAB reference Jacobian [v; omega]:\n');
disp(JReferenceStore(:,:,1));

fprintf('Difference (Control - Simulation analytic):\n');
disp(JControlStore(:,:,1) - JReferenceStore(:,:,1));

fprintf('Finite-difference Jacobian [v; omega]:\n');
disp(JFiniteDifferenceStore(:,:,1));

fprintf('Difference (Control - finite difference):\n');
disp(JControlStore(:,:,1) - JFiniteDifferenceStore(:,:,1));

%% 9. Overall summary

fprintf('\n============================================================\n');
fprintf(' JACOBIAN CROSS-VALIDATION SUMMARY\n');
fprintf('============================================================\n');

fprintf('Analytic comparison passed: %d / %d\n', ...
    sum(analyticPass), numberOfTests);

fprintf('Finite-difference tests passed: %d / %d\n', ...
    sum(finiteDifferencePass), numberOfTests);

fprintf('Worst analytic max error: %.3e\n', ...
    max(analyticMaxError));

fprintf('Worst finite-difference max error: %.3e\n', ...
    max(finiteDifferenceMaxError));

if all(OverallPass)

    fprintf('\nOVERALL RESULT: PASS\n');
    fprintf(['Control Jacobian agrees with both the independent ', ...
             'Simulation analytic Jacobian and finite-difference TCP ', ...
             'motion for all five configurations.\n']);

else

    fprintf('\nOVERALL RESULT: FAIL\n');

    failedTests = find(~OverallPass);

    fprintf('Failed test number(s): ');
    fprintf('%d ', failedTests);
    fprintf('\n');

    fprintf(['Inspect whether the failure is in the analytic comparison, ', ...
             'finite-difference comparison, or both.\n']);

end

fprintf('============================================================\n\n');

end
