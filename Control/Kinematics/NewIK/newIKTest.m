%% test_ik.m  --  driver using controlFK for ground truth
clear; clc;

%% --- DH table (a, alpha, d, theta_offset) ---
%  Row = [a_i, alpha_i, d_i, theta_offset_i]
dh = [ 0.150,     0,       0.525,      0;
       0.770,   +pi/2,     0,          0;
       0.100,     0,       0,         pi;
       0,       -pi/2,     0.740,    -pi/2;
       0,       +pi/2,     0,          0;
       0,       -pi/2,     0.100,      0 ];

%% --- Build ONE robot struct for both functions ---
robot.dof = 6;

% Fields the IK reads
robot.a1    = dh(1,1);
robot.a2    = dh(2,1);
robot.a3    = dh(3,1);
robot.d1    = dh(1,3);
robot.d4    = dh(4,3);
robot.dTool = dh(6,3);

% Fields controlFK reads
robot.kinematics.a           = dh(:,1);
robot.kinematics.d           = dh(:,3);
robot.kinematics.alpha       = dh(:,2);
robot.kinematics.thetaOffset = dh(:,4);

% Tool transform: T_F_TCP maps frame 6 to the TCP.
% In our DH, dTool is already inside link 6, so T_F_TCP = identity.
robot.tool.T_F_TCP = eye(4);

%% --- Sanity check: controlFK vs analytic pose at zero config ---
T0 = controlFK(robot, zeros(6,1));
fprintf('controlFK(q=0) TCP = [%.4f %.4f %.4f]\n\n', T0(1:3,4));

%% --- Round-trip test using controlFK ---
rng(0);
N = 500;
nFail = 0;  maxQErr = 0;  maxPoseErr = 0;

fprintf('=== Round-trip test (%d trials, FK = controlFK) ===\n', N);
for k = 1:N
    % random joint config, avoid wrist singularity
    for tries = 1:100
        qTrue = (rand(6,1)-0.5) * 2;      % approx [-1, 1] rad
        if abs(sin(qTrue(5))) > 0.1, break; end
    end

    T06 = controlFK(robot, qTrue);        % <-- ground-truth pose

    [Q, info] = fanuc_arc_mate_100id_ik(T06, robot);

    if isempty(Q)
        nFail = nFail + 1;
        fprintf('  [fail] trial %d: no solutions\n', k);
        continue;
    end

    % joint-space match (mod 2*pi)
    dq = mod(Q - qTrue.' + pi, 2*pi) - pi;
    maxQErr = max(maxQErr, min(vecnorm(dq, 2, 2)));

    % pose-space match, using controlFK for each returned solution
    bestP = inf;
    for i = 1:size(Q,1)
        Ti = controlFK(robot, Q(i,:).');
        dp = norm(Ti(1:3,4)   - T06(1:3,4));
        dR = norm(Ti(1:3,1:3) - T06(1:3,1:3), 'fro');
        bestP = min(bestP, dp + dR);
    end
    maxPoseErr = max(maxPoseErr, bestP);

    if bestP > 1e-8
        fprintf('  [warn] trial %d pose err = %.3e\n', k, bestP);
    end
end

fprintf('\n  Failures        : %d / %d\n', nFail, N);
fprintf('  Max joint err   : %.3e rad\n', maxQErr);
fprintf('  Max pose  err   : %.3e\n', maxPoseErr);

%% --- Edge cases ---
fprintf('\n=== Edge cases ===\n');

% wrist singularity: q5 = 0
qS  = [0.3; -0.4; 0.5; 0.2; 0.0; 0.7];
Ts  = controlFK(robot, qS);
[Qs,~] = fanuc_arc_mate_100id_ik(Ts, robot);
fprintf('  Wrist singular : %d soln(s)\n', size(Qs,1));
if ~isempty(Qs)
    e = inf;
    for i = 1:size(Qs,1)
        Ti = controlFK(robot, Qs(i,:).');
        e = min(e, norm(Ti(1:3,4)-Ts(1:3,4)) + ...
                   norm(Ti(1:3,1:3)-Ts(1:3,1:3),'fro'));
    end
    fprintf('    best err = %.3e\n', e);
end

% unreachable
Tfar = eye(4); Tfar(1:3,4) = [5; 0; 0];
[Qf,~] = fanuc_arc_mate_100id_ik(Tfar, robot);
fprintf('  Unreachable    : %d soln(s) (expect 0)\n', size(Qf,1));

% a specific reachable target from zero joints
T0 = controlFK(robot, zeros(6,1));
[Q0,~] = fanuc_arc_mate_100id_ik(T0, robot);
fprintf('  Zero-config    : %d soln(s)\n', size(Q0,1));
if ~isempty(Q0)
    e = inf;
    for i = 1:size(Q0,1)
        Ti = controlFK(robot, Q0(i,:).');
        e = min(e, norm(Ti(1:3,4)-T0(1:3,4)) + ...
                   norm(Ti(1:3,1:3)-T0(1:3,1:3),'fro'));
    end
    fprintf('    best err = %.3e\n', e);
end

fprintf('\nDone.\n');