
%% Phase 1 - Admittance + Robot Integration
% Run design_6axis_admittance first to obtain M and D.

assert(exist('M','var')==1 && exist('D','var')==1, ...
    'Run design_6axis_admittance first.');

robot = loadRobot("robot_ver2");

Ts = 0.002;
t = 0:Ts:1.2;
N = numel(t);

q = zeros(N,6);
q(1,:) = deg2rad([0 60 120 120 0 0]);

W = zeros(6,N);
W(1,t>=0.1 & t<0.5) = 10;  % X force, N

V = zeros(6,N);
qd = zeros(N,6);

T0 = getTransform(robot.model,q(1,:),'tcp');
pDesired = zeros(3,N);
pActual = zeros(3,N);

pDesired(:,1) = T0(1:3,4);
pActual(:,1) = T0(1:3,4);

sigmaMin = zeros(1,N);
lambda = 0.02;   % Initial DLS damping for this test

for k = 2:N

    % 1. Admittance: wrench -> Cartesian twist
    V(:,k) = admittance_6d_tustin( ...
        V(:,k-1), W(:,k-1), W(:,k), M, D, Ts);

    % 2. Robot Jacobian (MATLAB: angular first)
    J = geometricJacobian(robot.model,q(k-1,:),'tcp');

    % Reorder to match admittance: [linear; angular]
    J = J([4:6 1:3],:);

    % 3. Damped least-squares velocity mapping
    dq = J' * ((J*J' + lambda^2*eye(6)) \ V(:,k));

    qd(k,:) = dq';

    % 4. Integrate joint positions
    q(k,:) = q(k-1,:) + Ts*dq';

    limits = robot.params.joints.positionLimits;
    assert(all(q(k,:) >= limits(:,1)') && ...
           all(q(k,:) <= limits(:,2)'), ...
           'Joint limit exceeded at sample %d', k);

    % 5. Actual TCP position from robot FK
    T = getTransform(robot.model,q(k,:),'tcp');
    pActual(:,k) = T(1:3,4);

    % 6. Desired TCP position from admittance
    pDesired(:,k) = integrate_translation_tustin( ...
        pDesired(:,k-1),V(:,k-1),V(:,k),Ts);

    sigmaMin(k) = min(svd(J));
end

%% Evaluate results

% Check sampled configurations for self-collision
sampleIdx = unique([1:10:N N]);
collisions = analyzeCollisions(robot,q(sampleIdx,:));

positionError = vecnorm(pDesired-pActual,2,1);

fprintf('\n=== PHASE 1 RESULTS ===\n');
fprintf('Maximum TCP position error: %.4f mm\n', ...
    1000*max(positionError));
fprintf('Maximum joint speed: %.4f rad/s\n', ...
    max(abs(qd(:))));
fprintf('Minimum Jacobian singular value: %.6f\n', ...
    min(sigmaMin(2:end)));
fprintf('Sampled self-collisions: %d\n', ...
    nnz(collisions.isSelfColliding));

%% Plot desired vs actual TCP X displacement
figure;
plot(t,1000*(pDesired(1,:)-pDesired(1,1)), ...
    'LineWidth',1.5);
hold on;
plot(t,1000*(pActual(1,:)-pActual(1,1)), ...
    '--','LineWidth',1.5);

xlabel('Time [s]');
ylabel('TCP X Displacement [mm]');
title('Admittance Command vs Robot Motion');
legend('Desired','Robot FK');
grid on;

%% Plot joint velocities
figure;
plot(t,qd,'LineWidth',1.2);
xlabel('Time [s]');
ylabel('Joint Velocity [rad/s]');
title('Joint Velocity Requirements');
legend('J1','J2','J3','J4','J5','J6');
grid on;
