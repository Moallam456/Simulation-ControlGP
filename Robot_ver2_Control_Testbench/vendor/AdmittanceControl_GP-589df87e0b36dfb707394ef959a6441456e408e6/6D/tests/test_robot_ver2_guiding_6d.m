
%% Six-Axis Admittance + Robot Integration
% Requires optimized M, D and w_design in workspace.
% Run design_6axis_admittance first.

assert(exist('M','var')==1 && ...
       exist('D','var')==1 && ...
       exist('w_design','var')==1, ...
       'Run design_6axis_admittance first.');

%% 1. Load robot

robot = loadRobot("robot_ver2");

Ts = 0.002;
t = 0:Ts:1.2;
N = numel(t);

q = zeros(N,6);
q(1,:) = deg2rad([0 60 120 120 0 0]);

qd = zeros(N,6);

%% 2. Simulated six-axis operator wrench

W = zeros(6,N);

% Apply force and torque
W(:,t>=0.1 & t<0.5) = ...
    repmat(w_design,1,nnz(t>=0.1 & t<0.5));

% Reverse force and torque
W(:,t>=0.5 & t<0.75) = ...
    repmat(-w_design,1,nnz(t>=0.5 & t<0.75));

% Release after 0.75 seconds

%% 3. Initialize Cartesian motion

V = zeros(6,N);

T0 = getTransform(robot.model,q(1,:),'tcp');

pDesired = zeros(3,N);
pActual = zeros(3,N);

pDesired(:,1) = T0(1:3,4);
pActual(:,1) = T0(1:3,4);

RDesired = zeros(3,3,N);
RDesired(:,:,1) = T0(1:3,1:3);

orientationError = zeros(1,N);
sigmaMin = zeros(1,N);

lambda = 0.02;

%% 4. Run robot simulation at 500 Hz

for k = 2:N

    % Admittance controller
    V(:,k) = admittance_6d_tustin( ...
        V(:,k-1),W(:,k-1),W(:,k),M,D,Ts);

    % Robot Jacobian
    J = geometricJacobian(robot.model,q(k-1,:),'tcp');

    % MATLAB Jacobian is [angular; linear]
    % Controller twist is [linear; angular]
    J = J([4:6 1:3],:);

    sigmaMin(k) = min(svd(J));

    % Damped least-squares mapping
    dq = J' * ((J*J' + lambda^2*eye(6)) \ V(:,k));

    qd(k,:) = dq';

    % Update joint positions
    q(k,:) = q(k-1,:) + Ts*dq';

    % Check configured joint limits
    limits = robot.params.joints.positionLimits;

    assert(all(q(k,:) >= limits(:,1)') && ...
           all(q(k,:) <= limits(:,2)'), ...
           'Joint limit exceeded at sample %d',k);

    % Actual robot TCP pose
    T = getTransform(robot.model,q(k,:),'tcp');

    pActual(:,k) = T(1:3,4);
    Ractual = T(1:3,1:3);

    % Desired Cartesian position
    pDesired(:,k) = integrate_translation_tustin( ...
        pDesired(:,k-1),V(:,k-1),V(:,k),Ts);

    % Desired Cartesian orientation
    RDesired(:,:,k) = integrate_orientation_so3( ...
        RDesired(:,:,k-1), ...
        V(4:6,k-1),V(4:6,k),Ts);

    % Orientation tracking error
    Rerror = RDesired(:,:,k)'*Ractual;

    orientationError(k) = acos(max(-1,min(1, ...
        (trace(Rerror)-1)/2)));

end

%% 5. Evaluate robot motion

positionError = vecnorm(pDesired-pActual,2,1);

qdd = [zeros(1,6);diff(qd,1,1)/Ts];

sampleIdx = unique([1:10:N N]);
collisions = analyzeCollisions(robot,q(sampleIdx,:));

fprintf('\n=== SIX-AXIS ROBOT TEST ===\n');

fprintf('Maximum TCP position error: %.4f mm\n', ...
    1000*max(positionError));

fprintf('Maximum orientation error: %.4f deg\n', ...
    rad2deg(max(orientationError)));

fprintf('Maximum joint speed: %.4f rad/s\n', ...
    max(abs(qd(:))));

fprintf('Maximum joint acceleration: %.4f rad/s^2\n', ...
    max(abs(qdd(:))));

fprintf('Minimum Jacobian singular value: %.6f\n', ...
    min(sigmaMin(2:end)));

fprintf('Sampled self-collisions: %d\n', ...
    nnz(collisions.isSelfColliding));

%% 6. Plot results

figure;
tiledlayout(2,1);

nexttile;
plot(t,1000*(pDesired-pActual)','LineWidth',1.2);
xlabel('Time [s]');
ylabel('Position Error [mm]');
title('Cartesian Position Tracking Error');
legend('X','Y','Z');
grid on;

nexttile;
plot(t,rad2deg(orientationError),'LineWidth',1.5);
xlabel('Time [s]');
ylabel('Orientation Error [deg]');
title('TCP Orientation Tracking Error');
grid on;

figure;
plot(t,qd,'LineWidth',1.2);
xlabel('Time [s]');
ylabel('Joint Velocity [rad/s]');
title('Six-Axis Joint Velocity Requirements');
legend('J1','J2','J3','J4','J5','J6');
grid on;
