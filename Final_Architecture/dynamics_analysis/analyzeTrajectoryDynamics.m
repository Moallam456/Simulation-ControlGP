function results = analyzeTrajectoryDynamics(robot,trajectory,scenario)
% ANALYZETRAJECTORYDYNAMICS Ideal joint-side rigid-body inverse dynamics.
% time: N-by-1 s; q/qd/qdd: N-by-DOF in rad, rad/s, rad/s^2.
if nargin < 3 || ~isstruct(scenario) || ~isfield(scenario,'gravity')
    error('dynamics:MissingGravity','scenario.gravity is required.');
end
if ~isstruct(trajectory) || ~all(isfield(trajectory,{'time','q','qd','qdd'}))
    error('dynamics:InvalidTrajectory','Trajectory needs time, q, qd, and qdd.');
end
time = trajectory.time;
q = trajectory.q;
qd = trajectory.qd;
qdd = trajectory.qdd;
validateConfigurations(q,robot);
[n,dof] = size(q);
if ~isnumeric(time) || ~isequal(size(time),[n 1]) || n < 2 || ...
        any(~isfinite(time)) || any(diff(time) <= 0)
    error('dynamics:InvalidTime','time must be finite, strictly increasing, N-by-1, N>=2.');
end
if ~isnumeric(qd) || ~isequal(size(qd),[n dof]) || any(~isfinite(qd(:))) || ...
        ~isnumeric(qdd) || ~isequal(size(qdd),[n dof]) || any(~isfinite(qdd(:)))
    error('dynamics:InvalidDerivatives','qd and qdd must be finite N-by-DOF matrices.');
end
[model,validation] = prepareDynamicsModel(robot,scenario.gravity);
total = zeros(n,dof);
inertial = zeros(n,dof);
velocity = zeros(n,dof);
gravity = zeros(n,dof);
for k = 1:n
    M = massMatrix(model,q(k,:));
    inertial(k,:) = (M*qdd(k,:).').';
    velocity(k,:) = velocityProduct(model,q(k,:),qd(k,:));
    gravity(k,:) = gravityTorque(model,q(k,:));
    total(k,:) = inverseDynamics(model,q(k,:),qd(k,:),qdd(k,:));
end
results.meta = dynamicsMetadata(robot,model.Gravity,validation,"trajectory inverse dynamics");
results.time = time;
results.state.q = q;
results.state.qd = qd;
results.state.qdd = qdd;
results.torque.total = total;
results.torque.gravity = gravity;
results.torque.inertial = inertial;
results.torque.velocity = velocity;
results.torque.residual = total - inertial - velocity - gravity;
results.power.joint = total.*qd;
results.torqueSpeed.speed = qd;
results.torqueSpeed.torque = total;
[results.summaryTable,results.peakEvents] = summarizeDynamicsResults(results,robot);
end
