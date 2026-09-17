function figures = plotDynamicsResults(results,options)
% PLOTDYNAMICSRESULTS Engineering views of a trajectory result.
if nargin < 2, options = struct(); end
if ~isfield(options,'joint'), options.joint = []; end
t = results.time;
tau = results.torque.total;
speed = results.state.qd;
power = results.power.joint;
dof = size(tau,2);
names = string(results.summaryTable.Joint);
figures = struct();
figures.torque = figure('Name','Joint torque');
tl = tiledlayout(figures.torque,dof,1,'TileSpacing','compact');
for j = 1:dof
    nexttile(tl); plot(t,tau(:,j)); grid on;
    ylabel(names(j) + " (N*m)");
end
xlabel(tl,'Time (s)');
figures.torqueSpeed = figure('Name','Joint torque-speed');
tl = tiledlayout(figures.torqueSpeed,dof,1,'TileSpacing','compact');
for j = 1:dof
    nexttile(tl); plot(speed(:,j),tau(:,j),'.-'); grid on;
    ylabel(names(j) + " (N*m)");
end
xlabel(tl,'Joint speed (rad/s)');
figures.power = figure('Name','Joint mechanical power');
tl = tiledlayout(figures.power,dof,1,'TileSpacing','compact');
for j = 1:dof
    nexttile(tl); plot(t,power(:,j)); grid on;
    ylabel(names(j) + " (W)");
end
xlabel(tl,'Time (s)');
if ~isempty(options.joint)
    j = options.joint;
    if ~isscalar(j) || j < 1 || j > dof || j ~= floor(j)
        error('dynamics:InvalidJoint','options.joint must index a model joint.');
    end
    figures.decomposition = figure('Name','Torque decomposition');
    plot(t,tau(:,j),t,results.torque.gravity(:,j), ...
        t,results.torque.inertial(:,j),t,results.torque.velocity(:,j));
    grid on; xlabel('Time (s)'); ylabel('Torque (N*m)');
    title(names(j)); legend('Total','Gravity','Inertial','Velocity');
end
end
