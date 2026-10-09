function results = validate_actuator_sizing_cases(robot)
% VALIDATE_ACTUATOR_SIZING_CASES Check the consolidated load-study workflow.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
if nargin < 1 || isempty(robot), robot = loadRobot(); end

options = struct('robot',robot,'showDashboard',true,'visible',false, ...
    'checkConvergence',true,'gravitySamples',25,'gravityStarts',1, ...
    'gravityMaxEvaluations',100);
session = actuator_sizing_analysis(options);
suite = session.suite;

assert(suite.robotID == string(robot.id));
assert(height(suite.jointSummary) == robot.structure.dof);
assert(isfield(suite.meta,'finalSizingAllowed'));
gravityCases = find(string({suite.cases.category}) == "gravity");
assert(isscalar(gravityCases));
assert(suite.cases(gravityCases).status == "PASS" && ~isempty(suite.gravity));

enabled = [suite.definitions.enabled];
for i = 1:numel(suite.cases)
    result = suite.cases(i);
    if ~enabled(i)
        assert(result.status == "DISABLED" && strlength(result.reason) > 0);
        continue;
    end
    if result.category == "visualization"
        assert(result.status == "VISUAL_ONLY" && ...
            result.trajectoryReport.pass && result.convergence.pass);
        assert(~isempty(result.dynamics) && ...
            result.trajectoryReport.collision.summary.collisionCount == 0);
        assert(min(result.trajectoryReport.collision.minimumClearance) >= ...
            suite.definitions(i).validation.minimumClearance_m);
        if result.id == "JOINT_SWEEP_180"
            trajectory = result.trajectory;
            [~,middle] = min(abs(trajectory.q(:,2)));
            assert(abs(trajectory.q(1,2)+pi/2) < 1e-10 && ...
                abs(trajectory.q(end,2)-pi/2) < 1e-10 && ...
                abs(trajectory.q(middle,2)) < 1e-10 && ...
                all(abs(trajectory.qd(middle,[1 3:6])) > 0.01));
        end
        continue;
    end
    assert(result.status == "PASS",'%s failed: %s',result.id,result.reason);
    if result.category == "gravity", continue; end
    trajectory = result.trajectory;
    assert(size(trajectory.q,1) == numel(trajectory.time));
    assert(size(trajectory.q,2) == robot.structure.dof);
    assert(all(isfinite([trajectory.q(:);trajectory.qd(:);trajectory.qdd(:)])));
    assert(result.trajectoryReport.pass && result.convergence.pass);
    assert(isequal(result.dynamics.state.q,trajectory.q));
end

passingMotions = arrayfun(@(x) x.status == "PASS" && ...
    ~isempty(x.dynamics),suite.cases);
assert(any(passingMotions));
assert(numel(suite.aggregate.validCaseIDs) == nnz(passingMotions));
visualIDs = string({suite.cases( ...
    string({suite.cases.category}) == "visualization").id});
assert(~any(ismember(suite.aggregate.validCaseIDs,visualIDs)));
firstMotion = find(passingMotions,1);
session.dashboard.selectCase(firstMotion);
session.dashboard.selectJoint(min(2,robot.structure.dof));
dashboardState = session.dashboard.getState();
assert(dashboardState.caseIndex == firstMotion && ...
    dashboardState.status == "PASS" && isgraphics(session.dashboard.figure));
visualCase = find(string({suite.cases.status}) == "VISUAL_ONLY",1);
if ~isempty(visualCase)
    session.dashboard.selectCase(visualCase);
    visualViewer = session.dashboard.openSelected();
    assert(isgraphics(visualViewer.figure) && ...
        contains(string(visualViewer.figure.Name),"VISUAL ONLY"));
    visualViewer.close();
    unsafe = suite.definitions(visualCase);
    if unsafe.id == "JOINT_SWEEP_180"
        unsafe.path.configurations(:,2) = ...
            linspace(-pi,pi,size(unsafe.path.configurations,1)).';
        unsafe.startConfiguration = unsafe.path.configurations(1,:);
        [~,unsafeReport] = generateTrajectory(robot,unsafe);
        assert(~unsafeReport.pass && ~isempty(unsafeReport.collision) && ...
            unsafeReport.collision.summary.collisionCount > 0);
    end
end
session.dashboard.close();
for j = 1:robot.structure.dof
    points = suite.aggregate.torqueSpeed(j);
    assert(numel(points.speed_rad_s) == numel(points.torque_Nm));
    assert(numel(points.time_s) == numel(points.caseID));
    assert(isfinite(suite.jointSummary.StaticPeakAbs_Nm(j)));
    assert(isfinite(suite.jointSummary.DynamicPeakAbs_Nm(j)));
end

results.pass = true;
results.session = session;
results.suite = suite;
fprintf('Consolidated actuator load-study validation: PASS\n');
end
