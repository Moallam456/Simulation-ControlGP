function report = validate_dynamics_viewer(robot,suite)
% VALIDATE_DYNAMICS_VIEWER Check playback synchronization and case displays.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'visualization'),fullfile(root,'dynamics_analysis'), ...
    fullfile(root,'actuator_sizing_cases'),fullfile(root,'trajectory'));
if nargin < 1 || isempty(robot), robot = loadRobot(); end
if nargin < 2 || isempty(suite)
    checks = validate_actuator_sizing_cases(robot);
    suite = checks.suite;
end

q0 = robot.params.joints.homePosition;
trajectory.time = [0;0.05;0.10];
trajectory.q = repmat(q0,3,1);
trajectory.qd = zeros(3,robot.structure.dof);
trajectory.qdd = zeros(3,robot.structure.dof);
gravity = [0 0 -9.81];
dynamics = analyzeTrajectoryDynamics(robot,trajectory,struct('gravity',gravity));
options = struct('autoplay',false,'visible',false,'joint',2);
viewer = playTrajectoryDynamics(robot,trajectory,dynamics,options);
cleanup = onCleanup(@() closeIfOpen(viewer.figure));
initialGraphics = numel(findall(viewer.figure));
initialLimits = [xlim(viewer.axes);ylim(viewer.axes);zlim(viewer.axes)];
viewer.seek(2);
state = viewer.getState();
report.sampleAligned = state.index == 2 && state.time == trajectory.time(2) && ...
    isequal(state.q,trajectory.q(2,:)) && ...
    isequal(state.qd,trajectory.qd(2,:)) && ...
    isequal(state.qdd,trajectory.qdd(2,:)) && ...
    isequal(state.torque,dynamics.torque.total(2,:)) && ...
    isequal(state.gravity,dynamics.torque.gravity(2,:)) && ...
    isequal(state.inertial,dynamics.torque.inertial(2,:)) && ...
    isequal(state.velocity,dynamics.torque.velocity(2,:)) && ...
    isequal(state.power,dynamics.power.joint(2,:));
viewer.selectJoint(3);
viewer.jumpToPeak();
[~,peakIndex] = max(abs(dynamics.torque.total(:,3)));
report.peakAligned = viewer.getState().index == peakIndex;
viewer.seek(3);
viewer.restart();
report.restartAligned = viewer.getState().index == 1;
report.noGraphicAccumulation = numel(findall(viewer.figure)) == initialGraphics;
report.fixedAxes = isequal([xlim(viewer.axes);ylim(viewer.axes); ...
    zlim(viewer.axes)],initialLimits);
report.robotVisible = ~isempty(findall(viewer.axes,'Type','patch')) || ...
    ~isempty(findall(viewer.axes,'Type','surface'));
viewer.play();
report.playState = viewer.getState().playing;
viewer.pause();
report.pauseState = ~viewer.getState().playing;
viewer.restart();
viewer.play();
pause(0.6);
drawnow;
report.wallClockPlayback = viewer.getState().index == numel(trajectory.time) && ...
    ~viewer.getState().playing;
viewer.close();
clear cleanup
incorrect = dynamics;
incorrect.state.q(1,1) = incorrect.state.q(1,1)+0.01;
try
    mismatchViewer = playTrajectoryDynamics(robot,trajectory,incorrect,options);
    mismatchViewer.close();
    report.rejectMismatchedDynamics = false;
catch ME
    report.rejectMismatchedDynamics = strcmp(ME.identifier,'viewer:ResultMismatch');
end

staticQ = [q0;q0];
staticQ(2,2) = staticQ(2,2)+deg2rad(40);
staticResult = analyzeGravityLoading(robot,struct('gravity',gravity, ...
    'q',staticQ,'includeCandidatePoses',false));
staticResult.poseNames = ["home";"J2 +40 deg"];
staticViewer = viewGravityLoading(robot,staticResult, ...
    struct('visible',false,'joint',2,'initialPose',1));
staticCleanup = onCleanup(@() closeIfOpen(staticViewer.figure));
staticViewer.selectPose(2);
staticState = staticViewer.getState();
model = copy(robot.model);
model.Gravity = gravity;
report.staticAligned = staticState.index == 2 && ...
    isequal(staticState.q,staticQ(2,:)) && ...
    isequal(staticState.torque,staticResult.torque(2,:)) && ...
    norm(staticState.torque-gravityTorque(model,staticState.q),Inf) < 1e-10;
staticViewer.close();
clear staticCleanup
single = analyzeGravityLoading(robot,struct('gravity',gravity, ...
    'q',q0,'includeCandidatePoses',false));
singleViewer = viewGravityLoading(robot,single,struct('visible',false));
report.singleStaticPose = singleViewer.getState().index == 1;
singleViewer.close();

disabled = inspectDynamicsCase(robot,"TRANSFER_MOVE", ...
    struct('visible',false,'autoplay',false));
report.disabledRejected = disabled.status == "DISABLED" && ...
    isempty(disabled.viewer) && strlength(disabled.reason) > 0;

ids = ["LINE_WELD","ARC_WELD","FULL_CIRCLE_WELD","MULTI_SEGMENT"];
report.casePass = false(size(ids));
for i = 1:numel(ids)
    match = find(string({suite.cases.id}) == ids(i),1);
    if isempty(match) || suite.cases(match).status ~= "PASS", continue; end
    caseDynamics = suite.cases(match).dynamics;
    caseTrajectory = suite.cases(match).trajectory;
    caseViewer = playTrajectoryDynamics(robot,caseTrajectory,caseDynamics,options);
    caseCleanup = onCleanup(@() closeIfOpen(caseViewer.figure));
    joint = 2;
    [~,index] = max(abs(caseDynamics.torque.total(:,joint)));
    caseViewer.seek(index);
    atPeak = caseViewer.getState();
    report.casePass(i) = atPeak.index == index && ...
        isequal(atPeak.q,caseTrajectory.q(index,:)) && ...
        isequal(atPeak.torque,caseDynamics.torque.total(index,:)) && ...
        ~isempty(findall(caseViewer.axes,'Type','patch'));
    caseViewer.close();
    clear caseCleanup
end
report.pass = report.sampleAligned && report.peakAligned && ...
    report.restartAligned && report.noGraphicAccumulation && ...
    report.fixedAxes && report.robotVisible && report.playState && ...
    report.pauseState && report.wallClockPlayback && ...
    report.rejectMismatchedDynamics && report.staticAligned && ...
    report.singleStaticPose && ...
    report.disabledRejected && all(report.casePass);
fprintf('Synchronized dynamics viewer validation: %s\n',string(report.pass));
end

function closeIfOpen(fig)
if isgraphics(fig), close(fig); end
end
