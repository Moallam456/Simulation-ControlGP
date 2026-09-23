function session = inspectDynamicsCase(robot,selection,options)
% INSPECTDYNAMICSCASE Run one validated load case and open its viewer.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'kinematics'),fullfile(root,'trajectory'), ...
    fullfile(root,'dynamics_analysis'),fullfile(root,'visualization'), ...
    fullfile(root,'actuator_sizing_cases'));
if nargin < 1 || isempty(robot), robot = loadRobot(); end
if nargin < 2 || isempty(selection)
    error('sizing:MissingCase','Specify a case ID or a case struct.');
end
if nargin < 3, options = struct(); end
if ~isfield(options,'gravity'), options.gravity = [0 0 -9.81]; end
if ~isfield(options,'joint'), options.joint = min(2,robot.structure.dof); end
if ~isfield(options,'playbackRate'), options.playbackRate = 1; end
if ~isfield(options,'autoplay'), options.autoplay = true; end
if ~isfield(options,'visible'), options.visible = true; end
if ~isfield(options,'runOptions'), options.runOptions = struct(); end
if ~isnumeric(options.gravity) || numel(options.gravity) ~= 3 || ...
        any(~isfinite(options.gravity))
    error('sizing:InvalidGravity','gravity must be a finite 1-by-3 vector.');
end

if isstruct(selection)
    if ~isscalar(selection) || ~isfield(selection,'id')
        error('sizing:InvalidCase','Supply one complete case struct.');
    end
    selected = selection;
else
    catalog = createActuatorSizingCases(robot);
    matching = find(string({catalog.id}) == string(selection));
    if numel(matching) ~= 1
        error('sizing:UnknownCase','Unknown or ambiguous case ID: %s',string(selection));
    end
    selected = catalog(matching);
end
scenario.gravity = reshape(options.gravity,1,3);
suite = runActuatorSizingCases(robot,selected,scenario,options.runOptions);
caseResult = suite.cases(1);
session.robotID = string(robot.id);
session.caseID = string(selected.id);
session.status = caseResult.status;
session.reason = caseResult.reason;
session.caseResult = caseResult;
session.suite = suite;
session.viewer = [];
if caseResult.status ~= "PASS"
    fprintf('No dynamics viewer opened for %s: %s\n', ...
        session.caseID,session.reason);
    return;
end

viewerOptions = struct('joint',options.joint, ...
    'playbackRate',options.playbackRate,'autoplay',options.autoplay, ...
    'visible',options.visible);
if string(selected.category) == "gravity"
    peaks = suite.gravity;
    q = peaks.qAtMaxAbs;
    labels = string(robot.structure.jointNames(:)) + " found gravity peak";
    if isfield(robot,'gravityPoses') && ~isempty(robot.gravityPoses)
        q = [q;robot.gravityPoses.q];
        labels = [labels;string(robot.gravityPoses.names(:))];
    end
    poseOptions = struct('gravity',scenario.gravity,'q',q, ...
        'includeCandidatePoses',false);
    poseOptions.mode = "poses";
    poses = analyzeGravityLoading(robot,poseOptions);
    poses.poseNames = labels;
    session.staticPoses = poses;
    viewerOptions.peakPoseCount = robot.structure.dof;
    session.viewer = viewGravityLoading(robot,poses,viewerOptions);
else
    session.viewer = playTrajectoryDynamics(robot, ...
        caseResult.trajectory,caseResult.dynamics,viewerOptions);
end
end
