function session = actuator_sizing_analysis(options)
% ACTUATOR_SIZING_ANALYSIS Run and present the configured robot load study.
% Normal use requires no inputs: actuator_sizing_analysis

% User-editable study settings.
settings.gravity = [0 0 -9.81];       % m/s^2 in the robot base frame
settings.checkConvergence = true;
settings.convergenceTolerance = 0.08; % 8% change between dt/2 and dt/4
settings.gravitySamples = 200;
settings.gravityStarts = 2;
settings.gravityMaxEvaluations = 300;
settings.seed = 1;
settings.showDashboard = true;
settings.visible = true;
settings.robot = [];

if nargin < 1, options = struct(); end
settings = applyOptions(settings,options);
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
if isempty(settings.robot)
    robot = loadRobot();
else
    robot = settings.robot;
end
validation = validateDynamicModel(robot);
if ~validation.pass
    error('sizing:InvalidDynamicModel','%s',strjoin(validation.errors,newline));
end
if ~isfield(robot,'dynamicsCases') || isempty(robot.dynamicsCases)
    error('sizing:MissingRobotCases', ...
        ['The selected robot needs robots/<robotID>/dynamicsTestCases.m ', ...
         'before a load study can run.']);
end

scenario.gravity = settings.gravity;
suite = runStudy(robot,robot.dynamicsCases,scenario,settings,validation);
printSummary(suite);
dashboard = [];
if settings.showDashboard
    dashboard = createDashboard(robot,suite,settings);
end

session.robot = robot;
session.settings = settings;
session.suite = suite;
session.dashboard = dashboard;
session.status = suite.meta.studyStatus;

end

function settings = applyOptions(settings,options)
if ~isstruct(options) || ~isscalar(options)
    error('sizing:InvalidOptions','Options must be one scalar structure.');
end
names = fieldnames(options);
for i = 1:numel(names)
    if ~isfield(settings,names{i})
        error('sizing:UnknownOption','Unknown option: %s',names{i});
    end
    settings.(names{i}) = options.(names{i});
end
if ~isnumeric(settings.gravity) || numel(settings.gravity) ~= 3 || ...
        any(~isfinite(settings.gravity))
    error('sizing:InvalidGravity','gravity must contain three finite values.');
end
if ~isscalar(settings.convergenceTolerance) || ...
        settings.convergenceTolerance <= 0 || ...
        ~isscalar(settings.gravitySamples) || settings.gravitySamples < 0 || ...
        settings.gravitySamples ~= floor(settings.gravitySamples)
    error('sizing:InvalidSettings','Invalid convergence or gravity settings.');
end
end

function suite = runStudy(robot,cases,scenario,settings,validation)
empty = struct('id',"",'name',"",'category',"",'dataStatus',"", ...
    'collisionStatus',"",'source',"",'status',"",'reason',"", ...
    'trajectory',[],'trajectoryReport',[],'dynamics',[],'convergence',[]);
suite.robotID = string(robot.id);
suite.definitions = cases;
suite.scenario = scenario;
suite.settings = settings;
suite.validation = validation;
suite.cases = repmat(empty,numel(cases),1);
suite.gravity = [];

fprintf('\nRunning actuator load study for %s\n',robot.id);
for i = 1:numel(cases)
    definition = cases(i);
    fprintf('Running %-24s ...\n',definition.id);
    result = empty;
    result.id = string(definition.id);
    result.name = string(definition.name);
    result.category = string(definition.category);
    result.dataStatus = string(definition.dataStatus);
    result.collisionStatus = string(definition.collisionStatus);
    result.source = string(definition.source);
    if ~definition.enabled
        result.status = "DISABLED";
        result.reason = string(definition.disabledReason);
    elseif result.category == "gravity"
        try
            gravityOptions = struct('gravity',scenario.gravity, ...
                'numSamples',settings.gravitySamples, ...
                'numStarts',settings.gravityStarts, ...
                'maxFunctionEvaluations',settings.gravityMaxEvaluations, ...
                'seed',settings.seed);
            suite.gravity = analyzeGravityLoading(robot,gravityOptions);
            result.status = "PASS";
            result.reason = ...
                "Best stationary values found numerically; no global guarantee.";
        catch ME
            result.status = "FAIL";
            result.reason = string(ME.message);
        end
    else
        try
            [result.trajectory,result.trajectoryReport] = ...
                generateTrajectory(robot,definition);
            if ~isempty(result.trajectoryReport.collision)
                if result.trajectoryReport.pass
                    result.collisionStatus = "SAMPLED_CLEAR";
                else
                    result.collisionStatus = "COLLISION_OR_CLEARANCE_FAILED";
                end
            end
            if ~result.trajectoryReport.pass
                result.status = "FAIL";
                result.reason = result.trajectoryReport.reason;
            else
                result.dynamics = analyzeTrajectoryDynamics( ...
                    robot,result.trajectory,scenario);
                if settings.checkConvergence
                    result.convergence = checkConvergence( ...
                        robot,definition,scenario,result.dynamics, ...
                        settings.convergenceTolerance);
                    if result.convergence.pass
                        if result.category == "visualization"
                            result.status = "VISUAL_ONLY";
                            result.reason = ...
                                "Exploratory motion; excluded from motor-sizing peaks.";
                        else
                            result.status = "PASS";
                        end
                    else
                        result.status = "NOT_CONVERGED";
                        result.reason = result.convergence.reason;
                    end
                else
                    result.status = "UNVERIFIED";
                    result.reason = ...
                        "Time-step convergence was disabled; excluded from summary peaks.";
                end
            end
        catch ME
            result.status = "FAIL";
            result.reason = string(ME.message);
        end
    end
    suite.cases(i) = result;
    fprintf('%-24s %-14s %s\n',result.id,result.status,result.reason);
end

suite.aggregate = aggregateResults(robot,suite.cases,suite.gravity);
suite.jointSummary = buildJointSummary(robot,suite.aggregate,suite.gravity);
readinessAllowsSizing = ~isfield(robot,'readiness') || ...
    ~isfield(robot.readiness,'actuatorSizingAllowed') || ...
    robot.readiness.actuatorSizingAllowed;
activeCases = [cases.enabled] & ...
    string({cases.category}) ~= "visualization";
allCaseDataFinal = any(activeCases) && ...
    all(string({cases(activeCases).dataStatus}) == "FINAL");
activeCollisionStatus = string({cases(activeCases).collisionStatus});
allActiveCasesCollisionChecked = all(ismember( ...
    activeCollisionStatus,["CHECKED" "VALIDATED"]));
suite.meta.finalSizingAllowed = readinessAllowsSizing && ...
    ~validation.preliminary && allCaseDataFinal && ...
    allActiveCasesCollisionChecked;
suite.meta.preliminary = ~suite.meta.finalSizingAllowed;
if suite.meta.preliminary
    suite.meta.studyStatus = "PRELIMINARY LOAD STUDY - NOT FINAL MOTOR SIZING";
else
    suite.meta.studyStatus = "VALIDATED LOAD STUDY";
end
suite.meta.assumptions = [ ...
    "Ideal rigid-body joint torque only." ...
    "No friction, gearing, efficiency, rotor inertia, thermal model, or safety margin." ...
    "Per-case collision checks sample poses only; external welding forces are not included."];
end

function report = checkConvergence(robot,testCase,scenario,coarse,tolerance)
report.pass = false;
report.reason = "";
report.timeSteps = testCase.motion.timeStep./[1 2 4];
report.metrics = zeros(3,4,robot.structure.dof);
report.metrics(1,:,:) = reshape(extractMetrics(coarse),1,4,[]);
for level = 2:3
    refined = testCase;
    refined.motion.timeStep = report.timeSteps(level);
    fprintf('  convergence check at dt = %.5g s\n',report.timeSteps(level));
    [trajectory,trajectoryReport] = generateTrajectory(robot,refined);
    if ~trajectoryReport.pass
        report.reason = "Refined trajectory failed: " + trajectoryReport.reason;
        return;
    end
    dynamics = analyzeTrajectoryDynamics(robot,trajectory,scenario);
    report.metrics(level,:,:) = reshape(extractMetrics(dynamics),1,4,[]);
end
new = squeeze(report.metrics(3,:,:));
old = squeeze(report.metrics(2,:,:));
minimumScales = [1e-4;1e-3;1e-2;1e-2];
scale = max(max(abs(new),[],2),minimumScales);
report.relativeChange = abs(new-old)./scale;
report.maxRelativeChange = max(report.relativeChange,[],'all');
report.tolerance = tolerance;
report.pass = all(report.relativeChange(:) <= tolerance);
if ~report.pass
    report.reason = ...
        "Speed, acceleration, torque, or case RMS changed too much after time-step refinement.";
end
end

function metrics = extractMetrics(dynamics)
summary = dynamics.summaryTable;
metrics = [summary.MaxAbsSpeed_rad_s.'; ...
    summary.MaxAbsAcceleration_rad_s2.'; ...
    summary.PeakAbsTorque_Nm.'; ...
    summary.RMSTorqueOverTrajectory_Nm.'];
end

function aggregate = aggregateResults(robot,caseResults,gravity)
dof = robot.structure.dof;
emptyEvent = struct('value',NaN,'caseID',"",'time_s',NaN, ...
    'q',[],'speed_rad_s',NaN,'torque_Nm',NaN,'acceleration_rad_s2',NaN);
emptyJoint = struct('joint',"",'peakPositiveTorque',emptyEvent, ...
    'peakNegativeTorque',emptyEvent,'peakAbsoluteTorque',emptyEvent, ...
    'peakAbsoluteSpeed',emptyEvent,'peakAbsoluteAcceleration',emptyEvent, ...
    'peakPositivePower',emptyEvent,'peakNegativePower',emptyEvent);
aggregate.joints = repmat(emptyJoint,dof,1);
aggregate.torqueSpeed = repmat(struct('speed_rad_s',[], ...
    'torque_Nm',[],'caseID',strings(0,1),'time_s',[]),dof,1);
aggregate.caseComparison = table();
aggregate.validCaseIDs = strings(0,1);
aggregate.gravity = gravity;
for j = 1:dof
    aggregate.joints(j).joint = string(robot.structure.jointNames(j));
end

for i = 1:numel(caseResults)
    result = caseResults(i);
    if result.status ~= "PASS" || isempty(result.dynamics), continue; end
    aggregate.validCaseIDs(end+1,1) = result.id;
    dynamics = result.dynamics;
    for j = 1:dof
        count = numel(dynamics.time);
        paired = aggregate.torqueSpeed(j);
        paired.speed_rad_s = [paired.speed_rad_s;dynamics.state.qd(:,j)];
        paired.torque_Nm = [paired.torque_Nm;dynamics.torque.total(:,j)];
        paired.caseID = [paired.caseID;repmat(result.id,count,1)];
        paired.time_s = [paired.time_s;dynamics.time];
        aggregate.torqueSpeed(j) = paired;
        names = {'peakPositiveTorque','peakNegativeTorque','peakAbsoluteTorque', ...
            'peakAbsoluteSpeed','peakAbsoluteAcceleration', ...
            'peakPositivePower','peakNegativePower'};
        series = {dynamics.torque.total(:,j),dynamics.torque.total(:,j), ...
            abs(dynamics.torque.total(:,j)),abs(dynamics.state.qd(:,j)), ...
            abs(dynamics.state.qdd(:,j)),dynamics.power.joint(:,j), ...
            dynamics.power.joint(:,j)};
        for k = 1:numel(names)
            if ismember(k,[2 7])
                [value,index] = min(series{k});
                if value >= 0, continue; end
                better = isnan(aggregate.joints(j).(names{k}).value) || ...
                    value < aggregate.joints(j).(names{k}).value;
            else
                [value,index] = max(series{k});
                if ismember(k,[1 6]) && value <= 0, continue; end
                better = isnan(aggregate.joints(j).(names{k}).value) || ...
                    value > aggregate.joints(j).(names{k}).value;
            end
            if better
                event = emptyEvent;
                event.value = value;
                event.caseID = result.id;
                event.time_s = dynamics.time(index);
                event.q = dynamics.state.q(index,:);
                event.speed_rad_s = dynamics.state.qd(index,j);
                event.torque_Nm = dynamics.torque.total(index,j);
                event.acceleration_rad_s2 = dynamics.state.qdd(index,j);
                aggregate.joints(j).(names{k}) = event;
            end
        end
    end
    summary = dynamics.summaryTable;
    rows = table(repmat(result.id,dof,1),summary.Joint, ...
        summary.PeakAbsTorque_Nm,summary.RMSTorqueOverTrajectory_Nm, ...
        summary.MaxAbsSpeed_rad_s,summary.PeakAbsPower_W, ...
        'VariableNames',{'CaseID','Joint','PeakAbsTorque_Nm', ...
        'RMSTorque_Nm','PeakAbsSpeed_rad_s','PeakAbsPower_W'});
    aggregate.caseComparison = [aggregate.caseComparison;rows];
end
aggregate.wording = ...
    "Largest values observed in passing test cases; not final actuator requirements.";
aggregate.missionRMSAvailable = false;
end

function summary = buildJointSummary(robot,aggregate,gravity)
dof = robot.structure.dof;
joint = string(robot.structure.jointNames(:));
staticAbs = NaN(dof,1);
staticSigned = NaN(dof,1);
if ~isempty(gravity)
    staticAbs = gravity.summaryTable.MaxFoundAbsGravityTorque_Nm;
    staticSigned = gravity.summaryTable.SignedTorqueAtAbsMax_Nm;
    staticAbs(abs(staticAbs) < 1e-10) = 0;
    staticSigned(abs(staticSigned) < 1e-10) = 0;
end
dynamicAbs = NaN(dof,1);
dynamicCase = repmat("-",dof,1);
largestRMS = NaN(dof,1);
rmsCase = repmat("-",dof,1);
maxSpeed = NaN(dof,1);
maxPower = NaN(dof,1);
for j = 1:dof
    event = aggregate.joints(j).peakAbsoluteTorque;
    dynamicAbs(j) = event.value;
    if strlength(event.caseID) > 0, dynamicCase(j) = event.caseID; end
    if ~isempty(aggregate.caseComparison)
        rows = aggregate.caseComparison.Joint == joint(j);
    else
        rows = false(0,1);
    end
    if any(rows)
        subset = aggregate.caseComparison(rows,:);
        [largestRMS(j),index] = max(subset.RMSTorque_Nm);
        rmsCase(j) = subset.CaseID(index);
        maxSpeed(j) = max(subset.PeakAbsSpeed_rad_s);
        maxPower(j) = max(subset.PeakAbsPower_W);
    end
end
summary = table(joint,staticAbs,staticSigned,dynamicAbs,dynamicCase, ...
    largestRMS,rmsCase,maxSpeed,maxPower, ...
    'VariableNames',{'Joint','StaticPeakAbs_Nm','StaticSignedAtPeak_Nm', ...
    'DynamicPeakAbs_Nm','DynamicPeakCase','LargestCaseRMS_Nm', ...
    'RMSCase','MaxSpeed_rad_s','MaxPower_W'});
end

function printSummary(suite)
fprintf('\n%s\n',suite.meta.studyStatus);
fprintf('Robot: %s\n',suite.robotID);
disp(suite.jointSummary);
fprintf('%s\n',suite.aggregate.wording);
end

function dashboard = createDashboard(robot,suite,settings)
preliminary = suite.meta.preliminary;
bannerColor = [0.98 0.77 0.24];
if ~preliminary, bannerColor = [0.30 0.68 0.43]; end
fig = figure('Name',char(suite.meta.studyStatus + " | " + robot.id), ...
    'NumberTitle','off','Color','w','Position',[50 40 1500 880], ...
    'Visible',matlab.lang.OnOffSwitchState(settings.visible));
uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.02 0.935 0.96 0.05],'String',char(suite.meta.studyStatus), ...
    'BackgroundColor',bannerColor,'FontWeight','bold','FontSize',13);
warningText = strjoin(suite.meta.assumptions,"  ");
if isfield(robot,'readiness') && isfield(robot.readiness,'warnings') && ...
        ~isempty(robot.readiness.warnings)
    warningText = warningText + "  " + strjoin(robot.readiness.warnings,"  ");
end
uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.025 0.885 0.95 0.04],'String',char(warningText), ...
    'BackgroundColor','w','HorizontalAlignment','left','FontSize',9);

caseData = cell(numel(suite.cases),5);
for i = 1:numel(suite.cases)
    caseData(i,:) = {char(suite.cases(i).id),char(suite.cases(i).name), ...
        char(suite.cases(i).status),char(suite.cases(i).dataStatus), ...
        char(suite.cases(i).reason)};
end
caseTable = uitable(fig,'Units','normalized','Position',[0.025 0.63 0.95 0.245], ...
    'Data',caseData,'ColumnName',{'Case','Name','Result','Input status','Reason'}, ...
    'ColumnWidth',{150 220 105 125 650},'RowName',[]);

summary = suite.jointSummary;
jointData = table2cell(summary);
for row = 1:size(jointData,1)
    for column = 1:size(jointData,2)
        if isstring(jointData{row,column})
            jointData{row,column} = char(jointData{row,column});
        end
    end
end
jointTable = uitable(fig,'Units','normalized','Position',[0.025 0.385 0.95 0.225], ...
    'Data',jointData,'ColumnName',{'Joint','Static peak |torque| (N m)', ...
    'Signed static peak (N m)','Moving peak |torque| (N m)', ...
    'Moving peak case','Largest case RMS (N m)','RMS case', ...
    'Max speed (rad/s)','Max power (W)'}, ...
    'ColumnWidth',{100 120 145 130 155 135 145 120 110},'RowName',[]);

caseLabels = string({suite.cases.id}) + " | " + string({suite.cases.status});
defaultCase = find(arrayfun(@(x) x.status == "PASS" && ...
    ~isempty(x.dynamics),suite.cases),1);
if isempty(defaultCase), defaultCase = 1; end
uicontrol(fig,'Style','text','Units','normalized','Position',[0.025 0.335 0.04 0.03], ...
    'String','Case','BackgroundColor','w');
caseMenu = uicontrol(fig,'Style','popupmenu','Units','normalized', ...
    'Position',[0.065 0.332 0.25 0.04],'String',cellstr(caseLabels), ...
    'Value',defaultCase,'Callback',@updatePlots);
uicontrol(fig,'Style','text','Units','normalized','Position',[0.325 0.335 0.04 0.03], ...
    'String','Joint','BackgroundColor','w');
jointMenu = uicontrol(fig,'Style','popupmenu','Units','normalized', ...
    'Position',[0.365 0.332 0.12 0.04], ...
    'String',cellstr(string(robot.structure.jointNames(:))), ...
    'Value',min(2,robot.structure.dof),'Callback',@updatePlots);
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.51 0.329 0.13 0.045],'String','Animate case', ...
    'Callback',@animateCase);
gravityEnabled = 'off';
if ~isempty(suite.gravity), gravityEnabled = 'on'; end
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.65 0.329 0.13 0.045],'String','Gravity poses', ...
    'Enable',gravityEnabled, ...
    'Callback',@showGravity);

torqueAxes = axes('Parent',fig,'Position',[0.06 0.07 0.41 0.23]);
speedAxes = axes('Parent',fig,'Position',[0.56 0.07 0.39 0.23]);
updatePlots();

dashboard.figure = fig;
dashboard.caseTable = caseTable;
dashboard.jointTable = jointTable;
dashboard.torqueAxes = torqueAxes;
dashboard.speedAxes = speedAxes;
dashboard.selectCase = @selectCase;
dashboard.selectJoint = @selectJoint;
dashboard.openSelected = @openSelected;
dashboard.getState = @getState;
dashboard.close = @closeDashboard;

    function updatePlots(~,~)
        caseIndex = caseMenu.Value;
        jointIndex = jointMenu.Value;
        result = suite.cases(caseIndex);
        cla(torqueAxes); hold(torqueAxes,'on'); grid(torqueAxes,'on');
        if ismember(result.status,["PASS" "VISUAL_ONLY"]) && ...
                ~isempty(result.dynamics)
            dynamics = result.dynamics;
            plot(torqueAxes,dynamics.time,dynamics.torque.total(:,jointIndex), ...
                'LineWidth',1.6,'DisplayName','Total');
            plot(torqueAxes,dynamics.time,dynamics.torque.gravity(:,jointIndex), ...
                'LineWidth',1.2,'DisplayName','Gravity');
            plot(torqueAxes,dynamics.time,dynamics.torque.inertial(:,jointIndex), ...
                'LineWidth',1.2,'DisplayName','Inertial');
            plot(torqueAxes,dynamics.time,dynamics.torque.velocity(:,jointIndex), ...
                'LineWidth',1.2,'DisplayName','Velocity');
            legend(torqueAxes,'Location','best');
            xlabel(torqueAxes,'Time (s)'); ylabel(torqueAxes,'Torque (N m)');
        else
            text(torqueAxes,0.5,0.5,char(result.status + ": " + result.reason), ...
                'Units','normalized','HorizontalAlignment','center');
        end
        title(torqueAxes,result.id + " | " + ...
            string(robot.structure.jointNames(jointIndex)),'Interpreter','none');

        cla(speedAxes); hold(speedAxes,'on'); grid(speedAxes,'on');
        for index = 1:numel(suite.cases)
            other = suite.cases(index);
            if ~ismember(other.status,["PASS" "VISUAL_ONLY"]) || ...
                    isempty(other.dynamics), continue; end
            if other.status == "VISUAL_ONLY" && index ~= caseIndex
                continue;
            end
            markerSize = 9;
            if index == caseIndex, markerSize = 18; end
            scatter(speedAxes,other.dynamics.state.qd(:,jointIndex), ...
                other.dynamics.torque.total(:,jointIndex),markerSize,'filled', ...
                'DisplayName',char(other.id));
        end
        if ~isempty(suite.gravity)
            staticTorque = suite.gravity.signedTorqueAtMaxAbs_Nm(jointIndex);
            scatter(speedAxes,0,staticTorque,80,'d','filled', ...
                'DisplayName','Static gravity peak');
        end
        xlabel(speedAxes,'Joint speed (rad/s)'); ylabel(speedAxes,'Torque (N m)');
        title(speedAxes,string(robot.structure.jointNames(jointIndex)) + ...
            " observed torque-speed points",'Interpreter','none');
        if ~isempty(speedAxes.Children)
            legend(speedAxes,'Location','best','Interpreter','none');
        end
    end

    function animateCase(~,~)
        openSelected();
    end

    function viewer = openSelected()
        result = suite.cases(caseMenu.Value);
        if result.category == "gravity"
            viewer = openGravityViewer();
        elseif ismember(result.status,["PASS" "VISUAL_ONLY"]) && ...
                ~isempty(result.dynamics)
            viewerOptions = struct('joint',jointMenu.Value, ...
                'playbackRate',0.25,'autoplay',true,'visible',settings.visible, ...
                'visualOnly',result.status == "VISUAL_ONLY");
            if viewerOptions.visualOnly, viewerOptions.playbackRate = 1; end
            viewer = playTrajectoryDynamics(robot,result.trajectory, ...
                result.dynamics,viewerOptions);
        else
            viewer = [];
            errordlg(char(result.status + ": " + result.reason), ...
                'Case unavailable','modal');
        end
    end

    function showGravity(~,~)
        openGravityViewer();
    end

    function viewer = openGravityViewer()
        if isempty(suite.gravity)
            viewer = [];
            errordlg('No successful gravity result is available.', ...
                'Gravity unavailable','modal');
            return;
        end
        viewerOptions = struct('joint',jointMenu.Value, ...
            'peakPoseCount',robot.structure.dof,'visible',settings.visible);
        viewer = viewGravityLoading(robot,suite.gravity,viewerOptions);
    end

    function selectCase(selection)
        if isstring(selection) || ischar(selection)
            index = find(string({suite.cases.id}) == string(selection),1);
        else
            index = selection;
        end
        if isempty(index) || ~isscalar(index) || index < 1 || ...
                index > numel(suite.cases) || index ~= floor(index)
            error('sizing:InvalidCaseSelection','Unknown case selection.');
        end
        caseMenu.Value = index;
        updatePlots();
    end

    function selectJoint(index)
        if ~isscalar(index) || index < 1 || ...
                index > robot.structure.dof || index ~= floor(index)
            error('sizing:InvalidJointSelection','Joint must be in 1..DOF.');
        end
        jointMenu.Value = index;
        updatePlots();
    end

    function state = getState()
        state.caseIndex = caseMenu.Value;
        state.caseID = suite.cases(caseMenu.Value).id;
        state.joint = jointMenu.Value;
        state.status = suite.cases(caseMenu.Value).status;
    end

    function closeDashboard()
        if isgraphics(fig), close(fig); end
    end
end
