function session = force_analysis()
% FORCE_ANALYSIS Pose, search, and animate Joint-1 loads. Run without arguments.
% The robot is selected in projectConfig; search/motion settings live here.
gravity = [0 0 -9.81];
searchSampleCount = 1500;
searchSeed = 17;
minimumClearance_m = 0.010;
preferredTrajectory = "JOINT_SWEEP_180";
metricFields = {'bending_Nm','axial_N','radial_N','torsion_Nm'};
metricNames = {'Bending moment (N m)','Axial force (N)','Radial force (N)','Axis torque (N m)'};

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
robot = loadRobot();
[model,validation] = prepareDynamicsModel(robot,gravity);
meta = dynamicsMetadata(robot,gravity,validation,"joint_1_reaction_loads");
meta.label = "Ideal rigid-body Joint-1 resultant loads; not individual bearing loads";
dof = robot.structure.dof;
limits = robot.params.joints.positionLimits;
q = reshape(robot.params.joints.homePosition,1,[]);
result = [];
search = [];
motion = [];
collision = [];
setting = false;
motionViewer = [];
cases = robot.dynamicsCases;
if ~isempty(cases)
    keep = arrayfun(@(c) c.enabled && isfield(c.path,'type'),cases);
    cases = cases(keep);
end

fig = figure('Name',sprintf('%s | Joint 1 forces',robot.id), ...
    'NumberTitle','off','Color','w','Position',[60 50 1450 900]);
setappdata(fig,'ForceAnalysisBusy',false);
label(fig,[.03 .945 .94 .035],sprintf('%s | Joint 1 reaction loads',robot.id),15);
robotAx = axes(fig,'Units','normalized','Position',[.04 .36 .47 .52]);
show(model,q,'Parent',robotAx,'FastUpdate',true,'Frames','off','PreservePlot',false);
grid(robotAx,'on'); axis(robotAx,'equal'); view(robotAx,135,25);
xlabel(robotAx,'X (m)'); ylabel(robotAx,'Y (m)'); zlabel(robotAx,'Z (m)');
poses = repmat(q,2*dof+1,1);
for j = 1:dof
    poses(2*j,j) = limits(j,1);
    poses(2*j+1,j) = limits(j,2);
end
bounds = robotPoseBounds(robot,poses);
reach = max(abs(bounds))+.15;
axis(robotAx,[-reach reach -reach reach -reach reach]);
rotate3d(fig,'on');

panel = uipanel(fig,'Title','Joint angles (deg)','Units','normalized', ...
    'Position',[.56 .62 .41 .29],'BackgroundColor','w');
sliders = gobjects(dof,1); angles = gobjects(dof,1);
listeners = cell(dof,2);
for j = 1:dof
    y = .94-j*.88/dof;
    label(panel,[.02 y .22 .70/dof],char(robot.structure.jointNames(j)),10);
    sliders(j) = uicontrol(panel,'Style','slider','Units','normalized', ...
        'Position',[.25 y .54 .65/dof],'Min',limits(j,1),'Max',limits(j,2), ...
        'Value',q(j),'SliderStep',min([deg2rad([1 10])/diff(limits(j,:));1 1],[],1), ...
        'Callback',@(~,~) sliderChanged(j));
    angles(j) = label(panel,[.81 y .17 .70/dof],sprintf('%+.1f',rad2deg(q(j))),10);
    listeners{j,1} = addlistener(sliders(j),'ContinuousValueChange',@(~,~) sliderChanged(j));
    listeners{j,2} = addlistener(sliders(j),'Value','PostSet',@(~,~) sliderChanged(j));
end
setappdata(fig,'JointForceSliderListeners',listeners);
calculateButton = button(fig,[.56 .56 .18 .045],'Calculate',@calculateStatic);
homeButton = button(fig,[.76 .56 .10 .045],'Home',@() setConfiguration(robot.params.joints.homePosition));
fitButton = button(fig,[.88 .56 .09 .045],'Fit view',@fitView);
status = label(fig,[.56 .51 .41 .04],'Choose a pose, then Calculate.',11);
status.Tag = 'StaticLoadStatus';
tableView = uitable(fig,'Units','normalized','Position',[.56 .32 .41 .18], ...
    'ColumnName',{'Quantity','Value','Units'},'ColumnWidth',{210 260 65}, ...
    'ColumnEditable',[false false false],'Data',cell(0,3));
collisionLabel = label(fig,[.56 .26 .41 .05],'Self-collision not checked.',10);
label(fig,[.56 .215 .41 .04],'Vectors: support-on-arm, in the robot base axes.',10);

label(fig,[.03 .23 .48 .03],sprintf('Static peak search | %d random poses + home/current/limits',searchSampleCount),11);
metricSelect = uicontrol(fig,'Style','popupmenu','Units','normalized', ...
    'Position',[.03 .175 .23 .045],'String',metricNames, ...
    'Tag','StaticLoadMetric','Callback',@(~,~) metricChanged());
searchButton = button(fig,[.28 .175 .23 .045],'Find peak configuration',@runSearch);
searchLabel = label(fig,[.03 .075 .48 .085],'No search yet. Maximum found is not a proven global maximum.',10);
label(fig,[.56 .17 .41 .03],'Dynamic loads along a saved robot trajectory',11);
caseSelect = uicontrol(fig,'Style','popupmenu','Units','normalized', ...
    'Position',[.56 .125 .41 .04],'String',{'No trajectory cases'});
trajectoryButton = button(fig,[.56 .075 .22 .04],'Analyze trajectory',@analyzeMotion);
if isempty(cases)
    set([caseSelect trajectoryButton],'Enable','off');
else
    set(caseSelect,'String',cellstr(string({cases.name})));
    selected = find(string({cases.id})==preferredTrajectory,1);
    if ~isempty(selected), set(caseSelect,'Value',selected); end
end
quality = 'Model-based load study; individual bearing loads require bearing spacing and arrangement.';
if meta.preliminary, quality = ['PRELIMINARY | ' quality]; end
label(fig,[.03 .018 .94 .04],quality,10);
buttons = [calculateButton homeButton fitButton searchButton trajectoryButton];

session.figure = fig;
session.robot = robot;
session.setConfiguration = @setConfiguration;
session.calculate = @calculateStatic;
session.search = @runSearch;
session.analyzeTrajectory = @analyzeMotion;
session.showTrajectorySample = @showMotionSample;
session.getTrajectoryViewer = @getMotionViewer;
session.getState = @getState;
session.close = @closeAll;
set(fig,'CloseRequestFcn',@(~,~) closeAll());

    function sliderChanged(j)
        if setting || getappdata(fig,'ForceAnalysisBusy') || q(j)==sliders(j).Value, return; end
        q(j) = sliders(j).Value;
        updatePose();
    end

    function setConfiguration(value)
        value = reshape(value,1,[]);
        if numel(value)~=dof || any(~isfinite(value)) || ...
                any(value<limits(:,1).'-1e-12 | value>limits(:,2).'+1e-12)
            error('force_analysis:Configuration','Expected a finite, limit-valid joint row in radians.');
        end
        q = max(limits(:,1).',min(limits(:,2).',value));
        setting = true;
        for k = 1:dof, sliders(k).Value = q(k); end
        setting = false;
        updatePose();
    end

    function updatePose()
        for k = 1:dof, angles(k).String = sprintf('%+.1f',rad2deg(q(k))); end
        cameraProperties = {'XLim','YLim','ZLim','CameraPosition','CameraTarget', ...
            'CameraUpVector','CameraViewAngle'};
        cameraValues = get(robotAx,cameraProperties);
        show(model,q,'Parent',robotAx,'FastUpdate',true,'Frames','off','PreservePlot',false);
        set(robotAx,cameraProperties,cameraValues);
        result = []; collision = [];
        tableView.Data = cell(0,3);
        status.String = 'Pose changed. Press Calculate for STATIC loads.';
        collisionLabel.String = 'Self-collision not checked at this pose.';
        drawnow limitrate nocallbacks;
    end

    function calculateStatic()
        lock = lockUI(); %#ok<NASGU>
        status.String = 'Calculating static reactions...'; drawnow;
        result = joint1ReactionLoads(model,q);
        collision = checkPose(q);
        displayResult('STATIC | gravity only');
        fprintf('%s | Joint 1 static resultant: axial %.3f N, radial %.3f N, bending %.3f N m, axis torque %.3f N m\n', ...
            robot.id,result.axial_N,result.radial_N,result.bending_Nm,result.torsion_Nm);
    end

    function fitView()
        anglesView = robotAx.View;
        set(robotAx,'CameraPositionMode','auto','CameraTargetMode','auto', ...
            'CameraViewAngleMode','auto');
        axis(robotAx,robotPoseBounds(robot,q));
        view(robotAx,anglesView);
        drawnow;
    end

    function report = checkPose(value)
        report = analyzeCollisions(robot,value,struct('minimumClearance',minimumClearance_m));
    end

    function displayResult(source)
        tableView.Data = { ...
            'Axial force magnitude',sprintf('%.3f',result.axial_N),'N'; ...
            'Radial force magnitude',sprintf('%.3f',result.radial_N),'N'; ...
            'Bending moment magnitude',sprintf('%.3f',result.bending_Nm),'N m'; ...
            'Axis torque magnitude',sprintf('%.3f',result.torsion_Nm),'N m'; ...
            'Force [Fx Fy Fz]',sprintf('%+.3f  %+.3f  %+.3f',result.forceBase_N),'N'; ...
            'Moment at J1 [Mx My Mz]',sprintf('%+.3f  %+.3f  %+.3f',result.momentAtJointBase_Nm),'N m'};
        status.String = sprintf('%s | supported mass %.3f kg',source,result.supportedMass_kg);
        pair = strjoin(collision.closestBodyPair(1,:),' / ');
        if collision.isSelfColliding(1)
            collisionLabel.String = sprintf('SELF-COLLISION: %s. Free-space loads only; contact forces are NOT modeled.',pair);
            collisionLabel.ForegroundColor = [.75 .05 .05];
        else
            collisionLabel.String = sprintf('Self-collision clear at this sample: %.1f mm | %s. No path/environment check.', ...
                1000*collision.minimumClearance(1),pair);
            collisionLabel.ForegroundColor = [.15 .30 .20];
            if ~collision.hasRequiredClearance(1)
                collisionLabel.String = ['BELOW CLEARANCE MARGIN. ' collisionLabel.String];
                collisionLabel.ForegroundColor = [.7 .3 0];
            end
        end
        drawnow;
    end

    function runSearch(sampleCount)
        if nargin < 1, sampleCount = searchSampleCount; end
        validateattributes(sampleCount,{'numeric'},{'scalar','integer','positive'});
        lock = lockUI(); %#ok<NASGU>
        searchLabel.String = 'Computing sampled static loads...'; drawnow;
        stream = RandStream('mt19937ar','Seed',searchSeed);
        samples = limits(:,1).'+rand(stream,sampleCount,dof).*diff(limits,1,2).';
        samples = unique([q; poses; samples],'rows','stable');
        search = struct('loads',joint1ReactionLoads(model,samples), ...
            'collisionChecked',false(size(samples,1),1), ...
            'accepted',false(size(samples,1),1),'clearance_m',nan(size(samples,1),1), ...
            'minimumClearance_m',minimumClearance_m,'seed',searchSeed, ...
            'maximumIndices',nan(1,numel(metricFields)), ...
            'selectedMetric','','selectedIndex',NaN,'selectedValue',NaN, ...
            'tieTolerance',NaN,'constantAcrossSamples',false);
        reachWorst();
    end

    function metricChanged()
        if getappdata(fig,'ForceAnalysisBusy'), return; end
        if isempty(search)
            runSearch();
        else
            lock = lockUI(); %#ok<NASGU>
            reachWorst();
        end
    end

    function reachWorst()
        metric = metricSelect.Value;
        field = metricFields{metric};
        values = search.loads.(field);
        tolerance = 1e-8*max(1,max(abs(values)));
        constant = max(values)-min(values)<=tolerance;
        [~,order] = sort(values,'descend');
        % Numerical roundoff in a constant load must not decide a worst pose.
        if constant, order = (1:numel(values)).'; end
        search.selectedMetric = field;
        search.maximumIndices(metric) = NaN;
        search.selectedIndex = NaN;
        search.selectedValue = NaN;
        search.tieTolerance = tolerance;
        search.constantAcrossSamples = constant;
        found = [];
        % Descending screening finds the largest valid sampled value without
        % spending mesh-collision checks on every lower-ranked configuration.
        for candidate = order.'
            if ~search.collisionChecked(candidate)
                c = checkPose(search.loads.q(candidate,:));
                search.collisionChecked(candidate) = true;
                search.accepted(candidate) = c.hasRequiredClearance;
                search.clearance_m(candidate) = c.minimumClearance;
            end
            if search.accepted(candidate), found = candidate; break; end
            searchLabel.String = sprintf('Screening ranked poses: %d collision checks...',nnz(search.collisionChecked));
            drawnow limitrate;
        end
        if isempty(found)
            result = [];
            tableView.Data = cell(0,3);
            status.String = 'STATIC | no qualifying sampled pose';
            searchLabel.String = 'No sampled pose passed self-collision and the clearance margin.';
            return;
        end
        search.maximumIndices(metric) = found;
        search.selectedIndex = found;
        search.selectedValue = values(found);
        setConfiguration(search.loads.q(found,:));
        result = joint1ReactionLoads(model,q);
        collision = checkPose(q);
        if constant
            source = 'STATIC | representative tied peak';
            peakDescription = 'Load is constant across sampled poses within tolerance; no unique peak pose.';
        else
            source = 'STATIC | largest qualifying sampled load';
            peakDescription = 'Largest qualifying sampled value; no global maximum proof.';
        end
        displayResult(source);
        searchLabel.String = sprintf(['%d poses; %d collision checks; %.0f mm margin. Not a global proof.\n' ...
            '%s: %.6g. Sampled range %.6g to %.6g.\n%s'], ...
            numel(values),nnz(search.collisionChecked),1000*minimumClearance_m, ...
            metricNames{metric},values(found),min(values),max(values),peakDescription);
    end

    function analyzeMotion(caseID)
        if isempty(cases), return; end
        if nargin > 0
            selected = find(string({cases.id})==string(caseID),1);
            if isempty(selected), error('force_analysis:Case','Unknown trajectory case.'); end
            caseSelect.Value = selected;
        end
        lock = lockUI(); %#ok<NASGU>
        previousStatus = status.String;
        status.String = 'Generating and checking trajectory...'; drawnow;
        motion = [];
        if ~isempty(motionViewer), motionViewer.close(); end
        motionViewer = [];
        selectedCase = cases(caseSelect.Value);
        [trajectory,report] = generateTrajectory(robot,selectedCase);
        if ~report.pass
            status.String = ['Trajectory rejected: ' char(report.reason)];
            warning('force_analysis:TrajectoryRejected','%s',report.reason);
            return;
        end
        c = checkPose(trajectory.q);
        loads = joint1ReactionLoads(model,trajectory.q,trajectory.qd,trajectory.qdd);
        motion = struct('caseID',selectedCase.id,'trajectory',trajectory, ...
            'validation',report,'collision',c,'loads',loads);
        motionViewer = createForceMotionViewer(robot,model,motion);
        status.String = previousStatus;
        motionViewer.play();
    end

    function showMotionSample(sample)
        if isempty(motionViewer) || ~isgraphics(motionViewer.figure)
            error('force_analysis:NoMotion','Analyze a trajectory first.');
        end
        motionViewer.seek(sample);
    end

    function cleanup = lockUI()
        if getappdata(fig,'ForceAnalysisBusy')
            error('force_analysis:Busy','An analysis is already running.');
        end
        setappdata(fig,'ForceAnalysisBusy',true);
        controls = [buttons sliders.' caseSelect metricSelect];
        keepDisabled = gobjects(0);
        if isempty(cases), keepDisabled = [caseSelect trajectoryButton]; end
        set(controls,'Enable','off');
        cleanup = onCleanup(@() unlockControls(fig,controls,keepDisabled));
    end

    function state = getState()
        state = struct('q',q,'result',result,'collision',collision, ...
            'search',search,'motion',motion,'metadata',meta);
        state.motionView = [];
        if ~isempty(motionViewer) && isgraphics(motionViewer.figure)
            state.motionView = motionViewer.getState();
        end
    end

    function viewer = getMotionViewer()
        viewer = motionViewer;
    end

    function closeAll()
        if ~isempty(motionViewer), motionViewer.close(); end
        if isgraphics(fig), delete(fig); end
    end
end

function viewer = createForceMotionViewer(robot,model,motion)
% Playback owns its pose and consumes only precomputed loads/collision data.
t = motion.trajectory.time;
fields = {'bending_Nm','axial_N','radial_N','torsion_Nm'};
names = {'Bending moment (N m)','Axial force (N)','Radial force (N)','Axis torque (N m)'};
index = 1;
playing = false;
rate = 1;
clockStart = [];
timeStart = t(1);
fig = figure('Name',sprintf('%s | Joint 1 trajectory loads',robot.id), ...
    'NumberTitle','off','Color','w','Position',[70 50 1500 880]);
bad = nnz(~motion.collision.hasRequiredClearance);
label(fig,[.03 .95 .94 .035],sprintf('%s | %s | %d/%d samples fail clearance', ...
    robot.id,motion.caseID,bad,numel(t)),13);
robotAx = axes(fig,'Position',[.04 .38 .40 .52]);
show(model,motion.loads.q(1,:),'Parent',robotAx,'FastUpdate',true, ...
    'Frames','off','PreservePlot',false);
grid(robotAx,'on'); axis(robotAx,'equal'); view(robotAx,135,25);
axis(robotAx,robotPoseBounds(robot,motion.loads.q));
xlabel(robotAx,'X (m)'); ylabel(robotAx,'Y (m)'); zlabel(robotAx,'Z (m)');
set(robotAx,'PositionConstraint','outerposition','OuterPosition',[.025 .45 .45 .47]);
rotate3d(fig,'on');
dots = gobjects(4,1);
cursors = gobjects(4,1);
for k = 1:4
    row = floor((k-1)/2); column = mod(k-1,2);
    ax = axes(fig,'Position',[.54+column*.23 .64-row*.33 .19 .23]);
    hold(ax,'on'); grid(ax,'on');
    values = motion.loads.(fields{k});
    plot(ax,t,values,'LineWidth',1.4,'Color',[.12 .4 .68]);
    [peak,peakIndex] = max(values);
    plot(ax,t(peakIndex),peak,'o','Color',[.75 .25 .12]);
    dots(k) = plot(ax,t(1),values(1),'o','MarkerFaceColor',[.12 .4 .68], ...
        'MarkerEdgeColor',[.12 .4 .68],'MarkerSize',6);
    cursors(k) = xline(ax,t(1),':','Color',[.3 .3 .3]);
    xlim(ax,[t(1) t(end)]); title(ax,names{k}); xlabel(ax,'Time (s)');
end
timeInfo = label(fig,[.07 .35 .40 .035],'',11);
info = label(fig,[.04 .285 .43 .065],'',11);
angleInfo = label(fig,[.04 .225 .43 .05],'',10);
collisionInfo = label(fig,[.04 .175 .92 .035],'',10);
scrubber = uicontrol(fig,'Style','slider','Units','normalized', ...
    'Position',[.04 .13 .92 .03],'Min',1,'Max',numel(t),'Value',1, ...
    'SliderStep',[1 min(10,numel(t)-1)]/(numel(t)-1), ...
    'Callback',@(source,~) seek(round(source.Value)));
scrubListener = addlistener(scrubber,'ContinuousValueChange',@(source,~) seek(round(source.Value)));
setappdata(fig,'ForceMotionScrubListener',scrubListener);
playButton = button(fig,[.04 .065 .08 .045],'Play',@togglePlay);
button(fig,[.13 .065 .08 .045],'Restart',@restart);
label(fig,[.225 .066 .045 .035],'Replay',10);
speedMenu = uicontrol(fig,'Style','popupmenu','Units','normalized', ...
    'Position',[.27 .065 .075 .045],'String',{'0.25x','1x','2x'},'Value',2, ...
    'Callback',@(~,~) changeSpeed());
peakNames = {'Peak bending','Peak axial','Peak radial','Peak torque'};
for k = 1:4
    button(fig,[.39+(k-1)*.145 .065 .13 .045],peakNames{k},@() showPeak(k));
end
label(fig,[.04 .012 .92 .035], ...
    'Peaks include all samples. Self-collision checks only; no swept-path, environment, or contact-force calculation.',10);
playTimer = timer('ExecutionMode','fixedSpacing','Period',.04, ...
    'BusyMode','drop','Name','Joint1ForcePlayback','TimerFcn',@tick,'ErrorFcn',@timerError);
set(fig,'CloseRequestFcn',@(~,~) closeViewer(),'DeleteFcn',@(~,~) disposeTimer());
try
    render(1);
catch exception
    closeViewer();
    rethrow(exception);
end
viewer.figure = fig;
viewer.axes = robotAx;
viewer.seek = @seek;
viewer.play = @play;
viewer.pause = @pausePlayback;
viewer.restart = @restart;
viewer.showPeak = @showPeak;
viewer.getState = @getState;
viewer.close = @closeViewer;

    function render(sample)
        if ~isgraphics(fig), return; end
        index = sample;
        cameraProperties = {'XLim','YLim','ZLim','CameraPosition','CameraTarget', ...
            'CameraUpVector','CameraViewAngle'};
        cameraValues = get(robotAx,cameraProperties);
        show(model,motion.loads.q(index,:),'Parent',robotAx,'FastUpdate',true, ...
            'Frames','off','PreservePlot',false);
        set(robotAx,cameraProperties,cameraValues);
        scrubber.Value = index;
        for j = 1:4
            dots(j).XData = t(index);
            dots(j).YData = motion.loads.(fields{j})(index);
            cursors(j).Value = t(index);
        end
        timeInfo.String = sprintf('t %.2f / %.2f s | sample %d / %d',t(index),t(end),index,numel(t));
        info.String = sprintf(['Axial %.3f N   |   Radial %.3f N\n' ...
            'Bending %.3f N m   |   Axis torque %.3f N m'], ...
            motion.loads.axial_N(index),motion.loads.radial_N(index), ...
            motion.loads.bending_Nm(index),motion.loads.torsion_Nm(index));
        angleInfo.String = ['Joint angles (deg): ' sprintf('%+.1f  ',rad2deg(motion.loads.q(index,:)))];
        pair = strjoin(motion.collision.closestBodyPair(index,:),' / ');
        if motion.collision.isSelfColliding(index)
            collisionInfo.String = sprintf('SELF-COLLISION: %s. Displayed loads exclude contact forces.',pair);
            collisionInfo.ForegroundColor = [.75 .05 .05];
        else
            collisionInfo.String = sprintf('Sample clearance %.1f mm | %s | required %.1f mm', ...
                1000*motion.collision.minimumClearance(index),pair, ...
                1000*motion.collision.settings.minimumClearance);
            collisionInfo.ForegroundColor = [.15 .3 .2];
            if ~motion.collision.hasRequiredClearance(index)
                collisionInfo.String = ['BELOW CLEARANCE MARGIN | ' collisionInfo.String];
                collisionInfo.ForegroundColor = [.7 .3 0];
            end
        end
        drawnow limitrate nocallbacks;
    end

    function seek(sample)
        validateattributes(sample,{'numeric'},{'scalar','integer','>=',1,'<=',numel(t)});
        pausePlayback();
        render(sample);
    end

    function play()
        if playing || ~isgraphics(fig), return; end
        if index==numel(t), render(1); end
        timeStart = t(index); clockStart = tic;
        playing = true; playButton.String = 'Pause';
        start(playTimer);
    end

    function pausePlayback()
        playing = false;
        if isvalid(playTimer) && strcmp(playTimer.Running,'on'), stop(playTimer); end
        if isgraphics(playButton), playButton.String = 'Play'; end
    end

    function togglePlay()
        if playing, pausePlayback(); else, play(); end
    end

    function restart()
        seek(1);
        play();
    end

    function changeSpeed()
        rates = [.25 1 2];
        if playing
            target = min(t(end),timeStart+toc(clockStart)*rate);
            render(find(t<=target,1,'last'));
        end
        rate = rates(speedMenu.Value);
        timeStart = t(index); clockStart = tic;
    end

    function tick(~,~)
        if ~playing || ~isgraphics(fig), return; end
        target = min(t(end),timeStart+toc(clockStart)*rate);
        sample = find(t<=target,1,'last');
        if sample~=index, render(sample); end
        if target>=t(end), pausePlayback(); end
    end

    function timerError(~,event)
        pausePlayback();
        warning('force_analysis:Playback','Playback stopped: %s',event.Data.Message);
    end

    function showPeak(metric)
        validateattributes(metric,{'numeric'},{'scalar','integer','>=',1,'<=',4});
        [~,sample] = max(motion.loads.(fields{metric}));
        seek(sample);
    end

    function state = getState()
        state.index = index;
        state.time = t(index);
        state.q = motion.loads.q(index,:);
        state.qd = motion.loads.qd(index,:);
        state.qdd = motion.loads.qdd(index,:);
        state.forceBase_N = motion.loads.forceBase_N(index,:);
        state.momentAtJointBase_Nm = motion.loads.momentAtJointBase_Nm(index,:);
        state.playing = playing;
        state.rate = rate;
        state.cursorTimes = arrayfun(@(h) h.Value,cursors);
    end

    function disposeTimer()
        pausePlayback();
        if isvalid(playTimer), delete(playTimer); end
    end

    function closeViewer()
        disposeTimer();
        if isgraphics(fig), delete(fig); end
    end
end

function h = label(parent,position,value,fontSize)
h = uicontrol(parent,'Style','text','Units','normalized','Position',position, ...
    'String',value,'FontSize',fontSize,'HorizontalAlignment','left','BackgroundColor','w');
end

function h = button(parent,position,value,action)
h = uicontrol(parent,'Style','pushbutton','Units','normalized','Position',position, ...
    'String',value,'Callback',@(~,~) action());
end

function unlockControls(fig,controls,keepDisabled)
if ~isgraphics(fig), return; end
setappdata(fig,'ForceAnalysisBusy',false);
set(controls(isgraphics(controls)),'Enable','on');
set(keepDisabled(isgraphics(keepDisabled)),'Enable','off');
end
