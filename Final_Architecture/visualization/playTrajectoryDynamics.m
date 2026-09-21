function viewer = playTrajectoryDynamics(robot,trajectory,dynamics,options)
% PLAYTRAJECTORYDYNAMICS Replay precomputed motion and joint-side loads.
% Playback does not apply torque or recalculate inverse dynamics.
if nargin < 4, options = struct(); end
if ~isfield(options,'joint'), options.joint = min(2,robot.structure.dof); end
if ~isfield(options,'playbackRate'), options.playbackRate = 1; end
if ~isfield(options,'autoplay'), options.autoplay = true; end
if ~isfield(options,'visible'), options.visible = true; end
validateInputs(robot,trajectory,dynamics,options);
qualityLabel = "";
if dynamics.meta.preliminary, qualityLabel = "PRELIMINARY "; end

t = trajectory.time;
q = trajectory.q;
tau = dynamics.torque.total;
dof = robot.structure.dof;
names = string(robot.structure.jointNames(:));
selectedJoint = options.joint;
rate = options.playbackRate;
currentIndex = 1;
playStartTime = t(1);
playClock = [];
isPlaying = false;

fig = figure('Name',char(qualityLabel + string(robot.id) + " dynamics playback"), ...
    'NumberTitle','off','Color','w','Position',[80 60 1450 840], ...
    'Visible',matlab.lang.OnOffSwitchState(options.visible), ...
    'CloseRequestFcn',@closeViewer);
robotAx = axes('Parent',fig,'Position',[0.04 0.31 0.42 0.63]);
show(robot.model,q(1,:),'Parent',robotAx,'Frames','off', ...
    'PreservePlot',false);
grid(robotAx,'on'); axis(robotAx,'equal'); view(robotAx,135,25);
xlabel(robotAx,'X (m)'); ylabel(robotAx,'Y (m)'); zlabel(robotAx,'Z (m)');
robotLimits = robotPoseBounds(robot,q);
axis(robotAx,robotLimits);

torqueAxes = gobjects(dof,1);
torqueLines = gobjects(dof,1);
currentDots = gobjects(dof,1);
for j = 1:dof
    row = ceil(j/2);
    col = mod(j-1,2);
    position = [0.52+col*0.23 0.76-(row-1)*0.20 0.20 0.15];
    ax = axes('Parent',fig,'Position',position);
    hold(ax,'on'); grid(ax,'on');
    torqueAxes(j) = ax;
    torqueLines(j) = plot(ax,t(1),tau(1,j),'Color',[0.12 0.4 0.68], ...
        'LineWidth',1.4);
    [~,peakIndex] = max(abs(tau(:,j)));
    plot(ax,t(peakIndex),tau(peakIndex,j),'o','MarkerSize',5, ...
        'MarkerEdgeColor',[0.7 0.25 0.15]);
    currentDots(j) = plot(ax,t(1),tau(1,j),'o','MarkerSize',6, ...
        'MarkerFaceColor',[0.12 0.4 0.68], ...
        'MarkerEdgeColor',[0.12 0.4 0.68]);
    margin = max(0.1,0.10*max(abs(tau(:,j))));
    ylim(ax,[min(tau(:,j))-margin max(tau(:,j))+margin]);
    xlim(ax,[t(1) t(end)]);
    title(ax,names(j),'Interpreter','none','FontSize',10);
    ylabel(ax,'N m');
    if row == 3, xlabel(ax,'Time (s)'); end
end

focusAx = axes('Parent',fig,'Position',[0.53 0.14 0.43 0.16]);
hold(focusAx,'on'); grid(focusAx,'on');
components = {'total','gravity','inertial','velocity'};
colors = [0.12 0.35 0.65;0.18 0.58 0.28; ...
    0.82 0.35 0.12;0.47 0.38 0.58];
focusLines = gobjects(4,1);
for componentIndex = 1:4
    values = dynamics.torque.(components{componentIndex})(:,selectedJoint);
    focusLines(componentIndex) = plot(focusAx,t(1),values(1), ...
        'Color',colors(componentIndex,:),'LineWidth',1.5);
end
focusCursor = line(focusAx,[t(1) t(1)],[-1 1], ...
    'Color',[0.25 0.25 0.25],'LineStyle',':');
xlabel(focusAx,'Time (s)'); ylabel(focusAx,'Torque (N m)');
legend(focusAx,{'Total','Gravity','Inertial','Velocity'}, ...
    'Location','northoutside','Orientation','horizontal','FontSize',8);

infoText = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.04 0.16 0.43 0.10],'BackgroundColor','w', ...
    'HorizontalAlignment','left','FontSize',10);
slider = uicontrol(fig,'Style','slider','Units','normalized', ...
    'Position',[0.04 0.085 0.92 0.035],'Min',t(1),'Max',t(end), ...
    'Value',t(1),'Callback',@onScrub);
slider.SliderStep = [min(1,1/(numel(t)-1)) min(1,10/(numel(t)-1))];
playButton = uicontrol(fig,'Style','pushbutton','String','Play', ...
    'Units','normalized','Position',[0.04 0.015 0.09 0.05], ...
    'Callback',@onPlayPause);
uicontrol(fig,'Style','pushbutton','String','Restart', ...
    'Units','normalized','Position',[0.14 0.015 0.09 0.05], ...
    'Callback',@onRestart);
uicontrol(fig,'Style','text','String','Speed','Units','normalized', ...
    'Position',[0.25 0.020 0.05 0.035],'BackgroundColor','w');
speedMenu = uicontrol(fig,'Style','popupmenu', ...
    'String',{'0.25x','1x','2x'},'Value',rateIndex(rate), ...
    'Units','normalized','Position',[0.30 0.017 0.08 0.048], ...
    'Callback',@onSpeedChange);
uicontrol(fig,'Style','text','String','Joint','Units','normalized', ...
    'Position',[0.40 0.020 0.05 0.035],'BackgroundColor','w');
jointMenu = uicontrol(fig,'Style','popupmenu','String',cellstr(names), ...
    'Value',selectedJoint,'Units','normalized', ...
    'Position',[0.45 0.017 0.11 0.048],'Callback',@onJointChange);
uicontrol(fig,'Style','pushbutton','String','Jump to peak', ...
    'Units','normalized','Position',[0.58 0.015 0.12 0.05], ...
    'Callback',@onPeak);

playTimer = timer('ExecutionMode','fixedSpacing','Period',0.04, ...
    'BusyMode','drop','TimerFcn',@onTick,'ErrorFcn',@onTimerError);
renderSample(1);
viewer.figure = fig;
viewer.axes = robotAx;
viewer.seek = @seekSample;
viewer.play = @play;
viewer.pause = @pausePlayback;
viewer.restart = @restart;
viewer.jumpToPeak = @jumpToPeak;
viewer.selectJoint = @selectJoint;
viewer.getState = @getState;
viewer.close = @closeSession;
if options.autoplay, play(); end

    function renderSample(index)
        if ~isgraphics(fig), return; end
        currentIndex = index;
        show(robot.model,q(index,:),'Parent',robotAx,'FastUpdate',true, ...
            'Frames','off','PreservePlot',false);
        axis(robotAx,robotLimits);
        slider.Value = t(index);
        for joint = 1:dof
            torqueLines(joint).XData = t(1:index);
            torqueLines(joint).YData = tau(1:index,joint);
            currentDots(joint).XData = t(index);
            currentDots(joint).YData = tau(index,joint);
        end
        updateFocus();
        infoText.String = sprintf( ...
            't %.2f / %.2f s   |   %s: q %+.1f deg\nSpeed %+.3f rad/s   Torque %+.3f N m   Power %+.3f W', ...
            t(index),t(end),names(selectedJoint), ...
            rad2deg(q(index,selectedJoint)), ...
            trajectory.qd(index,selectedJoint),tau(index,selectedJoint), ...
            dynamics.power.joint(index,selectedJoint));
        title(robotAx,sprintf('%s%s | sample %d / %d', ...
            qualityLabel,robot.id,index,numel(t)),'Interpreter','none');
        drawnow limitrate;
    end

    function updateFocus()
        allValues = zeros(numel(t),4);
        for focusIndex = 1:4
            values = dynamics.torque.(components{focusIndex})(:,selectedJoint);
            allValues(:,focusIndex) = values;
            focusLines(focusIndex).XData = t(1:currentIndex);
            focusLines(focusIndex).YData = values(1:currentIndex);
        end
        margin = max(0.1,0.10*max(abs(allValues(:))));
        low = min(allValues(:))-margin;
        high = max(allValues(:))+margin;
        ylim(focusAx,[low high]);
        xlim(focusAx,[t(1) t(end)]);
        focusCursor.XData = [t(currentIndex) t(currentIndex)];
        focusCursor.YData = [low high];
        title(focusAx,names(selectedJoint),'Interpreter','none');
    end

    function seekSample(index)
        if ~isscalar(index) || ~isfinite(index) || index ~= floor(index) || ...
                index < 1 || index > numel(t)
            error('viewer:InvalidSample','Sample index must be in 1..N.');
        end
        pausePlayback();
        renderSample(index);
    end

    function play()
        if ~isgraphics(fig) || isPlaying, return; end
        if currentIndex == numel(t), renderSample(1); end
        playStartTime = t(currentIndex);
        playClock = tic;
        isPlaying = true;
        playButton.String = 'Pause';
        start(playTimer);
    end

    function pausePlayback()
        if ~isPlaying, return; end
        isPlaying = false;
        if strcmp(playTimer.Running,'on'), stop(playTimer); end
        if isgraphics(playButton), playButton.String = 'Play'; end
    end

    function restart()
        pausePlayback();
        renderSample(1);
    end

    function jumpToPeak()
        [~,peakIndex] = max(abs(tau(:,selectedJoint)));
        seekSample(peakIndex);
    end

    function selectJoint(joint)
        if ~isscalar(joint) || joint < 1 || joint > dof || joint ~= floor(joint)
            error('viewer:InvalidJoint','Joint must be an integer in 1..DOF.');
        end
        selectedJoint = joint;
        jointMenu.Value = joint;
        renderSample(currentIndex);
    end

    function state = getState()
        state.index = currentIndex;
        state.time = t(currentIndex);
        state.q = q(currentIndex,:);
        state.qd = trajectory.qd(currentIndex,:);
        state.qdd = trajectory.qdd(currentIndex,:);
        state.torque = tau(currentIndex,:);
        state.gravity = dynamics.torque.gravity(currentIndex,:);
        state.inertial = dynamics.torque.inertial(currentIndex,:);
        state.velocity = dynamics.torque.velocity(currentIndex,:);
        state.power = dynamics.power.joint(currentIndex,:);
        state.selectedJoint = selectedJoint;
        state.playing = isPlaying;
    end

    function onTick(~,~)
        if ~isPlaying || ~isgraphics(fig), return; end
        target = min(t(end),playStartTime+toc(playClock)*rate);
        index = find(t <= target,1,'last');
        if isempty(index), index = 1; end
        if index ~= currentIndex, renderSample(index); end
        if target >= t(end), pausePlayback(); end
    end

    function onTimerError(~,event)
        pausePlayback();
        warning('viewer:PlaybackError','Playback stopped: %s',event.Data.Message);
    end

    function onPlayPause(~,~)
        if isPlaying, pausePlayback(); else, play(); end
    end

    function onRestart(~,~)
        restart();
    end

    function onScrub(~,~)
        [~,index] = min(abs(t-slider.Value));
        seekSample(index);
    end

    function onSpeedChange(~,~)
        rates = [0.25 1 2];
        rate = rates(speedMenu.Value);
        if isPlaying
            playStartTime = t(currentIndex);
            playClock = tic;
        end
    end

    function onJointChange(~,~)
        selectJoint(jointMenu.Value);
    end

    function onPeak(~,~)
        jumpToPeak();
    end

    function closeSession()
        if isgraphics(fig), close(fig); end
    end

    function closeViewer(~,~)
        pausePlayback();
        if isvalid(playTimer), delete(playTimer); end
        delete(fig);
    end
end

function validateInputs(robot,trajectory,dynamics,options)
if ~isa(robot.model,'rigidBodyTree') || ...
        ~isstruct(trajectory) || ~isstruct(dynamics) || ...
        ~all(isfield(trajectory,{'time','q','qd','qdd'}))
    error('viewer:InvalidInput','Supply a loaded robot, trajectory, and dynamics result.');
end
t = trajectory.time;
q = trajectory.q;
[n,dof] = size(q);
if n < 2 || dof ~= robot.structure.dof || ~isequal(size(t),[n 1]) || ...
        any(~isfinite(t)) || any(diff(t) <= 0) || ...
        ~isequal(size(trajectory.qd),[n dof]) || ...
        ~isequal(size(trajectory.qdd),[n dof]) || ...
        any(~isfinite([q(:);trajectory.qd(:);trajectory.qdd(:)]))
    error('viewer:InvalidTrajectory','Trajectory has invalid states or time.');
end
limits = robot.params.joints.positionLimits;
if any(q < limits(:,1).'-1e-10 | q > limits(:,2).'+1e-10,'all')
    error('viewer:JointLimits','Trajectory exceeds joint limits.');
end
if ~isfield(dynamics,'meta') || ...
        string(dynamics.meta.robotID) ~= string(robot.id) || ...
        ~isequal(dynamics.time,t) || ...
        ~isequal(dynamics.state.q,q) || ...
        ~isequal(dynamics.state.qd,trajectory.qd) || ...
        ~isequal(dynamics.state.qdd,trajectory.qdd)
    error('viewer:ResultMismatch','Dynamics must match the robot and trajectory.');
end
for name = ["total","gravity","inertial","velocity"]
    if ~isequal(size(dynamics.torque.(name)),[n dof]) || ...
            any(~isfinite(dynamics.torque.(name)),'all')
        error('viewer:InvalidDynamics','Invalid torque component.');
    end
end
if ~isequal(size(dynamics.power.joint),[n dof]) || ...
        any(~isfinite(dynamics.power.joint),'all')
    error('viewer:InvalidDynamics','Invalid joint power.');
end
if ~isscalar(options.joint) || options.joint < 1 || ...
        options.joint > dof || options.joint ~= floor(options.joint) || ...
        ~isscalar(options.playbackRate) || ...
        ~ismember(options.playbackRate,[0.25 1 2]) || ...
        ~isscalar(options.autoplay) || ~isscalar(options.visible)
    error('viewer:InvalidOptions','Invalid joint, playbackRate, autoplay, or visible.');
end
end

function index = rateIndex(rate)
rates = [0.25 1 2];
index = find(rates == rate,1);
end
