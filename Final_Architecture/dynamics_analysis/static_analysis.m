function session = static_analysis()
% STATIC_ANALYSIS Inspect gravity holding torques at a slider-selected pose.
% Run with no arguments; the robot is selected by projectConfig.m.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'visualization'));
robot = loadRobot();
gravity = [0 0 -9.81];
[model,validation] = prepareDynamicsModel(robot,gravity);
dof = robot.structure.dof;
limits = robot.params.joints.positionLimits;
q = reshape(robot.params.joints.homePosition,1,[]);
jointNames = string(robot.structure.jointNames);
torque = nan(1,dof);
calculated = false;

fig = figure('Name',sprintf('%s | Static analysis',robot.id), ...
    'NumberTitle','off','Color','w','Position',[80 80 1400 800]);
uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.04 0.94 0.90 0.04], ...
    'String',sprintf('%s  |  Static holding torque',robot.id), ...
    'FontSize',15,'FontWeight','bold','HorizontalAlignment','left', ...
    'BackgroundColor','w');

robotAx = axes('Parent',fig,'Units','normalized', ...
    'Position',[0.04 0.10 0.52 0.80]);
show(robot.model,q,'Parent',robotAx,'Frames','off','PreservePlot',false);
grid(robotAx,'on');
axis(robotAx,'equal');
view(robotAx,135,25);
xlabel(robotAx,'X (m)');
ylabel(robotAx,'Y (m)');
zlabel(robotAx,'Z (m)');
poses = repmat(q,2*dof+1,1);
for j = 1:dof
    poses(2*j,j) = limits(j,1);
    poses(2*j+1,j) = limits(j,2);
end
poseLimits = robotPoseBounds(robot,poses);
reach = max(abs(poseLimits(1:4))) + 0.10;
zLow = min(-0.15,poseLimits(5)-0.10);
zHigh = poseLimits(6)+0.10;
axis(robotAx,[-reach reach -reach reach zLow zHigh]);
rotate3d(fig,'on');

controls = uipanel(fig,'Title','Joint angles (deg)', ...
    'Units','normalized','Position',[0.60 0.53 0.36 0.38], ...
    'BackgroundColor','w','FontWeight','bold');
sliders = gobjects(dof,1);
angleLabels = gobjects(dof,1);
rowHeight = 0.88/dof;
for j = 1:dof
    rowY = 0.93-j*rowHeight;
    uicontrol(controls,'Style','text','Units','normalized', ...
        'Position',[0.025 rowY 0.23 rowHeight*0.75], ...
        'String',char(jointNames(j)), ...
        'HorizontalAlignment','left','BackgroundColor','w');
    range = diff(limits(j,:));
    sliders(j) = uicontrol(controls,'Style','slider','Units','normalized', ...
        'Position',[0.25 rowY+rowHeight*0.12 0.55 rowHeight*0.65], ...
        'Min',limits(j,1),'Max',limits(j,2),'Value',q(j), ...
        'SliderStep',min([deg2rad([1 10])/range; 1 1],[],1), ...
        'Callback',@(~,~) sliderChanged(j));
    angleLabels(j) = uicontrol(controls,'Style','text','Units','normalized', ...
        'Position',[0.81 rowY 0.17 rowHeight*0.75], ...
        'String',sprintf('%+.1f',rad2deg(q(j))), ...
        'HorizontalAlignment','right','BackgroundColor','w');
end

uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.60 0.46 0.22 0.055],'String','Calculate', ...
    'FontWeight','bold','Callback',@(~,~) calculateTorques());
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.83 0.46 0.13 0.055],'String','Home', ...
    'Callback',@(~,~) setConfiguration(robot.params.joints.homePosition));

resultLabel = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.60 0.415 0.36 0.035], ...
    'String','Choose a pose, then press Calculate.', ...
    'HorizontalAlignment','left','BackgroundColor','w');
torqueAx = axes('Parent',fig,'Units','normalized', ...
    'Position',[0.65 0.255 0.29 0.15]);
drawTorqueChart();
torqueTable = uitable(fig,'Units','normalized', ...
    'Position',[0.60 0.05 0.36 0.18], ...
    'ColumnName',{'Joint','Holding torque (N m)'}, ...
    'ColumnEditable',[false false], ...
    'ColumnWidth',{190 200}, ...
    'Data',[cellstr(jointNames(:)) repmat({''},dof,1)]);

if validation.preliminary
    quality = 'PRELIMINARY mass properties';
else
    quality = 'Validated mass properties';
end
if isfield(robot.readiness,'jointLimitsProvisional') && ...
        robot.readiness.jointLimitsProvisional
    quality = [quality '  |  provisional joint limits'];
end
uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.04 0.012 0.92 0.028], ...
    'String',quality,'HorizontalAlignment','left', ...
    'BackgroundColor','w','ForegroundColor',[0.65 0.27 0.05]);

session.figure = fig;
session.robot = robot;
session.setConfiguration = @setConfiguration;
session.calculate = @calculateTorques;
session.getState = @getState;
session.close = @() close(fig);

    function sliderChanged(jointIndex)
        q(jointIndex) = get(sliders(jointIndex),'Value');
        updatePose();
    end

    function setConfiguration(configuration)
        configuration = reshape(configuration,1,[]);
        if numel(configuration) ~= dof || any(~isfinite(configuration)) || ...
                any(configuration < limits(:,1).' | configuration > limits(:,2).')
            error('static_analysis:InvalidConfiguration', ...
                'Configuration must be a finite, limit-valid %d-joint row in radians.',dof);
        end
        q = configuration;
        for k = 1:dof
            set(sliders(k),'Value',q(k));
        end
        updatePose();
    end

    function updatePose()
        for k = 1:dof
            set(angleLabels(k),'String',sprintf('%+.1f',rad2deg(q(k))));
        end
        show(robot.model,q,'Parent',robotAx,'FastUpdate',true, ...
            'Frames','off','PreservePlot',false);
        calculated = false;
        torque(:) = nan;
        set(resultLabel,'String','Pose changed. Press Calculate.');
        set(torqueTable,'Data',[cellstr(jointNames(:)) repmat({''},dof,1)]);
        drawTorqueChart();
        drawnow limitrate;
    end

    function calculateTorques()
        torque = reshape(gravityTorque(model,q),1,[]);
        calculated = true;
        rows = cell(dof,2);
        for k = 1:dof
            rows{k,1} = char(jointNames(k));
            rows{k,2} = sprintf('%+.3f',torque(k));
        end
        set(torqueTable,'Data',rows);
        set(resultLabel,'String','Gravity holding torque at the displayed pose');
        drawTorqueChart();
        fprintf('%s static holding torque (N m), joint order: %s\n', ...
            robot.id,strjoin(jointNames,' '));
        disp(torque);
    end

    function drawTorqueChart()
        cla(torqueAx);
        if calculated
            barh(torqueAx,torque,'FaceColor',[0.18 0.43 0.70]);
            span = max(0.1,max(abs(torque))*1.2);
        else
            barh(torqueAx,zeros(1,dof),'FaceColor',[0.75 0.78 0.81]);
            span = 1;
        end
        xlim(torqueAx,[-span span]);
        ylim(torqueAx,[0.5 dof+0.5]);
        set(torqueAx,'YTick',1:dof,'YTickLabel',cellstr(jointNames), ...
            'YDir','reverse','TickLabelInterpreter','none');
        grid(torqueAx,'on');
        xlabel(torqueAx,'Torque (N m)');
    end

    function state = getState()
        state.q = q;
        state.torque_Nm = torque;
        state.calculated = calculated;
        state.gravity_m_s2 = gravity;
    end
end
