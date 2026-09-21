function viewer = viewGravityLoading(robot,poses,options)
% VIEWGRAVITYLOADING Inspect independent stationary poses and holding torques.
% poses is an analyzeGravityLoading result, optionally with renamed poseNames.
if nargin < 3, options = struct(); end
if ~isfield(options,'joint'), options.joint = min(2,robot.structure.dof); end
if ~isfield(options,'visible'), options.visible = true; end
if ~isfield(options,'peakPoseCount'), options.peakPoseCount = 0; end
q = poses.q;
tau = poses.torque;
[n,dof] = size(q);
if ~isfield(options,'initialPose')
    options.initialPose = min(options.joint,n);
end
if n < 1 || dof ~= robot.structure.dof || ...
        ~isequal(size(tau),[n dof]) || numel(poses.poseNames) ~= n || ...
        any(~isfinite([q(:);tau(:)])) || ...
        options.joint < 1 || options.joint > dof || ...
        options.peakPoseCount < 0 || options.peakPoseCount > min(n,dof) || ...
        options.initialPose < 1 || options.initialPose > n || ...
        options.initialPose ~= floor(options.initialPose)
    error('viewer:InvalidGravityPoses','Expected matching finite poses and torques.');
end
names = string(poses.poseNames(:));
index = options.initialPose;
qualityLabel = "";
if poses.meta.preliminary, qualityLabel = "PRELIMINARY "; end
fig = figure('Name',char(qualityLabel + string(robot.id) + " stationary gravity"), ...
    'NumberTitle','off','Color','w','Position',[120 100 1250 710], ...
    'Visible',matlab.lang.OnOffSwitchState(options.visible));
robotAx = axes('Parent',fig,'Position',[0.04 0.20 0.50 0.72]);
show(robot.model,q(index,:),'Parent',robotAx,'Frames','off', ...
    'PreservePlot',false);
grid(robotAx,'on'); axis(robotAx,'equal'); view(robotAx,135,25);
xlabel(robotAx,'X (m)'); ylabel(robotAx,'Y (m)'); zlabel(robotAx,'Z (m)');
robotLimits = robotPoseBounds(robot,q);
axis(robotAx,robotLimits);
torqueAx = axes('Parent',fig,'Position',[0.62 0.34 0.33 0.52]);
bars = bar(torqueAx,1:dof,tau(index,:),'FaceColor','flat');
grid(torqueAx,'on'); ylabel(torqueAx,'Holding torque (N m)');
set(torqueAx,'XTick',1:dof, ...
    'XTickLabel',cellstr(string(robot.structure.jointNames(:))), ...
    'TickLabelInterpreter','none');
maxTorque = max(abs(tau(:)));
ylim(torqueAx,1.15*[-max(0.1,maxTorque) max(0.1,maxTorque)]);
info = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.60 0.19 0.36 0.09],'BackgroundColor','w', ...
    'HorizontalAlignment','left','FontSize',10);
uicontrol(fig,'Style','text','String','Stationary pose', ...
    'Units','normalized','Position',[0.05 0.075 0.13 0.05], ...
    'BackgroundColor','w');
poseMenu = uicontrol(fig,'Style','popupmenu','String',cellstr(names), ...
    'Value',index,'Units','normalized', ...
    'Position',[0.18 0.072 0.60 0.06], ...
    'Callback',@onPoseChange);
viewer.figure = fig;
viewer.axes = robotAx;
viewer.selectPose = @selectPose;
viewer.selectJoint = @selectJoint;
viewer.getState = @getState;
viewer.close = @closeSession;
renderPose(index);

    function renderPose(poseIndex)
        index = poseIndex;
        if index <= options.peakPoseCount, options.joint = index; end
        show(robot.model,q(index,:),'Parent',robotAx,'FastUpdate',true, ...
            'Frames','off','PreservePlot',false);
        axis(robotAx,robotLimits);
        poseMenu.Value = index;
        bars.YData = tau(index,:);
        colors = repmat([0.15 0.42 0.65],dof,1);
        selected = min(options.joint,dof);
        colors(selected,:) = [0.80 0.27 0.16];
        bars.CData = colors;
        info.String = sprintf('%s\nq (deg): %s\n%s holding torque: %+.3f N m', ...
            names(index),mat2str(round(rad2deg(q(index,:)),1)), ...
            string(robot.structure.jointNames(selected)),tau(index,selected));
        title(robotAx,qualityLabel + names(index),'Interpreter','none');
        drawnow limitrate;
    end

    function selectPose(poseIndex)
        if ~isscalar(poseIndex) || ~isfinite(poseIndex) || ...
                poseIndex ~= floor(poseIndex) || poseIndex < 1 || poseIndex > n
            error('viewer:InvalidPose','Pose index must be in 1..N.');
        end
        renderPose(poseIndex);
    end

    function selectJoint(joint)
        if ~isscalar(joint) || ~isfinite(joint) || ...
                joint ~= floor(joint) || joint < 1 || joint > dof
            error('viewer:InvalidJoint','Joint must be in 1..DOF.');
        end
        options.joint = joint;
        if joint <= options.peakPoseCount
            renderPose(joint);
        else
            renderPose(index);
        end
    end

    function state = getState()
        state.index = index;
        state.name = names(index);
        state.q = q(index,:);
        state.torque = tau(index,:);
        state.selectedJoint = options.joint;
        state.gravity = poses.meta.gravity;
    end

    function onPoseChange(~,~)
        selectPose(poseMenu.Value);
    end

    function closeSession()
        if isgraphics(fig), close(fig); end
    end
end
