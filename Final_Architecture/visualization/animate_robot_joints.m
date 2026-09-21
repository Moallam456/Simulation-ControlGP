function result = animate_robot_joints(robot,options)
% ANIMATE_ROBOT_JOINTS Sweep selected joints one at a time for visual review.
% Angles supplied in options are degrees; robot configurations remain radians.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'kinematics'));
if nargin < 1 || isempty(robot), robot = loadRobot(); end
if nargin < 2, options = struct(); end
if ~isfield(options,'qReference'), options.qReference = robot.params.joints.homePosition; end
if ~isfield(options,'jointIndices'), options.jointIndices = 1:robot.structure.dof; end
if ~isfield(options,'amplitudeDeg'), options.amplitudeDeg = 45; end
if ~isfield(options,'framesPerSweep'), options.framesPerSweep = 20; end
if ~isfield(options,'pauseSeconds'), options.pauseSeconds = 0.025; end
if ~isfield(options,'visible'), options.visible = true; end

dof = robot.structure.dof;
q0 = reshape(options.qReference,1,[]);
limits = robot.params.joints.positionLimits;
indices = options.jointIndices(:).';
amplitude = options.amplitudeDeg;
if isscalar(amplitude), amplitude = repmat(amplitude,1,dof); end
if numel(q0) ~= dof || any(~isfinite(q0)) || ...
        any(q0 < limits(:,1).' | q0 > limits(:,2).')
    error('animation:InvalidReference','qReference must be within joint limits in radians.');
end
if isempty(indices) || any(indices < 1 | indices > dof | indices ~= floor(indices)) || ...
        numel(unique(indices)) ~= numel(indices)
    error('animation:InvalidJoints','jointIndices must contain unique joint indices.');
end
if numel(amplitude) ~= dof || any(~isfinite(amplitude) | amplitude < 0) || ...
        ~isscalar(options.framesPerSweep) || ...
        options.framesPerSweep < 2 || options.framesPerSweep ~= floor(options.framesPerSweep) || ...
        ~isscalar(options.pauseSeconds) || options.pauseSeconds < 0 || ...
        ~isscalar(options.visible)
    error('animation:InvalidOptions','Check amplitudeDeg, framesPerSweep, pauseSeconds, and visible.');
end

fig = figure('Name',char(string(robot.id) + " joint motion check"), ...
    'Color','w','Visible',matlab.lang.OnOffSwitchState(options.visible));
ax = axes('Parent',fig);
show(robot.model,q0,'Parent',ax,'Frames','off','PreservePlot',false);
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
reach = norm(robot.params.tool.TFlangeTCP(1:3,4));
for i = 2:dof
    reach = reach + norm(robot.chain.fixedTransforms{i}(1:3,4));
end
reach = reach + 0.15;
baseHeight = norm(robot.chain.fixedTransforms{1}(1:3,4));
axis(ax,[-reach reach -reach reach -0.2 baseHeight+reach]);
view(ax,135,25);
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
[~,frames] = forward_kinematics(robot,q0);
centers = jointCenters(frames,dof);
markers = scatter3(ax,centers(:,1),centers(:,2),centers(:,3), ...
    65,repmat([0.1 0.4 0.7],dof,1),'filled','MarkerEdgeColor','k');
activeLabel = text(ax,centers(1,1),centers(1,2),centers(1,3), ...
    ' J1','FontWeight','bold','Color',[0.75 0.1 0.1]);
axisScale = min(0.10,0.15*reach);
triadColors = [0.85 0.15 0.1;0.1 0.6 0.2;0.15 0.3 0.85];
triad = gobjects(3,1);
for k = 1:3
    triad(k) = quiver3(ax,centers(1,1),centers(1,2),centers(1,3), ...
        axisScale*frames{2}(1,k),axisScale*frames{2}(2,k), ...
        axisScale*frames{2}(3,k),0,'Color',triadColors(k,:), ...
        'LineWidth',2,'MaxHeadSize',0.7);
end
result.figure = fig;
result.axes = ax;
result.completedJoints = [];
result.qFinal = q0;
for j = indices
    positive = min(deg2rad(amplitude(j)),limits(j,2)-q0(j));
    negative = max(-deg2rad(amplitude(j)),limits(j,1)-q0(j));
    n = options.framesPerSweep;
    offsets = [linspace(0,positive,n),linspace(positive,0,n), ...
        linspace(0,negative,n),linspace(negative,0,n)];
    for offset = offsets
        if ~isgraphics(fig), return; end
        q = q0;
        q(j) = q0(j) + offset;
        show(robot.model,q,'Parent',ax,'FastUpdate',true, ...
            'Frames','off','PreservePlot',false);
        [~,frames] = forward_kinematics(robot,q);
        centers = jointCenters(frames,dof);
        markers.XData = centers(:,1);
        markers.YData = centers(:,2);
        markers.ZData = centers(:,3);
        colors = repmat([0.1 0.4 0.7],dof,1);
        colors(j,:) = [0.85 0.15 0.1];
        markers.CData = colors;
        activeLabel.Position = centers(j,:);
        activeLabel.String = sprintf(' J%d',j);
        for k = 1:3
            triad(k).XData = centers(j,1);
            triad(k).YData = centers(j,2);
            triad(k).ZData = centers(j,3);
            triad(k).UData = axisScale*frames{j+1}(1,k);
            triad(k).VData = axisScale*frames{j+1}(2,k);
            triad(k).WData = axisScale*frames{j+1}(3,k);
        end
        title(ax,sprintf('%s | J%d = %+.1f deg', ...
            robot.id,j,rad2deg(q(j))),'Interpreter','none');
        result.qFinal = q;
        drawnow;
        pause(options.pauseSeconds);
    end
    result.completedJoints(end+1) = j; %#ok<AGROW>
end
title(ax,sprintf('%s | joint sweep complete',robot.id),'Interpreter','none');
end

function centers = jointCenters(frames,dof)
centers = zeros(dof,3);
for j = 1:dof
    centers(j,:) = frames{j+1}(1:3,4).';
end
end
