function frames = plot_robot(robot,q)

% PLOT_ROBOT Generic visualization for a loaded robot.
%
% Preferred:
%   robot = loadRobot();
%   plot_robot(robot, q)
%
% Convenience:
%   plot_robot()
%   plot_robot(q)

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'kinematics'));

if nargin < 1 || isempty(robot)
    robot = loadRobot();
    q = robot.params.joints.homePosition;
elseif isnumeric(robot)
    q = robot;
    robot = loadRobot();
elseif nargin < 2 || isempty(q)
    q = robot.params.joints.homePosition;
end

q = q(:).';

if numel(q) ~= robot.structure.dof
    error('plot_robot:InvalidJointVector', ...
        'Expected %d joint values for robot %s; received %d.', ...
        robot.structure.dof, robot.id, numel(q));
end

[~, frames] = forward_kinematics(robot,q);

figure('Name',char(robot.params.displayName),'Color','w');

try
    show( ...
        robot.model, ...
        q, ...
        'Frames','off', ...
        'PreservePlot',false);
catch
    show(robot.model,q);
end

hold on;
grid on;
axis equal;

drawCenterOfMassMarkers(robot,q);

frameScale = max(0.08,0.07*nominalReach(robot));

for i = 1:numel(frames)
    if i == 1
        labelText = 'B';
    elseif i == numel(frames)
        labelText = 'TCP';
    else
        labelText = sprintf('J%d',i-1);
    end

    drawFrameLabel(frames{i},frameScale,labelText);
end

title(sprintf('%s - %s',robot.params.displayName,robot.id),'Interpreter','none');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
view(135,30);

allPoints = collectPlotPoints(robot,frames);
margin = max(0.25,0.1*nominalReach(robot));

xlim([min(allPoints(:,1))-margin max(allPoints(:,1))+margin]);
ylim([min(allPoints(:,2))-margin max(allPoints(:,2))+margin]);
zlim([min(allPoints(:,3))-margin max(allPoints(:,3))+margin]);

hold off;

end

function reach = nominalReach(robot)

g = robot.params.geometry;
reach = abs(g.l3) + abs(g.l4) + abs(g.l7) + ...
    abs(g.l8) + abs(g.l9) + abs(g.l10);

end

function points = collectPlotPoints(robot,frames)

points = cellfun(@(T) T(1:3,4).',frames,'UniformOutput',false);
points = vertcat(points{:});

if isfield(robot.chain,'visualSegments')
    for i = 1:robot.structure.dof
        points = [
            points
            transformPoints(frames{i},robot.chain.visualSegments{i})]; %#ok<AGROW>
    end
end

if isfield(robot.chain,'toolSegment')
    points = [
        points
        transformPoints(frames{end-1},robot.chain.toolSegment)]; %#ok<AGROW>
end

end

function drawFrameLabel(T,scale,labelText)

origin = T(1:3,4);
xAxis = T(1:3,1);
yAxis = T(1:3,2);
zAxis = T(1:3,3);

quiver3(origin(1),origin(2),origin(3), ...
    scale*xAxis(1),scale*xAxis(2),scale*xAxis(3), ...
    'r','LineWidth',1.5,'MaxHeadSize',0.6);

quiver3(origin(1),origin(2),origin(3), ...
    scale*yAxis(1),scale*yAxis(2),scale*yAxis(3), ...
    'g','LineWidth',1.5,'MaxHeadSize',0.6);

quiver3(origin(1),origin(2),origin(3), ...
    scale*zAxis(1),scale*zAxis(2),scale*zAxis(3), ...
    'b','LineWidth',1.5,'MaxHeadSize',0.6);

text( ...
    origin(1), ...
    origin(2), ...
    origin(3), ...
    [' ' labelText], ...
    'FontWeight','bold', ...
    'FontSize',9, ...
    'Interpreter','none');

end

function drawCenterOfMassMarkers(robot,q)

for i = 1:numel(robot.model.Bodies)
    body = robot.model.Bodies{i};
    comLocal = body.CenterOfMass;

    if body.Mass == 0 || numel(comLocal) ~= 3 || any(~isfinite(comLocal))
        continue;
    end

    bodyFrame = getTransform(robot.model,q,body.Name);
    comWorld = transformPoints(bodyFrame,comLocal);

    scatter3( ...
        comWorld(1), ...
        comWorld(2), ...
        comWorld(3), ...
        70, ...
        'd', ...
        'filled', ...
        'MarkerFaceColor',[0.85 0 0.85], ...
        'MarkerEdgeColor','k');

    text( ...
        comWorld(1), ...
        comWorld(2), ...
        comWorld(3), ...
        sprintf(' %s COM %.1fkg',body.Name,body.Mass), ...
        'FontWeight','bold', ...
        'FontSize',8, ...
        'Color',[0.45 0 0.45], ...
        'Interpreter','none');
end

end

function pointsWorld = transformPoints(T,pointsLocal)

pointsHomogeneous = [pointsLocal ones(size(pointsLocal,1),1)];
pointsHomogeneous = (T * pointsHomogeneous.').';
pointsWorld = pointsHomogeneous(:,1:3);

end
