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
        'Frames','on', ...
        'PreservePlot',false);
catch
    show(robot.model,q);
end

hold on;
grid on;
axis equal;

drawVisualSegments(robot,frames);
drawCenterOfMassMarkers(robot,frames);

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

if isfield(robot.dh,'visualSegments')
    for i = 1:robot.structure.dof
        points = [
            points
            transformPoints(frames{i},robot.dh.visualSegments{i})]; %#ok<AGROW>
    end
end

if isfield(robot.dh,'toolSegment')
    points = [
        points
        transformPoints(frames{end-1},robot.dh.toolSegment)]; %#ok<AGROW>
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

function drawVisualSegments(robot,frames)

if isfield(robot.dh,'visualSegments')
    for i = 1:robot.structure.dof
        pointsWorld = transformPoints(frames{i},robot.dh.visualSegments{i});
        linkParams = robot.params.links(i);

        if isfield(linkParams,'radius') && isfinite(linkParams.radius)
            radius = linkParams.radius;
        else
            radius = 0.04;
        end

        if isfield(linkParams,'jointRadius') && isfinite(linkParams.jointRadius)
            jointRadius = linkParams.jointRadius;
        else
            jointRadius = 1.4*radius;
        end

        if isfield(linkParams,'color') && numel(linkParams.color) == 3
            color = linkParams.color;
        else
            color = [0.8 0.8 0.8];
        end

        plot3( ...
            pointsWorld(:,1), ...
            pointsWorld(:,2), ...
            pointsWorld(:,3), ...
            'k-', ...
            'LineWidth',3);

        scatter3( ...
            pointsWorld(end,1), ...
            pointsWorld(end,2), ...
            pointsWorld(end,3), ...
            45, ...
            'filled', ...
            'MarkerFaceColor',[0 0.4470 0.7410], ...
            'MarkerEdgeColor','k');

        for j = 1:size(pointsWorld,1)-1
            drawCylinder(pointsWorld(j,:),pointsWorld(j+1,:),radius,color);
        end

        drawSphere(pointsWorld(end,:),jointRadius,color);
    end
end

if isfield(robot.dh,'toolSegment')
    pointsWorld = transformPoints(frames{end-1},robot.dh.toolSegment);
    toolRadius = 0.025;

    plot3( ...
        pointsWorld(:,1), ...
        pointsWorld(:,2), ...
        pointsWorld(:,3), ...
        'r-', ...
        'LineWidth',3);

    scatter3( ...
        pointsWorld(end,1), ...
        pointsWorld(end,2), ...
        pointsWorld(end,3), ...
        80, ...
        'filled', ...
        'MarkerFaceColor','r', ...
        'MarkerEdgeColor','k');

    for j = 1:size(pointsWorld,1)-1
        drawCylinder(pointsWorld(j,:),pointsWorld(j+1,:),toolRadius,[0.05 0.05 0.05]);
    end

    drawSphere(pointsWorld(end,:),1.4*toolRadius,[0.8 0.1 0.1]);
end

end

function drawCenterOfMassMarkers(robot,frames)

for i = 1:numel(robot.model.Bodies)
    body = robot.model.Bodies{i};
    comLocal = body.CenterOfMass;

    if numel(comLocal) ~= 3 || any(~isfinite(comLocal))
        continue;
    end

    if i <= robot.structure.dof
        bodyFrame = frames{i+1};
    else
        bodyFrame = frames{end};
    end

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

function drawCylinder(p1,p2,radius,color)

p1 = p1(:);
p2 = p2(:);
axisVector = p2 - p1;
length = norm(axisVector);

if length < 1e-9
    return;
end

[x,y,z] = cylinder(radius,24);
z = z*length;

points = [x(:) y(:) z(:)];
T = segmentTransform(p1,p2);
pointsWorld = transformPoints(T,points);

xWorld = reshape(pointsWorld(:,1),size(x));
yWorld = reshape(pointsWorld(:,2),size(y));
zWorld = reshape(pointsWorld(:,3),size(z));

surf( ...
    xWorld, ...
    yWorld, ...
    zWorld, ...
    'FaceColor',color, ...
    'EdgeColor','none', ...
    'FaceAlpha',0.75);

end

function drawSphere(center,radius,color)

[x,y,z] = sphere(18);

surf( ...
    radius*x + center(1), ...
    radius*y + center(2), ...
    radius*z + center(3), ...
    'FaceColor',color, ...
    'EdgeColor','none', ...
    'FaceAlpha',0.85);

end

function T = segmentTransform(p1,p2)

axisVector = p2 - p1;
length = norm(axisVector);

T = eye(4);
T(1:3,1:3) = frameWithZ(axisVector/length);
T(1:3,4) = p1;

end

function R = frameWithZ(zAxis)

zAxis = zAxis/norm(zAxis);

if abs(dot(zAxis,[0;0;1])) < 0.9
    xGuess = [0;0;1];
else
    xGuess = [0;1;0];
end

yAxis = cross(zAxis,xGuess);
yAxis = yAxis/norm(yAxis);
xAxis = cross(yAxis,zAxis);
xAxis = xAxis/norm(xAxis);

R = [xAxis yAxis zAxis];

end

function pointsWorld = transformPoints(T,pointsLocal)

pointsHomogeneous = [pointsLocal ones(size(pointsLocal,1),1)];
pointsHomogeneous = (T * pointsHomogeneous.').';
pointsWorld = pointsHomogeneous(:,1:3);

end
