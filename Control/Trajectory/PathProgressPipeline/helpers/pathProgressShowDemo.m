function fig = pathProgressShowDemo(data,robot)
%PATHPROGRESSSHOWDEMO Render the URDF robot, station, and reference plots.
helpers = fileparts(mfilename('fullpath'));
repo = fileparts(fileparts(fileparts(fileparts(helpers))));
oldPath = path;
restore = onCleanup(@() path(oldPath));
addpath(repo);
station = environment.buildPreliminaryWeldingStation();
fig = figure('Name',"Path progress " + data.demo);
ax = axes(fig);
drawRobot(robot.model,data.qPath(:,1),ax);
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
show(station.mountingPlane.object,'Parent',ax);
show(station.table.object,'Parent',ax);
plot3(ax,data.rawGeometryPath(1,:),data.rawGeometryPath(2,:),data.rawGeometryPath(3,:),'--');
plot3(ax,data.arcGeometryPath(1,:),data.arcGeometryPath(2,:),data.arcGeometryPath(3,:),'.');
plot3(ax,data.pPath(1,:),data.pPath(2,:),data.pPath(3,:),'LineWidth',2);
plot3(ax,data.pAchieved(1,:),data.pAchieved(2,:),data.pAchieved(3,:),'--');
xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]');
view(ax,3);
uicontrol(fig,'Style','pushbutton','String','Replay Motion','Units','normalized', ...
    'Position',[.81 .02 .16 .07],'Callback',@(~,~) play(data,robot.model,ax));
plots = figure('Name',"Path-progress references " + data.demo);
layout = tiledlayout(plots,2,3);
trace(layout,data.tGlobal,data.qPath,'Joint position [rad]');
trace(layout,data.tGlobal,data.qDot,'Joint velocity [rad/s]');
trace(layout,data.tGlobal,data.qDDot,'Joint acceleration [rad/s^2]');
trace(layout,data.tGlobal,data.tcpSpeedPath,'Planned TCP speed [m/s]');
trace(layout,data.tGlobal,data.positionError,'TCP position error [m]');
trace(layout,data.tGlobal,data.orientationError,'TCP rotation error [rad]');
play(data,robot.model,ax);
end

function trace(layout,time,values,label)
ax = nexttile(layout);
plot(ax,time,values.');
grid(ax,'on');
xlabel(ax,'Time [s]');
ylabel(ax,label);
end

function play(data,model,ax)
for i = 1:numel(data.tGlobal)
    if ~isgraphics(ax), return; end
    drawRobot(model,data.qPath(:,i),ax);
    title(ax,sprintf('robot_ver2 | %.2f / %.2f s',data.tGlobal(i),data.tGlobal(end)), ...
        'Interpreter','none');
    drawnow limitrate
    if i < numel(data.tGlobal)
        pause(data.tGlobal(i+1)-data.tGlobal(i));
    end
end
drawnow
setappdata(ancestor(ax,'figure'),'PathProgressLastFrame',numel(data.tGlobal));
end

function drawRobot(model,q,ax)
show(model,q.','Parent',ax,'FastUpdate',true,'PreservePlot',false, ...
    'Visuals','on','Collisions','off');
end
