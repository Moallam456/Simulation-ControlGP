function session = open_robot_testbench()
% OPEN_ROBOT_TESTBENCH Standalone no-argument entry point.
root=fileparts(mfilename('fullpath'));addpath(root,fullfile(root,'src'),fullfile(root,'control'));
addpath(fullfile(root,'mechanics','URDF_Assembly_Robot_ver2','meshes'));
cfg=testbench_config(); robot=testbench_robot(cfg);
if ~isfile(fullfile(root,'robot_ver2_testbench.slx')),build_robot_testbench();end
load_system(fullfile(root,'robot_ver2_testbench.slx'));
assert(strcmpi(get_param('robot_ver2_testbench','FileName'),fullfile(root,'robot_ver2_testbench.slx')), ...
    'testbench:OtherPackageLoaded','Close the testbench loaded from another folder first.');
w=get_param('robot_ver2_testbench','ModelWorkspace');
assert(strcmp(w.getVariable('packagedUrdfSHA256'),testbench_hash(fullfile(root,'mechanics','URDF_Assembly_Robot_ver2','urdf','URDF_Assembly_Robot_ver2.urdf'))), ...
    'testbench:StaleMechanics','URDF changed. Rebuild before using the saved physical model.');
assert(strcmp(w.getVariable('configurationSHA256'),testbench_hash(fullfile(root,'testbench_config.m'))), ...
    'testbench:StaleConfiguration','Configuration changed. Run build_robot_testbench before opening.');
f=figure('Name','robot_ver2 | Control Testbench','NumberTitle','off','Color','w','Position',[40 50 1340 820]);
ax=axes(f,'Position',[.04 .29 .54 .66]);
show(robot,cfg.home_rad,'Parent',ax,'Visuals','on','Frames','off');axis(ax,'equal');grid(ax,'on');view(ax,135,25);
hold(ax,'on');
for k=1:numel(cfg.environment)
    e=cfg.environment(k);b=collisionBox(e.size_m(1),e.size_m(2),e.size_m(3));b.Pose=trvec2tform(e.center_m);
    before=findall(ax,'Type','Patch');show(b,'Parent',ax);
    patches=setdiff(findall(ax,'Type','Patch'),before);
    set(patches,'FaceColor',[.5 .5 .5],'FaceAlpha',.15);
end
xlim(ax,[-.7 1.1]);ylim(ax,[-.7 1.1]);zlim(ax,[-.1 1.2]);
uicontrol(f,'Style','text','Units','normalized','Position',[.61 .91 .36 .05], ...
    'String','DEMONSTRATION | Hardware ratings: NOT EVALUATED','BackgroundColor','w','ForegroundColor',[.75 .1 .1]);
mode=uicontrol(f,'Style','popupmenu','Units','normalized','Position',[.62 .84 .34 .05], ...
    'String',{'C software-in-loop','Simulink laboratory','External C / CANopen','Joint-motion demonstration'});
button('Run', [.62 .76 .16 .06],@runBench);button('Open Simulink',[.80 .76 .16 .06],@(~,~) open_system(fullfile(root,'robot_ver2_testbench.slx')));
button('Go Home',[.62 .68 .16 .06],@goHome);button('Usable Workspace',[.80 .68 .16 .06],@(~,~) testbench_workspace(robot,cfg,ax));
table=uitable(f,'Units','normalized','Position',[.61 .32 .36 .30], ...
    'ColumnName',{'Joint','Measured (deg)','Command (deg)','Joint torque (Nm)'}, ...
    'ColumnWidth',{45 105 115 135},'Data',[(1:6).' cfg.home_deg.' cfg.home_deg.' gravityTorque(robot,cfg.home_rad).']);
status=uicontrol(f,'Style','text','Units','normalized','Position',[.04 .08 .92 .13], ...
    'BackgroundColor','w','HorizontalAlignment','left', ...
    'String','Compact home | Initial torques: static gravity-holding demand. Motor/sensor profiles are synthetic.');
session=struct('figure',f,'robot',robot,'config',cfg);

    function button(name,pos,callback)
        uicontrol(f,'Style','pushbutton','Units','normalized','Position',pos,'String',name,'Callback',callback);
    end
    function goHome(~,~)
        if mode.Value==3
            status.String='External Supervisor owns motion. Use its HMI Home command; compact home is supplied in the C configuration.';
            return;
        end
        setappdata(0,'RobotTestbenchHomeRequested',true);
        status.String='Home request queued. The full direct route must pass checks; no teleport.';
    end
    function runBench(~,~)
        choices={'c_sil','laboratory','external_can','demonstration'};cfg.controller=choices{mode.Value};
        setappdata(0,'RobotTestbenchConfig',cfg);setappdata(0,'RobotTestbenchHomeRequested',false);
        if strcmp(cfg.mode,'hardware_validation'),testbench_require_hardware(cfg);end
        load_system(fullfile(root,'robot_ver2_testbench.slx'));
        status.String='Simulating. Mechanics Explorer shows actual physical motion.';drawnow;
        if isappdata(0,'RobotTestbenchLive'),rmappdata(0,'RobotTestbenchLive');end
        refreshTimer=timer('ExecutionMode','fixedSpacing','Period',.25,'BusyMode','drop', ...
            'TimerFcn',@(~,~) refreshLive());
        start(refreshTimer);timerCleanup=onCleanup(@() closeTimer(refreshTimer));
        try
            out=sim('robot_ver2_testbench');
            result=getappdata(0,'RobotTestbenchResults');
            for k=1:numel(result.log)
                if ~isgraphics(f),break;end
                row=result.log(k);
                show(robot,row.q,'Parent',ax,'Frames','off','PreservePlot',false,'FastUpdate',true);
                table.Data=[(1:6).' rad2deg(row.q).' rad2deg(row.reference).' row.torque.'];drawnow limitrate;
            end
            if isempty(result.fault),fault='none';else,fault=result.fault;end
            status.String=sprintf('Fault: %s | Ratings NOT EVALUATED | %.1f s wall time',fault,result.elapsedWall_s);
            if ~isempty(result.homeMessage),status.String=[status.String ' | ' result.homeMessage];end
            plotResults(out,result);
            setappdata(f,'SimulationOutput',out);
        catch exception
            status.String=exception.message;rethrow(exception);
        end
    end
    function refreshLive()
        if ~isgraphics(f) || ~isappdata(0,'RobotTestbenchLive'),return;end
        row=getappdata(0,'RobotTestbenchLive');
        show(robot,row.q,'Parent',ax,'Frames','off','PreservePlot',false,'FastUpdate',true);
        table.Data=[(1:6).' rad2deg(row.q).' rad2deg(row.reference).' row.torque.'];
        status.String=sprintf(['t %.2f s | Linear inverse condition %.3f | Self-clearance lower bound %.1f mm\n' ...
            'Fault: %s | Hardware ratings NOT EVALUATED'],row.t,row.safety.linearIndex, ...
            row.safety.selfClearance_m*1000,row.fault);
    end
end

function closeTimer(t)
if isvalid(t),stop(t);delete(t);end
end

function plotResults(out,result)
f=figure('Name','robot_ver2 | Measured response','Color','w');tiledlayout(f,3,1);
nexttile;plot(out.tb_joint_position.Time,rad2deg(squeeze(out.tb_joint_position.Data)));ylabel('Joint angle (deg)');grid on;
nexttile;plot(out.tb_joint_torque.Time,squeeze(out.tb_joint_torque.Data));ylabel('Joint torque (Nm)');grid on;
nexttile;
if ~isempty(result.log)
    t=[result.log.t];q=vertcat(result.log.q);r=vertcat(result.log.reference);
    plot(t,rad2deg(r-q));ylabel('Tracking error (deg)');grid on;
end
xlabel('Simulation time (s)');
end
