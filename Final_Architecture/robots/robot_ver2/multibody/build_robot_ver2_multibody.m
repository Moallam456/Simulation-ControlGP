function modelFile=build_robot_ver2_multibody()
% BUILD_ROBOT_VER2_MULTIBODY Build the torque-driven plant and a removable demo.
% Rebuilding backs up an existing saved SLX; unsaved models are never replaced.
demoDuration_s=8;
demoAmplitude_deg=[12 8 -8 15 12 20];
demoKp=[160 500 250 25 20 8]; % Demonstration only, N m/rad; not motor ratings.
gravity=[0 0 -9.81];

folder=fileparts(mfilename('fullpath'));
root=fileparts(fileparts(fileparts(folder))); addpath(genpath(root));
assert(exist('smimport','file')==2 && license('test','SimMechanics'), ...
    'multibody:ToolboxRequired','Install/license Simulink, Simscape and Simscape Multibody.');
robot=loadRobot('robot_ver2'); n=robot.structure.dof;
model='robot_ver2_multibody'; modelFile=fullfile(folder,[model '.slx']);
if bdIsLoaded(model)
    assert(strcmp(get_param(model,'Dirty'),'off'),'multibody:UnsavedChanges', ...
        'Save and close robot_ver2_multibody before rebuilding.');
    close_system(model,0);
end
if isfile(modelFile)
    backupFolder=fullfile(folder,'backups');
    if ~isfolder(backupFolder), mkdir(backupFolder); end
    backup=fullfile(backupFolder,['robot_ver2_multibody_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '.slx']);
    copyfile(modelFile,backup);
    fprintf('Existing model backed up: %s\n',backup);
end
previousFolder=pwd; restore=onCleanup(@() cd(previousFolder));
cd(folder);
fprintf('Importing robot_ver2 URDF into Simscape Multibody...\n');
smimport(char(robot.params.source.urdf),'ModelName',model);
load_system('nesl_utility'); load_system('simulink'); load_system('sm_lib');
meshFolder=fullfile(char(robot.params.source.packageFolder),'meshes'); addpath(meshFolder);
visuals=find_system(model,'LookUnderMasks','all','FollowLinks','on','MaskType','File Solid');
for k=1:numel(visuals)
    [~,name,extension]=fileparts(get_param(visuals{k},'ExtGeomFileName'));
    assert(isfile(fullfile(meshFolder,[name extension])),'multibody:MissingMesh','Missing mesh %s.',name);
    set_param(visuals{k},'ExtGeomFileName',[name extension]);
end
set_param([model '/MechanismConfiguration'],'GravityVector',mat2str(gravity,16));
workspace=get_param(model,'ModelWorkspace');
workspace.DataSource='Model File';
initialQ=reshape(robot.params.joints.homePosition,1,[]);
dynamicsModel=copy(robot.model); dynamicsModel.Gravity=gravity;
holdTorque=gravityTorque(dynamicsModel,initialQ);
M=massMatrix(dynamicsModel,initialQ);
demoKd=2*.95*sqrt(demoKp.*diag(M).');
time=(0:.01:demoDuration_s).'; omega=2*pi/demoDuration_s;
amplitude=deg2rad(demoAmplitude_deg);
demoQ=[time initialQ+.5*(1-cos(omega*time)).*amplitude];
demoQd=[time .5*omega*sin(omega*time).*amplitude];
values=struct('initialQ',initialQ,'holdTorque',holdTorque,'demoKp',demoKp, ...
    'demoKd',demoKd,'demoQ',demoQ,'demoQd',demoQd);
for name=reshape(string(fieldnames(values)),1,[])
    workspace.assignin(char(name),values.(name));
end
add_block('simulink/Sources/In1',[model '/tau_Nm'],'PortDimensions',num2str(n),'Port','1');
add_block('simulink/Signal Routing/Demux',[model '/TorqueDemux'],'Outputs',num2str(n));
add_line(model,'tau_Nm/1','TorqueDemux/1');
signals={'q_rad','qd_rad_s','qdd_rad_s2'}; units={'rad','rad/s','rad/s^2'};
for j=1:3
    add_block('simulink/Signal Routing/Mux',[model '/' signals{j} '_Mux'],'Inputs',num2str(n));
    add_block('simulink/Sinks/Out1',[model '/' signals{j}],'Port',num2str(j));
    add_line(model,[signals{j} '_Mux/1'],[signals{j} '/1']);
end
for i=1:n
    joint=[model '/' char(robot.structure.jointNames(i))];
    set_param(joint,'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion', ...
        'SensePosition','on','SenseVelocity','on','SenseAcceleration','on', ...
        'PositionTargetSpecify','on','PositionTargetValue',sprintf('initialQ(%d)',i), ...
        'PositionTargetValueUnits','rad','PositionTargetPriority','High', ...
        'VelocityTargetSpecify','on','VelocityTargetValue','0', ...
        'VelocityTargetValueUnits','rad/s','VelocityTargetPriority','High');
    ports=get_param(joint,'PortHandles');
    assert(numel(ports.LConn)==2 && numel(ports.RConn)==4,'multibody:JointPorts','Unexpected joint ports.');
    converter=[model sprintf('/TorqueToPS_%d',i)];
    add_block('nesl_utility/Simulink-PS Converter',converter,'Unit','N*m');
    add_line(model,sprintf('TorqueDemux/%d',i),sprintf('TorqueToPS_%d/1',i));
    cp=get_param(converter,'PortHandles'); add_line(model,cp.RConn(1),ports.LConn(2),'autorouting','on');
    for j=1:3
        name=sprintf('%s_%d',signals{j},i); converter=[model '/' name];
        add_block('nesl_utility/PS-Simulink Converter',converter,'Unit',units{j});
        cp=get_param(converter,'PortHandles'); add_line(model,ports.RConn(j+1),cp.LConn(1),'autorouting','on');
        add_line(model,[name '/1'],sprintf('%s_Mux/%d',signals{j},i));
    end
end
% Measure the actual imported torch-tip frame, not Joint 6 / flange position.
sensor=[model '/TCP_Sensor'];
add_block('sm_lib/Frames and Transforms/Transform Sensor',sensor, ...
    'SenseXYZ','on','SenseR','on','MeasurementFrame','World');
sp=get_param(sensor,'PortHandles'); wp=get_param([model '/World'],'PortHandles');
tcpPorts=get_param([model '/tcp'],'PortHandles'); tcpFrame=[tcpPorts.LConn tcpPorts.RConn];
assert(numel(tcpFrame)==1 && numel(sp.RConn)==3,'multibody:TCPPorts','Unexpected TCP sensor ports.');
add_line(model,wp.RConn(1),sp.LConn(1),'autorouting','on');
add_line(model,tcpFrame(1),sp.RConn(1),'autorouting','on');
for entry={ {'tcp_position_m','m',3,4}, {'tcp_rotation','1',2,5} }
    data=entry{1}; name=data{1}; converter=[model '/' name '_FromPS'];
    add_block('nesl_utility/PS-Simulink Converter',converter,'Unit',data{2});
    cp=get_param(converter,'PortHandles'); add_line(model,sp.RConn(data{3}),cp.LConn(1),'autorouting','on');
    add_block('simulink/Sinks/Out1',[model '/' name],'Port',num2str(data{4}));
    add_line(model,[name '_FromPS/1'],[name '/1']);
end
Simulink.BlockDiagram.arrangeSystem(model);
blocks=find_system(model,'SearchDepth',1,'Type','Block');
Simulink.BlockDiagram.createSubsystem(getSimulinkBlockHandle(blocks),'Name','RobotPlant');
% The imported physical network and SI converters are now one reusable plant.
outer=find_system(model,'SearchDepth',1,'BlockType','Inport');
for k=1:numel(outer), delete_block(outer{k}); end
outer=find_system(model,'SearchDepth',1,'BlockType','Outport');
for k=1:numel(outer), delete_block(outer{k}); end
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');
for k=reshape(lines,1,[]), delete_line(k); end
plant=[model '/RobotPlant']; set_param(plant,'Position',[430 100 640 350]);
addDemoController(model);
addMeasurements(model);
add_line(model,'DemoController/1','RobotPlant/1','autorouting','on');
add_line(model,'RobotPlant/1','DemoController/1','autorouting','on');
add_line(model,'RobotPlant/2','DemoController/2','autorouting','on');
for i=1:5, add_line(model,sprintf('RobotPlant/%d',i),sprintf('Measurements/%d',i),'autorouting','on'); end
add_line(model,'DemoController/1','Measurements/6','autorouting','on');
for name={'RobotPlant','DemoController','Measurements'}
    set_param([model '/' name{1}],'ContentPreviewEnabled','off');
end
% Keep the velocity-feedback return below block names, not through their text.
controllerPorts=get_param([model '/DemoController'],'PortHandles');
feedback=get_param(controllerPorts.Inport(2),'Line'); points=get_param(feedback,'Points');
start=points(1,:); finish=get_param(controllerPorts.Inport(2),'Position');
set_param(feedback,'Points',[start;start+[20 0];start(1)+20 415;15 415;15 finish(2);finish]);
measurementPorts=get_param([model '/Measurements'],'PortHandles');
torqueBranch=get_param(measurementPorts.Inport(6),'Line'); points=get_param(torqueBranch,'Points');
start=points(1,:); finish=get_param(measurementPorts.Inport(6),'Position');
set_param(torqueBranch,'Points',[start;start+[20 0];start(1)+20 390;800 390;800 finish(2);finish]);
note=Simulink.Annotation(model,sprintf(['robot_ver2 | Ideal joint-torque control bench\n' ...
    'Joint order: J1..J6. Torque: N m. Angles: rad. Replace DemoController with your controller.\n' ...
    'No motor/gearbox dynamics, torque ratings or collision/contact response are assumed.']));
note.Position=[40 15];
set_param(model,'SolverType','Variable-step','Solver','ode15s','RelTol','1e-6', ...
    'AbsTol','1e-8','MaxStep','.01','StopTime',num2str(demoDuration_s), ...
    'ReturnWorkspaceOutputs','on','SignalLogging','on', ...
    'SimMechanicsOpenEditorOnUpdate','on');
meshCallback=['addpath(fullfile(fileparts(get_param(bdroot,''FileName'')), ' ...
    '''..'',''CAD'',''URDF_Assembly_Robot_ver2'',''meshes''));'];
set_param(model,'PostLoadFcn',meshCallback,'InitFcn',meshCallback);
info.robotID='robot_ver2'; info.schemaVersion=1;
info.urdfSHA256=sha256(char(robot.params.source.urdf));
info.jointNames=robot.structure.jointNames; info.gravity=gravity;
info.units=struct('torque','N m','position','rad','velocity','rad/s','acceleration','rad/s^2');
info.source='robots/robot_ver2/CAD/URDF_Assembly_Robot_ver2/urdf/URDF_Assembly_Robot_ver2.urdf';
info.builtAt=char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
info.demoOnly=true; info.motorRatingsIncluded=false; info.contactsIncluded=false;
workspace.assignin('multibodyInfo',info);
save_system(model,modelFile);
fprintf('Saved: %s\n',modelFile);
open_system(model); set_param(model,'ZoomFactor','FitSystem');
end

function addDemoController(model)
parent=[model '/DemoController']; add_block('built-in/Subsystem',parent,'Position',[65 140 250 285]);
for i=1:2
    name={'q_rad','qd_rad_s'};
    add_block('simulink/Sources/In1',[parent '/' name{i}],'Port',num2str(i),'Position',[30 70+120*(i-1) 60 90+120*(i-1)]);
end
add_block('simulink/Sources/From Workspace',[parent '/AngleReference'], ...
    'VariableName','demoQ','Position',[30 20 150 45]);
add_block('simulink/Sources/From Workspace',[parent '/SpeedReference'], ...
    'VariableName','demoQd','Position',[30 140 150 165]);
for i=1:2
    name={'PositionError','VelocityError'}; gain={'Kp','Kd'}; value={'demoKp','demoKd'}; y=30+120*(i-1);
    add_block('simulink/Math Operations/Sum',[parent '/' name{i}],'Inputs','+-','Position',[210 y 235 y+40]);
    add_block('simulink/Math Operations/Gain',[parent '/' gain{i}],'Gain',value{i}, ...
        'Multiplication','Element-wise(K.*u)','Position',[285 y 355 y+40]);
end
add_block('simulink/Sources/Constant',[parent '/GravityAtHome'], ...
    'Value','holdTorque','Position',[285 255 390 280]);
add_block('simulink/Math Operations/Sum',[parent '/TorqueSum'],'Inputs','+++','Position',[440 90 465 160]);
add_block('simulink/Sinks/Out1',[parent '/tau_Nm'],'Port','1','Position',[530 115 560 135]);
add_line(parent,'AngleReference/1','PositionError/1'); add_line(parent,'q_rad/1','PositionError/2');
add_line(parent,'SpeedReference/1','VelocityError/1'); add_line(parent,'qd_rad_s/1','VelocityError/2');
add_line(parent,'PositionError/1','Kp/1'); add_line(parent,'VelocityError/1','Kd/1');
add_line(parent,'Kp/1','TorqueSum/1','autorouting','on'); add_line(parent,'Kd/1','TorqueSum/2','autorouting','on');
add_line(parent,'GravityAtHome/1','TorqueSum/3','autorouting','on'); add_line(parent,'TorqueSum/1','tau_Nm/1');
end

function addMeasurements(model)
parent=[model '/Measurements']; add_block('built-in/Subsystem',parent,'Position',[855 95 1045 360]);
names={'q_rad','qd_rad_s','qdd_rad_s2','tcp_position_m','tcp_rotation','tau_Nm'};
variables={'mb_q','mb_qd','mb_qdd','mb_tcp_position','mb_tcp_rotation','mb_tau'};
for i=1:numel(names)
    add_block('simulink/Sources/In1',[parent '/' names{i}],'Port',num2str(i),'Position',[25 40*i 55 40*i+20]);
    add_block('simulink/Sinks/To Workspace',[parent '/' variables{i}], ...
        'VariableName',variables{i},'SaveFormat','Timeseries', ...
        'Position',[160 40*i-5 280 40*i+25]);
    add_line(parent,[names{i} '/1'],[variables{i} '/1']);
end
add_block('simulink/Sinks/Scope',[parent '/JointSignals'],'NumInputPorts','3','Position',[380 70 430 190]);
scope=get_param([parent '/JointSignals'],'ScopeConfiguration');
scope.LayoutDimensions=[3 1]; scope.AxesScaling='Auto'; scope.ShowLegend=true;
titles={'Joint angles (J1..J6)','Joint speeds (J1..J6)','Applied joint torques (J1..J6)'};
labels={'rad','rad/s','N m'};
for i=1:3
    scope.ActiveDisplay=i; scope.Title=titles{i}; scope.YLabel=labels{i};
end
scope.ActiveDisplay=1;
add_line(parent,'q_rad/1','JointSignals/1','autorouting','on');
add_line(parent,'qd_rad_s/1','JointSignals/2','autorouting','on');
add_line(parent,'tau_Nm/1','JointSignals/3','autorouting','on');
end

function value=sha256(path)
file=fopen(path,'rb'); assert(file>=0,'multibody:SourceRead','Cannot read URDF.');
cleanup=onCleanup(@() fclose(file)); data=fread(file,Inf,'*uint8');
digest=java.security.MessageDigest.getInstance('SHA-256'); digest.update(data);
value=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
