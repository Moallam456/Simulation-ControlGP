function build_robot_testbench()
% BUILD_ROBOT_TESTBENCH Reproducible R2024b models; does not touch upstream files.
root=fileparts(mfilename('fullpath')); addpath(root,fullfile(root,'src'),fullfile(root,'control'));
cfg=testbench_config(); export_robot_geometry();
addpath(fullfile(root,'mechanics','URDF_Assembly_Robot_ver2','meshes'));
if ~isempty(mex.getCompilerConfigurations('C','Selected'))
    mex('-silent','-outdir',fullfile(root,'control'),'-output','rb2_native', ...
        fullfile(root,'control','robot_ver2_kinematics.c'),fullfile(root,'control','testbench_mex.c'));
    upstream=fullfile(root,'vendor','GPRobotControl_Simulation-15aa22efc071554fe06f1e42e1007633dee15cc9','HandGuiding');
    mex('-silent',['-I' upstream],'-outdir',fullfile(root,'control'),'-output','rb2_admittance_native', ...
        fullfile(root,'control','admittance_mex.c'),fullfile(upstream,'admittance_control.c'));
    u=fileparts(upstream);
    mex('-silent',['-I' u],['-I' fullfile(root,'integration')],['-I' fullfile(root,'control')], ...
        '-outdir',fullfile(root,'control'),'-output','rb2_controller_native', ...
        fullfile(root,'control','controller_mex.c'),fullfile(root,'control','robot_ver2_kinematics.c'), ...
        fullfile(root,'integration','robot_ver2_control_adapter.c'), ...
        fullfile(upstream,'admittance_control.c'),fullfile(upstream,'cartesian_joint_mapper.c'), ...
        fullfile(u,'ControlCore','Math','matrix6.c'),fullfile(u,'ControlCore','Analysis','singularity.c'));
else
    error('testbench:Compiler','Configure a C compiler with mex -setup C.');
end
robot=testbench_robot(cfg); check=testbench_safety(robot,cfg.home_rad,cfg);
assert(check.accepted,'testbench:HomeUnsafe','Configured home rejected: %s',check.reason);
buildReferences(root);
model='robot_ver2_testbench'; file=fullfile(root,[model '.slx']);
if bdIsLoaded(model)
    assert(strcmp(get_param(model,'Dirty'),'off'),'testbench:Unsaved','Save/close testbench edits first.');
    close_system(model,0);
end
source=fullfile(root,'mechanics','robot_ver2_mechanics_source.slx');
build_mechanics_source(root,cfg);
sourceHandle=load_system(source); sourceModel=get_param(sourceHandle,'Name');
sourceCallback=['addpath(fullfile(fileparts(get_param(bdroot,''FileName'')), ' ...
    '''URDF_Assembly_Robot_ver2'',''meshes''));'];
set_param(sourceModel,'PostLoadFcn',sourceCallback,'InitFcn',sourceCallback);
save_system(sourceModel,source);
clean=onCleanup(@() close_system(sourceModel,0));
new_system(model); load_system('simulink'); load_system('nesl_utility');load_system('sm_lib');
add_block([sourceModel '/RobotPlant'],[model '/RobotPlant'],'Position',[420 90 640 340]);
w=get_param(model,'ModelWorkspace');w.DataSource='Model File';w.assignin('initialQ',cfg.home_rad);
w.assignin('packagedUrdfSHA256',testbench_hash(fullfile(root,'mechanics','URDF_Assembly_Robot_ver2','urdf','URDF_Assembly_Robot_ver2.urdf')));
w.assignin('configurationSHA256',testbench_hash(fullfile(root,'testbench_config.m')));
for i=1:6
    set_param([model '/RobotPlant/Joint_' num2str(i)],'PositionTargetValue',sprintf('initialQ(%d)',i));
end
set_param([model '/RobotPlant/MechanismConfiguration'],'GravityVector',mat2str(cfg.gravity_m_s2,17));
addWrenchPort(model);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[model '/DriveAndController'], ...
    'FunctionName','testbench_runtime','Position',[70 100 280 260]);
add_line(model,'RobotPlant/1','DriveAndController/1','autorouting','on');
add_line(model,'RobotPlant/2','DriveAndController/2','autorouting','on');
add_line(model,'DriveAndController/1','RobotPlant/1','autorouting','on');
add_line(model,'DriveAndController/2','RobotPlant/2','autorouting','on');
M=cfg.demo.virtualMass(:);D=cfg.demo.virtualDamping(:);dt=cfg.sampleTime_s;
w.assignin('A6',(2*M-D*dt)./(2*M+D*dt));w.assignin('B6',dt./(2*M+D*dt));
w.assignin('cfg',cfg);w.assignin('bias_est',zeros(6,1));
w.assignin('alpha',1-exp(-2*pi*cfg.sensor.filterCutoff_Hz*dt));
[b,a]=butter(2,cfg.sensor.filterCutoff_Hz*dt*2);w.assignin('bButter',b);w.assignin('aButter',a);
load_system(fullfile(root,'control','rb2_sensor_reference.slx'));
load_system(fullfile(root,'control','rb2_admittance_reference.slx'));
add_block('built-in/Subsystem',[model '/SensorReference'],'ReferencedSubsystem','rb2_sensor_reference');
add_block('built-in/Subsystem',[model '/AdmittanceReference'],'ReferencedSubsystem','rb2_admittance_reference');
add_line(model,'DriveAndController/2','SensorReference/1','autorouting','on');
add_line(model,'SensorReference/2','AdmittanceReference/1','autorouting','on');
add_line(model,'AdmittanceReference/1','DriveAndController/3','autorouting','on');
add_line(model,'SensorReference/2','DriveAndController/4','autorouting','on');
add_block('simulink/Sinks/Terminator',[model '/UnusedButterworth']);
add_line(model,'SensorReference/1','UnusedButterworth/1');
add_block('simulink/Sinks/To Workspace',[model '/diagnostics'],'VariableName','tb_diagnostics', ...
    'SaveFormat','Timeseries');add_line(model,'DriveAndController/3','diagnostics/1');
names={'joint_position','joint_velocity','joint_acceleration','tcp_position','tcp_rotation'};
for i=1:5
    add_block('simulink/Sinks/To Workspace',[model '/' names{i}], ...
        'VariableName',['tb_' names{i}],'SaveFormat','Timeseries','Position',[790 30+70*i 950 60+70*i]);
    add_line(model,sprintf('RobotPlant/%d',i),[names{i} '/1'],'autorouting','on');
end
add_block('simulink/Sinks/To Workspace',[model '/joint_torque'],'VariableName','tb_joint_torque', ...
    'SaveFormat','Timeseries','Position',[790 470 950 500]);
add_line(model,'DriveAndController/1','joint_torque/1','autorouting','on');
note=Simulink.Annotation(model,sprintf(['robot_ver2 | SYNTHETIC ACTUATOR PROFILE\n' ...
    'Measured feedback closes the loop. Motor ratings: NOT EVALUATED.\n' ...
    'Controller/safety/transport diagnostics: open_robot_testbench.']));note.Position=[50 15];
set_param(model,'Solver','ode4','SolverType','Fixed-step','FixedStep',num2str(cfg.sampleTime_s/4,17), ...
    'RelTol','1e-6','AbsTol','1e-8','StopTime',num2str(cfg.duration_s), ...
    'ReturnWorkspaceOutputs','on','SimMechanicsOpenEditorOnUpdate','on');
callback=['addpath(fileparts(get_param(bdroot,''FileName'')), ' ...
    'fullfile(fileparts(get_param(bdroot,''FileName'')),''src''), ' ...
    'fullfile(fileparts(get_param(bdroot,''FileName'')),''control''), ' ...
    'fullfile(fileparts(get_param(bdroot,''FileName'')),''mechanics'',''URDF_Assembly_Robot_ver2'',''meshes''));'];
set_param(model,'PostLoadFcn',callback,'InitFcn',callback);
layout_robot_testbench(model);save_system(model,file);close_system(model,0);
fprintf('Built standalone model: %s\n',file);
end

function addWrenchPort(model)
p=[model '/RobotPlant'];
add_block('simulink/Sources/In1',[p '/wrench_base_tcp'],'Port','2','PortDimensions','6');
add_block('simulink/Signal Routing/Demux',[p '/WrenchSplit'],'Outputs','[3 3]');
add_line(p,'wrench_base_tcp/1','WrenchSplit/1');
b=[p '/AppliedWrench'];
add_block('sm_lib/Forces and Torques/External Force and Torque',b);
set_param(b,'EnableForce','on','EnableTorque','on');
params=get_param(b,'DialogParameters');
% Block parameter names are checked before the generated model is saved.
if isfield(params,'ForceResolutionFrame'),set_param(b,'ForceResolutionFrame','World');end
if isfield(params,'TorqueResolutionFrame'),set_param(b,'TorqueResolutionFrame','World');end
bp=get_param(b,'PortHandles');
tcp=get_param([p '/tcp'],'PortHandles');frame=[tcp.LConn tcp.RConn];
% R2024b External Force and Torque defaults: force/torque vectors in world.
all=[bp.LConn bp.RConn];
assert(numel(all)==3,'testbench:WrenchPorts','Expected frame, force and torque ports.');
% Port labels on the physical block identify the frame and vector signals.
for k=1:2
    name=sprintf('WrenchToPS%d',k);unit={'N','N*m'};
    add_block('nesl_utility/Simulink-PS Converter',[p '/' name],'Unit',unit{k});
    add_line(p,sprintf('WrenchSplit/%d',k),[name '/1']);
end
cp1=get_param([p '/WrenchToPS1'],'PortHandles');cp2=get_param([p '/WrenchToPS2'],'PortHandles');
add_line(p,frame(1),bp.RConn(1),'autorouting','on');
add_line(p,cp1.RConn(1),bp.LConn(1),'autorouting','on');
add_line(p,cp2.RConn(1),bp.LConn(2),'autorouting','on');
end

function buildReferences(root)
% Reconstruct the inspected simple R2025b reference equations using R2024b blocks.
model='rb2_admittance_reference'; if bdIsLoaded(model),close_system(model,0);end
new_system(model,'Subsystem');
add_block('simulink/Sources/In1',[model '/Wrench']);
add_block('simulink/Discrete/Unit Delay',[model '/PreviousWrench'],'SampleTime','.002','InitialCondition','zeros(6,1)');
add_block('simulink/Discrete/Unit Delay',[model '/PreviousVelocity'],'SampleTime','.002','InitialCondition','zeros(6,1)');
add_block('simulink/Math Operations/Sum',[model '/WrenchSum'],'Inputs','++');
for name={'A6','B6'}
    add_block('simulink/Math Operations/Gain',[model '/' name{1}],'Gain',name{1},'Multiplication','Element-wise(K.*u)');
end
add_block('simulink/Math Operations/Sum',[model '/VelocitySum'],'Inputs','++');
add_block('simulink/Sinks/Out1',[model '/Twist']);
for line={{'Wrench/1','PreviousWrench/1'},{'Wrench/1','WrenchSum/1'}, ...
        {'PreviousWrench/1','WrenchSum/2'},{'WrenchSum/1','B6/1'}, ...
        {'B6/1','VelocitySum/1'},{'PreviousVelocity/1','A6/1'}, ...
        {'A6/1','VelocitySum/2'},{'VelocitySum/1','PreviousVelocity/1'},{'VelocitySum/1','Twist/1'}}
    a=line{1};add_line(model,a{1},a{2},'autorouting','on');
end
Simulink.BlockDiagram.arrangeSystem(model); save_system(model,fullfile(root,'control',[model '.slx']));close_system(model,0);
model='rb2_sensor_reference';if bdIsLoaded(model),close_system(model,0);end
new_system(model,'Subsystem');
add_block('simulink/Sources/In1',[model '/TrueWrench']);
add_block('simulink/Math Operations/Gain',[model '/Coupling'], ...
    'Gain','cfg.sensor.coupling','Multiplication','Matrix(K*u) (u vector)');
add_block('simulink/Math Operations/Bias',[model '/Bias'],'Bias','cfg.sensor.bias');
add_block('simulink/Sources/Random Number',[model '/Noise'], ...
    'Mean','zeros(6,1)','Variance','cfg.sensor.noise_std.^2','Seed','[17;18;19;20;21;22]','SampleTime','.002');
add_block('simulink/Sources/Ramp',[model '/Drift'], ...
    'slope','cfg.sensor.drift_per_s','start','0','InitialOutput','zeros(6,1)');
add_block('simulink/Math Operations/Sum',[model '/Measurement'],'Inputs','+++');
add_block('simulink/Math Operations/Bias',[model '/Tare'],'Bias','-bias_est');
add_block('simulink/Discrete/Discrete Filter',[model '/Butterworth'], ...
    'Numerator','bButter','Denominator','aButter');
add_block('simulink/Discrete/Discrete Filter',[model '/EMA'], ...
    'Numerator','alpha','Denominator','[1 -(1-alpha)]');
add_block('simulink/Sinks/Out1',[model '/ButterworthWrench'],'Port','1');
add_block('simulink/Sinks/Out1',[model '/EMAWrench'],'Port','2');
for line={{'TrueWrench/1','Coupling/1'},{'Coupling/1','Bias/1'}, ...
        {'Bias/1','Measurement/1'},{'Noise/1','Measurement/2'},{'Drift/1','Measurement/3'}, ...
        {'Measurement/1','Tare/1'},{'Tare/1','Butterworth/1'},{'Tare/1','EMA/1'}, ...
        {'Butterworth/1','ButterworthWrench/1'},{'EMA/1','EMAWrench/1'}}
    a=line{1};add_line(model,a{1},a{2},'autorouting','on');
end
Simulink.BlockDiagram.arrangeSystem(model);save_system(model,fullfile(root,'control',[model '.slx']));close_system(model,0);
end
