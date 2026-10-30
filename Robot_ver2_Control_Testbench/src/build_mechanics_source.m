function build_mechanics_source(root,cfg)
% Fresh smimport on each explicit rebuild: the packaged URDF is authoritative.
model='robot_ver2_mechanics_source';destination=fullfile(root,'mechanics',[model '.slx']);
if bdIsLoaded(model)
    assert(strcmp(get_param(model,'Dirty'),'off'),'testbench:UnsavedMechanics','Save/close mechanics edits first.');
    close_system(model,0);
end
scratch=tempname;mkdir(scratch);old=pwd;cleanup=onCleanup(@() cd(old));cd(scratch);
urdf=fullfile(root,'mechanics','URDF_Assembly_Robot_ver2','urdf','URDF_Assembly_Robot_ver2.urdf');
smimport(urdf,'ModelName',model);load_system('nesl_utility');load_system('simulink');
visuals=find_system(model,'LookUnderMasks','all','MaskType','File Solid');
for k=1:numel(visuals)
    [~,name,extension]=fileparts(get_param(visuals{k},'ExtGeomFileName'));
    set_param(visuals{k},'ExtGeomFileName',[name extension]);
    assert(str2double(get_param(visuals{k},'Mass'))==0,'testbench:DuplicateMass','URDF visual must not add inertial mass.');
end
set_param([model '/MechanismConfiguration'],'GravityVector',mat2str(cfg.gravity_m_s2,17));
add_block('simulink/Sources/In1',[model '/tau_Nm'],'Port','1','PortDimensions','6');
add_block('simulink/Signal Routing/Demux',[model '/TorqueSplit'],'Outputs','6');
add_line(model,'tau_Nm/1','TorqueSplit/1');
names={'q_rad','qd_rad_s','qdd_rad_s2'};units={'rad','rad/s','rad/s^2'};
for j=1:3
    add_block('simulink/Signal Routing/Mux',[model '/' names{j} '_Mux'],'Inputs','6');
    add_block('simulink/Sinks/Out1',[model '/' names{j}],'Port',num2str(j));
    add_line(model,[names{j} '_Mux/1'],[names{j} '/1']);
end
for i=1:6
    joint=[model '/Joint_' num2str(i)];
    set_param(joint,'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion', ...
        'SensePosition','on','SenseVelocity','on','SenseAcceleration','on', ...
        'PositionTargetSpecify','on','PositionTargetValue',sprintf('initialQ(%d)',i), ...
        'PositionTargetValueUnits','rad','PositionTargetPriority','High', ...
        'VelocityTargetSpecify','on','VelocityTargetValue','0','VelocityTargetValueUnits','rad/s','VelocityTargetPriority','High');
    ports=get_param(joint,'PortHandles');
    name=sprintf('Torque_%d',i);add_block('nesl_utility/Simulink-PS Converter',[model '/' name],'Unit','N*m');
    converter=get_param([model '/' name],'PortHandles');add_line(model,sprintf('TorqueSplit/%d',i),[name '/1']);
    add_line(model,converter.RConn(1),ports.LConn(2),'autorouting','on');
    for j=1:3
        name=sprintf('%s_%d',names{j},i);add_block('nesl_utility/PS-Simulink Converter',[model '/' name],'Unit',units{j});
        converter=get_param([model '/' name],'PortHandles');add_line(model,ports.RConn(j+1),converter.LConn(1),'autorouting','on');
        add_line(model,[name '/1'],sprintf('%s_Mux/%d',names{j},i));
    end
end
sensor=[model '/TCP_Sensor'];add_block('sm_lib/Frames and Transforms/Transform Sensor',sensor,'SenseXYZ','on','SenseR','on','MeasurementFrame','World');
s=get_param(sensor,'PortHandles');world=get_param([model '/World'],'PortHandles');tcp=get_param([model '/tcp'],'PortHandles');
frame=[tcp.LConn tcp.RConn];add_line(model,world.RConn(1),s.LConn(1),'autorouting','on');add_line(model,frame(1),s.RConn(1),'autorouting','on');
for k=1:2
    name={'tcp_position_m','tcp_rotation'};unit={'m','1'};index=[3 2];
    add_block('nesl_utility/PS-Simulink Converter',[model '/' name{k} '_FromPS'],'Unit',unit{k});
    p=get_param([model '/' name{k} '_FromPS'],'PortHandles');add_line(model,s.RConn(index(k)),p.LConn(1),'autorouting','on');
    add_block('simulink/Sinks/Out1',[model '/' name{k}],'Port',num2str(k+3));add_line(model,[name{k} '_FromPS/1'],[name{k} '/1']);
end
blocks=find_system(model,'SearchDepth',1,'Type','Block');
Simulink.BlockDiagram.createSubsystem(getSimulinkBlockHandle(blocks),'Name','RobotPlant');
for type={'Inport','Outport'}
    blocks=find_system(model,'SearchDepth',1,'BlockType',type{1});for k=1:numel(blocks),delete_block(blocks{k});end
end
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');for k=reshape(lines,1,[]),delete_line(k);end
set_param(model,'PostLoadFcn','addpath(fullfile(fileparts(get_param(bdroot,''FileName'')),''URDF_Assembly_Robot_ver2'',''meshes''));');
w=get_param(model,'ModelWorkspace');w.DataSource='Model File';w.assignin('initialQ',cfg.home_rad);
save_system(model,destination);close_system(model,0);
% Scratch import files are left in tempdir for diagnostics, not runtime use.
end
