function layout_robot_testbench(model)
% Layout-only helper. Preserve endpoint handles and regenerate routing.
if getSimulinkBlockHandle([model '/Measurements'])<0
    sinks=find_system(model,'SearchDepth',1,'BlockType','ToWorkspace');
    Simulink.BlockDiagram.createSubsystem(getSimulinkBlockHandle(sinks),'Name','Measurements');
end
set_param([model '/SensorReference'],'Position',[70 130 420 230],'ContentPreviewEnabled','off');
set_param([model '/AdmittanceReference'],'Position',[510 130 840 230],'ContentPreviewEnabled','off');
set_param([model '/DriveAndController'],'Position',[70 350 340 620]);
set_param([model '/RobotPlant'],'Position',[570 350 830 620],'ContentPreviewEnabled','off');
set_param([model '/Measurements'],'Position',[1040 315 1260 650],'ContentPreviewEnabled','off');
set_param([model '/UnusedButterworth'],'Position',[510 65 530 85]);
set_param([model '/DriveAndController'],'Mask','on','MaskDisplay', ...
    sprintf(['disp(''Demo servo'');\n' ...
    'port_label(''input'',1,''Measured q'');port_label(''input'',2,''Measured qd'');\n' ...
    'port_label(''input'',3,''Lab twist'');port_label(''input'',4,''Filtered wrench'');\n' ...
    'port_label(''output'',1,''Joint torque'');port_label(''output'',2,''Applied wrench'');\n' ...
    'port_label(''output'',3,''Diagnostics'');']));
annotations=find_system(model,'FindAll','on','SearchDepth',1,'Type','annotation');
for k=reshape(annotations,1,[]),set_param(k,'Position',[55 10]);end
blocks=find_system(model,'SearchDepth',1,'Type','Block');edges=zeros(0,2);
for k=1:numel(blocks)
    ports=get_param(blocks{k},'PortHandles');
    for destination=reshape(ports.Inport,1,[])
        line=get_param(destination,'Line');if line<0,continue;end
        source=get_param(line,'SrcPortHandle');
        while source<0,line=get_param(line,'LineParent');source=get_param(line,'SrcPortHandle');end
        edges(end+1,:)=[source destination]; %#ok<AGROW>
    end
end
lines=find_system(model,'FindAll','on','SearchDepth',1,'Type','line');roots=[];
for k=reshape(lines,1,[]),if get_param(k,'LineParent')<0,roots(end+1)=k;end,end %#ok<AGROW>
for k=roots,delete_line(k);end
for k=1:size(edges,1)
    source=edges(k,1);destination=edges(k,2);
    line=add_line(model,source,destination,'autorouting','on');
    if strcmp(get_param(source,'Parent'),[model '/RobotPlant']) && ...
            strcmp(get_param(destination,'Parent'),[model '/DriveAndController'])
        a=get_param(source,'Position');b=get_param(destination,'Position');port=get_param(destination,'PortNumber');
        right=870+20*port;left=30-20*port;bottom=710+35*port;
        set_param(line,'Points',[a;right a(2);right bottom;left bottom;left b(2);b]);
    end
end
end
