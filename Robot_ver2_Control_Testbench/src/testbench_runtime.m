function testbench_runtime(block)
% Discrete controller/actuator/transport boundary. Multibody owns physical state.
block.NumDialogPrms=0; block.NumInputPorts=4; block.NumOutputPorts=3;
for i=1:4
    block.InputPort(i).Dimensions=6;block.InputPort(i).DatatypeID=0;
    block.InputPort(i).Complexity='Real';block.InputPort(i).DirectFeedthrough=false;
end
block.OutputPort(1).Dimensions=6; block.OutputPort(2).Dimensions=6;
block.OutputPort(3).Dimensions=19;
for i=1:3,block.OutputPort(i).DatatypeID=0;block.OutputPort(i).Complexity='Real';end
cfg=testbench_config();
if isappdata(0,'RobotTestbenchConfig'),cfg=getappdata(0,'RobotTestbenchConfig');end
block.SampleTimes=[cfg.sampleTime_s 0];block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('Start',@start);block.RegBlockMethod('Outputs',@outputs);
block.RegBlockMethod('Update',@update);block.RegBlockMethod('Terminate',@finish);
end

function start(block)
cfg=testbench_config();if isappdata(0,'RobotTestbenchConfig'),cfg=getappdata(0,'RobotTestbenchConfig');end
if strcmp(cfg.mode,'hardware_validation'),testbench_require_hardware(cfg);end
root=fileparts(fileparts(mfilename('fullpath')));w=get_param(bdroot(block.BlockHandle),'ModelWorkspace');
assert(strcmp(w.getVariable('packagedUrdfSHA256'),testbench_hash(fullfile(root, ...
    'mechanics','URDF_Assembly_Robot_ver2','urdf','URDF_Assembly_Robot_ver2.urdf'))), ...
    'testbench:StaleMechanics','URDF changed; rebuild model and native kinematics.');
s.cfg=cfg;s.robot=testbench_robot(cfg);s.ref=cfg.home_rad;s.lastQ=cfg.home_rad;
s.tau=gravityTorque(s.robot,cfg.home_rad);
s.motorTorque=s.tau./(cfg.demo.reduction.*cfg.demo.gearEfficiency);s.fault='';s.faultCode=0;
s.diagnostics=zeros(19,1);s.log=struct('t',{},'q',{},'reference',{},'torque',{},'motorTorque',{},'safety',{},'fault',{});
s.lastPacket=0;s.lastReceive=-Inf;s.peer='';s.udp=[];s.count=0;s.wall=tic;
s.homeRouteRequested=false;s.homeMessage='';s.runClock=tic;s.hasBeenEnabled=false;
s.safetyCache=testbench_collision_cache(s.robot);
check=testbench_safety(s.robot,s.lastQ,cfg);assert(check.accepted,'testbench:UnsafeInitialPose',check.reason);
if strcmp(cfg.controller,'external_can')
    s.udp=udpport('datagram','IPV4','LocalPort',cfg.adapter.commandPort);
end
if strcmp(cfg.controller,'c_sil')
    rb2_controller_native('reset',cfg.home_rad,cfg.demo.virtualMass,cfg.demo.virtualDamping, ...
        cfg.demo.admittanceSpeedLimit,cfg.demo.admittanceAccelerationLimit, ...
        cfg.demo.jointSpeedLimit_rad_s,cfg.demo.fullJacobianMinimumSigma,cfg.sampleTime_s);
end
runtimeState('put',block.BlockHandle,s);
end

function outputs(block)
s=runtimeState('output',block.BlockHandle);
block.OutputPort(1).Data=s.tau(:);
w=zeros(6,1);t=block.CurrentTime;
if t>=s.cfg.demo.wrenchActive_s(1) && t<s.cfg.demo.wrenchActive_s(2)
    % Smooth on/off avoids an infinite-bandwidth applied-force step.
    u=(t-s.cfg.demo.wrenchActive_s(1))/diff(s.cfg.demo.wrenchActive_s);
    w=s.cfg.demo.operatorWrench_base_tcp(:)*sin(pi*u)^2;
end
block.OutputPort(2).Data=w;block.OutputPort(3).Data=s.diagnostics;
end

function update(block)
s=runtimeState('get',block.BlockHandle);cfg=s.cfg;dt=cfg.sampleTime_s;t=block.CurrentTime;
q=reshape(block.InputPort(1).Data,1,6);qd=reshape(block.InputPort(2).Data,1,6);
if any(~isfinite([q qd])),s=trip(s,'NONFINITE_MEASURED_STATE',1);else
    [measured,s.safetyCache]=testbench_safety(s.robot,q,cfg,s.lastQ,true,s.safetyCache);
    if ~measured.accepted,s=trip(s,['MEASURED_' measured.reason],2);end
    if any(abs(qd)>cfg.demo.jointSpeedLimit_rad_s),s=trip(s,'MEASURED_SPEED_LIMIT',3);end
    proposed=s.ref;
    if isempty(s.fault)
        if ~s.homeRouteRequested
        switch cfg.controller
            case 'c_sil'
                try
                    proposed=rb2_controller_native('step',q,reshape(block.InputPort(4).Data,1,6),dt);
                catch exception
                    s=trip(s,['C_CONTROLLER_' exception.identifier],12);
                end
            case 'laboratory'
                twist=reshape(block.InputPort(3).Data,6,1);
                J=geometricJacobian(s.robot,q,'tcp');J=J([4:6 1:3],:);
                dq=J.'*((J*J.'+.02^2*eye(6))\twist);
                proposed=s.ref+dt*dq.';
            case 'demonstration'
                amplitude=deg2rad([3 2 -2 3 2 3]);
                proposed=cfg.home_rad+.5*(1-cos(2*pi*t/cfg.duration_s))*amplitude;
            case 'external_can'
                [s,proposed]=receiveCommand(s,q,qd,t);
        end
        end
        if isappdata(0,'RobotTestbenchHomeRequested') && getappdata(0,'RobotTestbenchHomeRequested')
            setappdata(0,'RobotTestbenchHomeRequested',false);
            if strcmp(cfg.controller,'external_can')
                s.homeMessage='GO HOME REFUSED: external Supervisor owns motion; use its HMI Home command.';
            else
                route=testbench_safety(s.robot,cfg.home_rad,cfg,q);
                if route.accepted,s.homeRouteRequested=true;else
                    s.homeMessage=['GO HOME REFUSED: ' route.reason];
                end
            end
        end
        if s.homeRouteRequested
            step=deg2rad(5)*dt;
            proposed=s.ref+max(-step,min(step,cfg.home_rad-s.ref));
        end
        if max(abs(proposed-s.ref))>cfg.adapter.maxJointStep_rad
            s=trip(s,'COMMAND_STEP',4);
        else
            [command,s.safetyCache]=testbench_safety(s.robot,proposed,cfg,s.ref,true,s.safetyCache);
            if ~command.accepted,s=trip(s,['COMMAND_' command.reason],5);else,s.ref=proposed;end
        end
    end
    if ~isempty(s.fault)
        % Demo-only active braking. No position teleport and no assumed brake.
        requested=gravityTorque(s.robot,q)-cfg.demo.kd_joint_Nm_s_rad.*qd;
    else
        requested=gravityTorque(s.robot,q)+cfg.demo.kp_joint_Nm_rad.*(s.ref-q)-cfg.demo.kd_joint_Nm_s_rad.*qd;
    end
    motor=requested./(cfg.demo.reduction.*cfg.demo.gearEfficiency);
    bounded=max(-cfg.demo.motorTorqueLimit_Nm,min(cfg.demo.motorTorqueLimit_Nm,motor));
    s.motorTorque=s.motorTorque+(1-exp(-dt/cfg.demo.motorTorqueTimeConstant_s))*(bounded-s.motorTorque);
    s.tau=s.motorTorque.*cfg.demo.reduction.*cfg.demo.gearEfficiency-cfg.demo.viscousFriction_Nm_s_rad.*qd;
    s.diagnostics=[measured.linearIndex;measured.fullSigma;measured.selfClearance_m; ...
        measured.environmentClearance_m;norm(s.ref-q);s.faultCode;double(any(bounded~=motor)); ...
        s.motorTorque(:);abs(s.tau(:))./(cfg.demo.reduction(:).*cfg.demo.motorTorqueLimit_Nm(:).*cfg.demo.gearEfficiency(:))];
    s.lastQ=q;s.count=s.count+1;
    if mod(s.count,500)==0,fprintf('Robot testbench: %.2f s simulated, %.1f s wall, fault=%s\n',t,toc(s.wall),s.fault);end
    if mod(s.count,10)==0 || s.count==1
        row=struct('t',t,'q',q,'reference',s.ref,'torque',s.tau,'motorTorque',s.motorTorque, ...
            'safety',measured,'fault',s.fault);s.log(end+1)=row;
        setappdata(0,'RobotTestbenchLive',s.log(end));drawnow limitrate;
    end
end
runtimeState('put',block.BlockHandle,s);
end

function [s,proposed]=receiveCommand(s,q,qd,t)
proposed=s.ref;cfg=s.cfg;
budget=0;
while s.udp.NumDatagramsAvailable>0 && budget<32
    budget=budget+1;
    d=read(s.udp,1,'uint8');b=uint8(d.Data);
    if ~strcmp(d.SenderAddress,cfg.adapter.peer)
        s=trip(s,'UNEXPECTED_CONTROLLER_PEER',13);break;
    end
    [packet,valid]=testbench_packet('decode_command',b);
    if ~valid,s=trip(s,'MALFORMED_COMMAND_PACKET',6);break;end
    if packet.sequence<=s.lastPacket,s=trip(s,'OLD_COMMAND_SEQUENCE',7);break;end
    if isfield(s,'commandSimulationTime') && packet.time<s.commandSimulationTime
        s=trip(s,'REVERSED_COMMAND_TIME',11);break;
    end
    if packet.enabled~=1
        if s.hasBeenEnabled,s=trip(s,'DRIVE_NOT_ENABLED',8);break;end
        packet.q=q; % Synthetic gravity-hold standby, not a physical brake.
    else
        s.hasBeenEnabled=true;
    end
    s.lastPacket=packet.sequence;s.lastReceive=toc(s.wall);s.peer=d.SenderAddress;
    proposed=packet.q;s.commandSimulationTime=packet.time;
end
if s.lastPacket>0
    if toc(s.wall)-s.lastReceive>cfg.adapter.staleTimeout_s,s=trip(s,'STALE_COMMAND',9);end
    feedback=testbench_packet('encode_feedback',struct('sequence',s.lastPacket,'time',t,'q',q,'qd',qd,'fault',s.faultCode));
    write(s.udp,feedback,'uint8',s.peer,cfg.adapter.feedbackPort);
elseif toc(s.wall)>cfg.startupTimeout_s
    s=trip(s,'NO_CONTROLLER_CONNECTION',10);
end
% Windows/WSL coupling is paced best-effort; measure rather than claim real time.
wait=t-toc(s.runClock);if wait>0,pause(min(wait,.01));end
end

function s=trip(s,reason,code)
if isempty(s.fault),s.fault=reason;s.faultCode=code;warning('testbench:LatchedFault','%s',reason);end
end

function finish(block)
s=runtimeState('get',block.BlockHandle);
setappdata(0,'RobotTestbenchResults',struct('log',{s.log},'fault',s.fault, ...
    'ratingsStatus','NOT EVALUATED','profile',s.cfg.demo.status,'elapsedWall_s',toc(s.wall), ...
    'collisionExactChecks',s.safetyCache.exactChecks,'collisionBoundChecks',s.safetyCache.boundChecks, ...
    'homeMessage',s.homeMessage));
if ~isempty(s.udp),delete(s.udp);end
runtimeState('clear',block.BlockHandle);
end

function result=runtimeState(action,handle,value)
% Keep mesh-bearing objects out of Simulink UserData serialization/copying.
persistent owner data
result=[];
switch action
    case 'put',owner=handle;data=value;
    case 'get'
        assert(isequal(owner,handle)&&~isempty(data),'testbench:RuntimeState','No active state for this block.');result=data;
    case 'output'
        assert(isequal(owner,handle)&&~isempty(data),'testbench:RuntimeState','No active state for this block.');
        result=struct('tau',data.tau,'diagnostics',data.diagnostics,'cfg',data.cfg);
    case 'clear',owner=[];data=[];
end
end
