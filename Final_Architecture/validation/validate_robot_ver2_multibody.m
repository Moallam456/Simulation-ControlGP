function report=validate_robot_ver2_multibody()
% VALIDATE_ROBOT_VER2_MULTIBODY Cross-check the physical plant against the RBT.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
folder=fullfile(root,'robots','robot_ver2','multibody');
model='robot_ver2_multibody'; file=fullfile(folder,[model '.slx']);
assert(isfile(file),'multibody:BuildFirst','Run build_robot_ver2_multibody first.');
wasLoaded=bdIsLoaded(model); wasOpen=false;
if wasLoaded
    assert(strcmp(get_param(model,'Dirty'),'off'),'multibody:UnsavedChanges','Save your model before validation.');
    wasOpen=strcmp(get_param(model,'Open'),'on');
end
oldPath=path; oldConfig=Simulink.fileGenControl('getConfig');
scratch=[tempname '_robot_ver2_multibody_test']; mkdir(scratch);
cleanup=onCleanup(@() restoreSession(model,file,wasLoaded,wasOpen,oldPath,oldConfig,scratch));
Simulink.fileGenControl('set','CacheFolder',fullfile(scratch,'cache'), ...
    'CodeGenFolder',fullfile(scratch,'codegen'),'createDir',true);
load_system(file);
set_param(model,'SimMechanicsOpenEditorOnUpdate','off');
robot=loadRobot('robot_ver2'); dynamicsModel=copy(robot.model); dynamicsModel.Gravity=[0 0 -9.81];
plant=[model '/RobotPlant'];
joints=find_system(plant,'LookUnderMasks','all','FollowLinks','on','MaskType','Revolute Joint');
assert(numel(joints)==6);
for i=1:6
    b=[plant '/Joint_' num2str(i)];
    assert(strcmp(get_param(b,'TorqueActuationMode'),'InputTorque'));
    assert(strcmp(get_param(b,'MotionActuationMode'),'ComputedMotion'));
    assert(strcmp(get_param(b,'LowerLimitBoundUnits'),'rad'));
    assert(abs(str2double(get_param(b,'LowerLimitBound'))+pi)<1e-12);
    assert(abs(str2double(get_param(b,'UpperLimitBound'))-pi)<1e-12);
    body=robot.model.Bodies{i};
    mass=str2double(get_param([plant '/' body.Name '/Inertia'],'Mass'));
    assert(abs(mass-body.Mass)<1e-12);
end
visuals=find_system(plant,'LookUnderMasks','all','FollowLinks','on','MaskType','File Solid');
assert(numel(visuals)==7);
for i=1:numel(visuals)
    assert(isfile(which(get_param(visuals{i},'ExtGeomFileName'))));
    assert(str2double(get_param(visuals{i},'Mass'))==0,'Visuals must not duplicate imported inertial mass.');
end
assert(getSimulinkBlockHandle([plant '/tcp_fixed'])>0);
assert(str2double(get_param([plant '/tcp/Inertia'],'Mass'))==0);
assert(strcmp(get_param([plant '/MechanismConfiguration'],'GravityVector'),'[0 0 -9.8100000000000005]') || ...
    norm(str2num(get_param([plant '/MechanismConfiguration'],'GravityVector'))-[0 0 -9.81])<1e-12); %#ok<ST2NM>
report=struct('robotID',"robot_ver2",'sixTorqueDrivenJoints',true,'massesMatch',true, ...
    'tcpFramePresent',true,'maxTorqueResidual_Nm',0,'maxPositionError_m',0,'maxRotationError',0);
configurations=[zeros(1,6);deg2rad([10 8 -6 12 10 15]);deg2rad([0 15 130 0 90 0])];
for k=1:size(configurations,1)
    q=configurations(k,:); g=gravityTorque(dynamicsModel,q);
    input=Simulink.SimulationInput(model);
    input=input.setModelParameter('StopTime','.01','MaxStep','.001','SimMechanicsOpenEditorOnUpdate','off');
    input=input.setVariable('initialQ',q,'Workspace',model);
    input=input.setVariable('demoKp',zeros(1,6),'Workspace',model);
    input=input.setVariable('demoKd',zeros(1,6),'Workspace',model);
    input=input.setVariable('holdTorque',g,'Workspace',model);
    held=sim(input);
    assert(max(abs(held.mb_q.Data-q),[],'all')<1e-6,'multibody:GravityMismatch','Gravity torques do not hold the imported model.');
    [pError,rError]=checkTCP(robot,held,q);
    report.maxPositionError_m=max(report.maxPositionError_m,pError);
    report.maxRotationError=max(report.maxRotationError,rError);
    applied=g+[.7 -.8 .5 -.15 .1 -.05];
    input=input.setVariable('holdTorque',applied,'Workspace',model);
    motion=sim(input);
    expected=forwardDynamics(dynamicsModel,q,zeros(1,6),applied);
    assert(norm(motion.mb_qdd.Data(1,:)-expected,Inf)<1e-5,'multibody:AccelerationMismatch','Joint acceleration/sign mismatch.');
    report.maxTorqueResidual_Nm=max(report.maxTorqueResidual_Nm,checkDynamics(dynamicsModel,motion));
end
fprintf('Gravity, FK, joint axes and forced accelerations agree at three configurations.\n');
demo=sim(Simulink.SimulationInput(model).setModelParameter('SimMechanicsOpenEditorOnUpdate','off'));
assert(all(isfinite(demo.mb_q.Data),'all') && all(isfinite(demo.mb_tau.Data),'all'));
report.demoJointTravel_deg=rad2deg(max(demo.mb_q.Data)-min(demo.mb_q.Data));
assert(all(report.demoJointTravel_deg>1),'Every demonstration joint should visibly move.');
report.maxTorqueResidual_Nm=max(report.maxTorqueResidual_Nm,checkDynamics(dynamicsModel,demo));
assert(report.maxTorqueResidual_Nm<1e-4,'multibody:DynamicsMismatch','Simscape torques disagree with inverseDynamics.');
report.demoDuration_s=demo.mb_q.Time(end);
samples=unique(round(linspace(1,size(demo.mb_q.Data,1),25)));
collision=analyzeCollisions(robot,demo.mb_q.Data(samples,:),struct('minimumClearance',.01));
assert(all(collision.hasRequiredClearance),'multibody:DemoCollision','The demo needs a collision-clear reference.');
report.demoSampledSelfClearance_m=min(collision.minimumClearance);
% The SLX must run after relocation, with no original mesh folder on the path.
close_system(model,0);
portable=fullfile(scratch,'portable','multibody'); mkdir(portable);
copyfile(file,fullfile(portable,[model '.slx']));
meshSource=fullfile(char(robot.params.source.packageFolder),'meshes');
meshDestination=fullfile(scratch,'portable','CAD','URDF_Assembly_Robot_ver2','meshes');
mkdir(fileparts(meshDestination)); copyfile(meshSource,meshDestination);
rmpath(meshSource); rmpath(folder); addpath(portable,'-begin');
load_system(fullfile(portable,[model '.slx']));
resolved=which('robot_ver2_Link_1.STL');
assert(startsWith(resolved,fullfile(scratch,'portable')),'multibody:NotPortable','Meshes still resolve outside the relocated package.');
relocated=sim(Simulink.SimulationInput(model).setModelParameter( ...
    'StopTime','.01','SimMechanicsOpenEditorOnUpdate','off'));
checkTCP(robot,relocated,zeros(1,6));
report.portable=true; report.pass=true; disp(report);
end

function [positionError,rotationError]=checkTCP(robot,output,q)
T=forward_kinematics(robot,q);
p=squeeze(output.mb_tcp_position.getdatasamples(1));
R=squeeze(output.mb_tcp_rotation.getdatasamples(1)); R=reshape(R,3,3);
positionError=norm(p(:)-T(1:3,4)); rotationError=norm(R-T(1:3,1:3),'fro');
assert(positionError<1e-8,'multibody:TCPPosition','TCP position does not match FK.');
assert(rotationError<1e-8,'multibody:TCPOrientation','TCP orientation does not match FK.');
end

function errorNm=checkDynamics(model,output)
indices=unique(round(linspace(1,numel(output.mb_q.Time),min(80,numel(output.mb_q.Time)))));
errorNm=0;
for k=indices
    q=output.mb_q.Data(k,:); qd=output.mb_qd.Data(k,:); qdd=output.mb_qdd.Data(k,:);
    expected=inverseDynamics(model,q,qd,qdd);
    errorNm=max(errorNm,norm(expected-output.mb_tau.Data(k,:),Inf));
end
end

function restoreSession(model,file,wasLoaded,wasOpen,oldPath,oldConfig,scratch)
if bdIsLoaded(model), close_system(model,0); end
path(oldPath); Simulink.fileGenControl('setConfig','config',oldConfig);
if wasLoaded
    load_system(file);
    if wasOpen, open_system(model); end
end
% Only this validator's explicitly created, canonical temporary directory.
canonical=char(java.io.File(scratch).getCanonicalPath());
temporaryRoot=char(java.io.File(tempdir).getCanonicalPath());
if startsWith(canonical,[temporaryRoot filesep],'IgnoreCase',true) && ...
        endsWith(canonical,'_robot_ver2_multibody_test') && isfolder(canonical)
    [~,~]=rmdir(canonical,'s');
end
end
