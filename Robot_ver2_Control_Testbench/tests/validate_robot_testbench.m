function report=validate_robot_testbench()
% Independent package tests; no Final_Architecture calls/paths.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root,fullfile(root,'src'),fullfile(root,'control'));
cfg=testbench_config();robot=testbench_robot(cfg);
report=struct();report.hardwareMissing=numel(testbench_require_hardware(cfg,false));
refused=false;try,testbench_require_hardware(cfg);catch e,refused=strcmp(e.identifier,'testbench:HardwareUnconfigured');end
assert(refused,'Unconfigured hardware must be rejected.');
assert(norm(getTransform(robot,zeros(1,6),'tcp','End_Effector')-[eye(3) [-.02287;.02053;.24577];0 0 0 1],'fro')<1e-10);
stream=RandStream('mt19937ar','Seed',23);q=(rand(stream,200,6)*2-1)*pi;
report.maxFKError=0;report.maxJacobianError=0;report.ikSolved=0;report.maxIKPositionError=0;
for k=1:size(q,1)
    T=getTransform(robot,q(k,:),'tcp');C=rb2_native('fk',q(k,:));
    report.maxFKError=max(report.maxFKError,norm(T-C,'fro'));
    if k<=10
        F=rb2_native('frames',q(k,:));names=[{robot.BaseName} robot.BodyNames];
        for body=1:8,assert(norm(F(:,:,body)-getTransform(robot,q(k,:),names{body}),'fro')<1e-10);end
    end
    J=geometricJacobian(robot,q(k,:),'tcp');CJ=rb2_native('jacobian',q(k,:));
    report.maxJacobianError=max(report.maxJacobianError,norm(J([4:6 1:3],:)-CJ,'fro'));
    solutions=rb2_native('ik',T,q(k,:));report.ikSolved=report.ikSolved+~isempty(solutions);
    for j=1:size(solutions,1)
        actual=getTransform(robot,solutions(j,:),'tcp');
        report.maxIKPositionError=max(report.maxIKPositionError,norm(actual(1:3,4)-T(1:3,4)));
        assert(norm(actual(1:3,1:3)-T(1:3,1:3),'fro')<2e-4);
    end
end
assert(report.maxFKError<1e-10 && report.maxJacobianError<1e-10);
assert(report.ikSolved>=198 && report.maxIKPositionError<1e-5);
report.home=testbench_safety(robot,cfg.home_rad,cfg);assert(report.home.accepted);
report.minimumMassEigenvalue=min(eig(massMatrix(robot,cfg.home_rad)));
assert(report.minimumMassEigenvalue>0);
assert(all(isfinite(gravityTorque(robot,cfg.home_rad))));
modelFile=fullfile(root,'robot_ver2_testbench.slx');load_system(modelFile);
for k=1:6
    name=robot.BodyNames{k};mass=str2double(get_param(['robot_ver2_testbench/RobotPlant/' name '/Inertia'],'Mass'));
    assert(abs(mass-robot.Bodies{k}.Mass)<1e-12);
end
cache=testbench_collision_cache(robot);
for k=1:12
    nearby=cfg.home_rad+deg2rad(.03)*(2*rand(stream,1,6)-1);
    [bounded,cache]=testbench_safety(robot,nearby,cfg,nearby,true,cache);
    exact=testbench_safety(robot,nearby,cfg);
    assert(bounded.accepted==exact.accepted);
    assert(bounded.selfClearance_m<=exact.selfClearance_m+1e-10);
    assert(bounded.environmentClearance_m<=exact.environmentClearance_m+1e-10);
end
assert(cache.boundChecks>0);report.collisionBoundChecks=cache.boundChecks;
bad=cfg.home_rad;bad(1)=4;rejection=testbench_safety(robot,bad,cfg);assert(~rejection.accepted);
p=testbench_packet('encode_command',struct('sequence',42,'enabled',1,'time',.002,'q',cfg.home_rad));
[d,valid]=testbench_packet('decode_command',p);assert(valid && isequal(d.q,cfg.home_rad));
[~,valid]=testbench_packet('decode_command',p(1:50));assert(~valid);
load_system(fullfile(root,'robot_ver2_testbench.slx'));
original=cfg;cfg.controller='demonstration';cfg.duration_s=.2;
setappdata(0,'RobotTestbenchConfig',cfg);
input=Simulink.SimulationInput('robot_ver2_testbench');
input=input.setModelParameter('StopTime','.2','SimMechanicsOpenEditorOnUpdate','off');
out=sim(input);assert(all(isfinite(out.tb_joint_position.Data),'all'));
results=getappdata(0,'RobotTestbenchResults');assert(isempty(results.fault));
assert(strcmp(results.ratingsStatus,'NOT EVALUATED'));
report.dynamicFinite=true;report.ratingsStatus=results.ratingsStatus;
setappdata(0,'RobotTestbenchConfig',original);
report.pass=true;disp(report);
end
