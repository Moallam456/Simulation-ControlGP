function report=validate_integrator()
root=fileparts(fileparts(mfilename('fullpath')));addpath(root,fullfile(root,'src'),fullfile(root,'control'));
cfg=testbench_config();cfg.demo.wrenchActive_s=[.01 .09];setappdata(0,'RobotTestbenchConfig',cfg);
load_system(fullfile(root,'robot_ver2_testbench.slx'));
in=Simulink.SimulationInput('robot_ver2_testbench');
in=in.setModelParameter('SolverType','Fixed-step','Solver','ode4','FixedStep','.0005', ...
    'StopTime','.1','SimMechanicsOpenEditorOnUpdate','off');
a=sim(in);in=in.setModelParameter('FixedStep','.00025');b=sim(in);
report.finalAngleDifference_rad=max(abs(a.tb_joint_position.Data(end,:)-b.tb_joint_position.Data(end,:)));
assert(report.finalAngleDifference_rad<1e-5,'testbench:IntegratorConvergence','Refine the integration step.');
robot=testbench_robot(cfg);residual=0;
indices=round(linspace(3,numel(a.tb_joint_position.Time)-2,10));
for i=indices
    t=a.tb_joint_position.Time(i);q=a.tb_joint_position.Data(i,:);
    qd=a.tb_joint_velocity.Data(i,:);qdd=a.tb_joint_acceleration.Data(i,:);
    tau=interp1(a.tb_joint_torque.Time,a.tb_joint_torque.Data,t,'previous');
    tick=floor((t+1e-12)/cfg.sampleTime_s)*cfg.sampleTime_s;wrench=zeros(6,1);
    if tick>=cfg.demo.wrenchActive_s(1) && tick<cfg.demo.wrenchActive_s(2)
        u=(tick-cfg.demo.wrenchActive_s(1))/diff(cfg.demo.wrenchActive_s);
        wrench=cfg.demo.operatorWrench_base_tcp(:)*sin(pi*u)^2;
    end
    J=geometricJacobian(robot,q,'tcp');external=J.'*wrench([4:6 1:3]);
    predicted=inverseDynamics(robot,q,qd,qdd)-external.';
    residual=max(residual,max(abs(predicted-tau)));
end
report.maxDynamicsResidual_Nm=residual;assert(residual<1e-4,'testbench:ForceDynamicsMismatch','Check wrench frame/point and torque sampling.');
report.pass=true;disp(report);
end
