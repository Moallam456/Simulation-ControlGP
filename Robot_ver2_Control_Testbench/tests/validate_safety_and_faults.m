function report=validate_safety_and_faults()
root=fileparts(fileparts(mfilename('fullpath')));addpath(root,fullfile(root,'src'),fullfile(root,'control'));
cfg=testbench_config();robot=testbench_robot(cfg);report=struct();
cache=testbench_collision_cache(robot);stream=RandStream('mt19937ar','Seed',91);
for k=1:12
    q=cfg.home_rad+deg2rad(.03)*(rand(stream,1,6)*2-1);
    [fast,cache]=testbench_safety(robot,q,cfg,q,true,cache);exact=testbench_safety(robot,q,cfg);
    assert(fast.accepted==exact.accepted && fast.selfClearance_m<=exact.selfClearance_m+1e-10);
    assert(fast.environmentClearance_m<=exact.environmentClearance_m+1e-10);
end
report.boundChecks=cache.boundChecks;report.exactChecks=cache.exactChecks;assert(cache.boundChecks>0);
blocked=cfg;blocked.environment=struct('name','Blocking fixture','size_m',[4 4 4], ...
    'center_m',[0 0 0],'rpy_rad',[0 0 0]);
e=testbench_safety(robot,cfg.home_rad,blocked);assert(~e.accepted && contains(e.reason,'ENVIRONMENT'));
report.environmentRejection=e.reason;
linear=cfg;linear.minimumLinearInverseCondition=.99;
e=testbench_safety(robot,cfg.home_rad,linear);assert(strcmp(e.reason,'LINEAR_INVERSE_CONDITION'));
% Deliberately raise the guard to exercise the separate full-task failure path.
full=cfg;full.demo.fullJacobianMinimumSigma=100;
e=testbench_safety(robot,cfg.home_rad,full);assert(strcmp(e.reason,'FULL_TASK_SINGULARITY'));
bad=cfg.home_rad;bad(1)=4;e=testbench_safety(robot,cfg.home_rad,cfg,bad);assert(strcmp(e.reason,'JOINT_LIMIT'));
report.invalidHomeRouteRejected=true;
W=100*ones(200,6);M=cfg.demo.virtualMass;D=cfg.demo.virtualDamping;
v=rb2_admittance_native(W,M,D,.002,.01*ones(1,6),.1*ones(1,6),.004);
assert(max(abs(v),[],'all')<=.01+eps);report.originalCLimiterPassed=true;
refused=false;try,invalid=rb2_admittance_native(W,M,D,.1,ones(1,6),ones(1,6),.004);catch e,refused=strcmp(e.identifier,'testbench:AdmittanceFault');end %#ok<NASGU>
assert(refused);report.originalCTimingFaultPassed=true;
p=testbench_packet('encode_command',struct('sequence',1,'enabled',1,'time',0,'q',cfg.home_rad));
p(1)=0;[~,valid]=testbench_packet('decode_command',p);assert(~valid);
report.pass=true;disp(report);
end
