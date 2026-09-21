function report = checkTrajectoryConvergence(robot,testCase,scenario,coarse,tolerance)
% CHECKTRAJECTORYCONVERGENCE Repeat identical geometry at dt/2 and dt/4.
report.pass=false;
report.reason="";
report.timeSteps=testCase.motion.timeStep./[1 2 4];
report.metrics=zeros(3,4,robot.structure.dof);
report.metrics(1,:,:)=reshape(extractMetrics(coarse),1,4,[]);
for level=2:3
    c=testCase;
    c.motion.timeStep=report.timeSteps(level);
    [trajectory,validation]=generateTrajectory(robot,c);
    if ~validation.pass
        report.reason="Trajectory validation failed at refined time step: " + ...
            validation.reason;
        return;
    end
    dynamics=analyzeTrajectoryDynamics(robot,trajectory,scenario);
    report.metrics(level,:,:)=reshape(extractMetrics(dynamics),1,4,[]);
end
new=squeeze(report.metrics(3,:,:));
old=squeeze(report.metrics(2,:,:));
scale=max(max(abs(new),[],2),[1e-4;1e-3;1e-2;1e-2]);
report.relativeChange=abs(new-old)./scale;
report.maxRelativeChange=max(report.relativeChange,[],'all');
report.tolerance=tolerance;
report.pass=all(report.relativeChange(:)<=tolerance);
if ~report.pass
    report.reason="Peak speed/acceleration/torque or RMS torque has not converged with dt refinement.";
end
end

function metrics = extractMetrics(dynamics)
summary=dynamics.summaryTable;
metrics=[summary.MaxAbsSpeed_rad_s.'; ...
    summary.MaxAbsAcceleration_rad_s2.'; ...
    summary.PeakAbsTorque_Nm.'; ...
    summary.RMSTorqueOverTrajectory_Nm.'];
end
