function aggregate = aggregateActuatorSizingResults(robot,caseResults,gravity)
% AGGREGATEACTUATORSIZINGRESULTS Preserve paired samples and peak provenance.
dof=robot.structure.dof;
emptyEvent=struct('value',NaN,'caseID',"",'time_s',NaN, ...
    'q',[],'speed_rad_s',NaN,'torque_Nm',NaN,'acceleration_rad_s2',NaN);
emptyJoint=struct('joint',"",'peakPositiveTorque',emptyEvent, ...
    'peakNegativeTorque',emptyEvent,'peakAbsoluteTorque',emptyEvent, ...
    'peakAbsoluteSpeed',emptyEvent,'peakAbsoluteAcceleration',emptyEvent, ...
    'peakPositivePower',emptyEvent,'peakNegativePower',emptyEvent);
aggregate.joints=repmat(emptyJoint,dof,1);
aggregate.torqueSpeed=repmat(struct('speed_rad_s',[], ...
    'torque_Nm',[],'caseID',strings(0,1),'time_s',[]),dof,1);
aggregate.caseComparison=table();
aggregate.validCaseIDs=strings(0,1);
aggregate.gravity=gravity;
for j=1:dof
    aggregate.joints(j).joint=string(robot.structure.jointNames(j));
end
for i=1:numel(caseResults)
    c=caseResults(i);
    if c.status~="PASS" || isempty(c.dynamics), continue; end
    aggregate.validCaseIDs(end+1,1)=c.id;
    d=c.dynamics;
    for j=1:dof
        n=numel(d.time);
        paired=aggregate.torqueSpeed(j);
        paired.speed_rad_s=[paired.speed_rad_s;d.state.qd(:,j)];
        paired.torque_Nm=[paired.torque_Nm;d.torque.total(:,j)];
        paired.caseID=[paired.caseID;repmat(c.id,n,1)];
        paired.time_s=[paired.time_s;d.time];
        aggregate.torqueSpeed(j)=paired;
        names={'peakPositiveTorque','peakNegativeTorque','peakAbsoluteTorque', ...
            'peakAbsoluteSpeed','peakAbsoluteAcceleration', ...
            'peakPositivePower','peakNegativePower'};
        series={d.torque.total(:,j),d.torque.total(:,j), ...
            abs(d.torque.total(:,j)),abs(d.state.qd(:,j)), ...
            abs(d.state.qdd(:,j)),d.power.joint(:,j),d.power.joint(:,j)};
        for k=1:numel(names)
            if ismember(k,[2 7])
                [value,index]=min(series{k});
                if value>=0, continue; end
                better=isnan(aggregate.joints(j).(names{k}).value) || ...
                    value<aggregate.joints(j).(names{k}).value;
            else
                [value,index]=max(series{k});
                if ismember(k,[1 6]) && value<=0, continue; end
                better=isnan(aggregate.joints(j).(names{k}).value) || ...
                    value>aggregate.joints(j).(names{k}).value;
            end
            if better
                e=emptyEvent;
                e.value=value; e.caseID=c.id; e.time_s=d.time(index);
                e.q=d.state.q(index,:);
                e.speed_rad_s=d.state.qd(index,j);
                e.torque_Nm=d.torque.total(index,j);
                e.acceleration_rad_s2=d.state.qdd(index,j);
                aggregate.joints(j).(names{k})=e;
            end
        end
    end
    s=d.summaryTable;
    row=table(repmat(c.id,dof,1),s.Joint, ...
        s.PeakAbsTorque_Nm,s.RMSTorqueOverTrajectory_Nm, ...
        s.MaxAbsSpeed_rad_s,s.PeakAbsPower_W, ...
        'VariableNames',{'CaseID','Joint','PeakAbsTorque_Nm', ...
        'RMSTorque_Nm','PeakAbsSpeed_rad_s','PeakAbsPower_W'});
    aggregate.caseComparison=[aggregate.caseComparison;row]; %#ok<AGROW>
end
aggregate.wording="Largest values observed in current validated test-case suite; not final actuator requirements.";
aggregate.missionRMSAvailable=false;
end
