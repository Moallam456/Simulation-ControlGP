function suite = example_actuator_sizing_cases(showPlots)
% EXAMPLE_ACTUATOR_SIZING_CASES Current preliminary observed load cases.
if nargin<1, showPlots=true; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'kinematics'),fullfile(root,'trajectory'), ...
    fullfile(root,'dynamics_analysis'),fullfile(root,'actuator_sizing_cases'));
robot=loadRobot();
validation=validateDynamicModel(robot);
if ~validation.pass
    error('sizing:InvalidModel','Dynamic model validation failed.');
end
cases=createActuatorSizingCases(robot);
scenario.gravity=[0 0 -9.81];
suite=runActuatorSizingCases(robot,cases,scenario);
disp(suite.aggregate.wording);
disp(suite.aggregate.caseComparison);
if ~isempty(suite.gravity)
    disp(suite.gravity.summaryTable);
end
for j=1:numel(suite.aggregate.joints)
    event=suite.aggregate.joints(j).peakAbsoluteTorque;
    fprintf('%s observed peak |torque|: %.3f N m, case %s at %.3f s, speed %.3f rad/s\n', ...
        suite.aggregate.joints(j).joint,event.value,event.caseID, ...
        event.time_s,event.speed_rad_s);
end
if showPlots
    plotActuatorSizingEnvelope(suite);
    first=find(arrayfun(@(c)c.status=="PASS" && ...
        ~isempty(c.dynamics),suite.cases),1);
    if ~isempty(first)
        result=suite.cases(first).dynamics;
        figure('Name','Example per-case torque');
        plot(result.time,result.torque.total); grid on;
        xlabel('Time (s)'); ylabel('Joint torque (N m)');
        legend(result.summaryTable.Joint,'Location','best');
    end
end
end
