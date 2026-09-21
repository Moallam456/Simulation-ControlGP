function suite = runActuatorSizingCases(robot,cases,scenario,options)
% RUNACTUATORSIZINGCASES Evaluate traceable load cases without hiding failures.
if nargin<4, options=struct(); end
if ~isfield(scenario,'gravity')
    error('sizing:MissingGravity','scenario.gravity is required.');
end
if ~isfield(options,'checkConvergence'), options.checkConvergence=true; end
if ~isfield(options,'convergenceTolerance'), options.convergenceTolerance=0.08; end
if ~isfield(options,'gravitySamples'), options.gravitySamples=200; end
if ~isfield(options,'gravityStarts'), options.gravityStarts=2; end
if ~isfield(options,'gravityMaxEvaluations'), options.gravityMaxEvaluations=300; end
if ~isfield(options,'seed'), options.seed=1; end
empty=struct('id',"",'category',"",'dataStatus',"",'source',"", ...
    'status',"",'reason',"", ...
    'trajectory',[],'trajectoryReport',[],'dynamics',[], ...
    'convergence',[]);
suite.cases=repmat(empty,numel(cases),1);
suite.gravity=[];
suite.robotID=string(robot.id);
suite.definitions=cases;
suite.scenario=scenario;
suite.options=options;
for i=1:numel(cases)
    c=cases(i);
    result=empty;
    result.id=string(c.id);
    result.category=string(c.category);
    result.dataStatus=string(c.dataStatus);
    result.source=string(c.source);
    if ~c.enabled
        result.status="DISABLED";
        result.reason=string(c.disabledReason);
    elseif result.category=="gravity"
        try
            gravityOptions=struct('gravity',scenario.gravity, ...
                'numSamples',options.gravitySamples,'numStarts',options.gravityStarts, ...
                'maxFunctionEvaluations',options.gravityMaxEvaluations, ...
                'seed',options.seed);
            suite.gravity=optimizeGravityLoading(robot,gravityOptions);
            result.status="PASS";
            result.reason="Numerical search only; not a certified global maximum.";
        catch ME
            result.status="FAIL";
            result.reason=string(ME.message);
        end
    else
        try
            [result.trajectory,result.trajectoryReport]=generateTrajectory(robot,c);
            if ~result.trajectoryReport.pass
                result.status="FAIL";
                result.reason=result.trajectoryReport.reason;
            else
                result.dynamics=analyzeTrajectoryDynamics(robot,result.trajectory,scenario);
                if options.checkConvergence
                    result.convergence=checkTrajectoryConvergence(robot,c,scenario, ...
                        result.dynamics,options.convergenceTolerance);
                    if ~result.convergence.pass
                        result.status="NOT_CONVERGED";
                        result.reason=result.convergence.reason;
                    else
                        result.status="PASS";
                    end
                else
                    result.status="UNVERIFIED";
                    result.reason="Time-resolution convergence was not run; excluded from sizing.";
                end
            end
        catch ME
            result.status="FAIL";
            result.reason=string(ME.message);
        end
    end
    suite.cases(i)=result;
    fprintf('%-24s %-14s %s\n',result.id,result.status,result.reason);
end
suite.aggregate=aggregateActuatorSizingResults(robot,suite.cases,suite.gravity);
end
