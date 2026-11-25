function summary = verifyPathProgressWaypointDrive()
%VERIFYPATHPROGRESSWAYPOINTDRIVE Unit checks and a bounded lag/delay experiment.
root = fileparts(fileparts(mfilename('fullpath')));
results = fullfile(root,'results');
output = fullfile(results,'waypoint_and_drive_response');
if ~isfolder(output), mkdir(output); end
previousPath = path;
cleanup = onCleanup(@() path(previousPath));
addpath(fullfile(root,'helpers'),fullfile(root,'tests'),fullfile(fileparts(root),'TimeScaling'));
verifyCore();
fprintf('Analytic drive, delayed arrivals, ideal equivalence, gate and timeout checks passed.\n');

saved = load(fullfile(results,'velocity_execution','execution.mat'), ...
    'data','reference','frames','limits','offset');
data = saved.data(3:4);
reference = saved.reference(3:4);
frames = saved.frames;
grid = .001;
nominal = struct('stops',true,'period',.05,'gain',8,'tau',zeros(6,1),'delay',0, ...
    'positionTol',.00035,'velocityTol',.003,'dwell',.05,'timeout',1.25,'gateStep',.001);
configs = makeScenarios(nominal);
inputs = struct2table(configs);
inputs.Cfg = [];
inputs.PositionTolerance_rad = arrayfun(@(x) x.Cfg.positionTol,configs).';
inputs.ActualVelocityTolerance_rad_s = arrayfun(@(x) x.Cfg.velocityTol,configs).';
inputs.Dwell_s = arrayfun(@(x) x.Cfg.dwell,configs).';
inputs.Timeout_s = arrayfun(@(x) x.Cfg.timeout,configs).';
inputs.GateCheck_ms = 1000*arrayfun(@(x) x.Cfg.gateStep,configs).';
writetable(inputs,fullfile(output,'configurations.csv'));
rows = cell(1,numel(configs));
stops = table();
traces = struct();
for i = 1:numel(configs)
    spec = configs(i);
    k = spec.ReferenceIndex;
    r = reference{k};
    q0 = pathProgressEvaluateReference(r,0);
    if spec.Initial=="Offset", q0 = q0+saved.offset; end
    assert(all(q0>=saved.limits(:,1) & q0<=saved.limits(:,2)), ...
        'The simulation start is outside the joint limits.');
    run = pathProgressRunDrive(r,q0,spec.Cfg,saved.limits,data{k}.rateLimits.velocityURDF);
    assert(all(diff(run.time)>0) && all(diff(run.refTime)>=-1e-12) && ...
        all(diff(run.sent)>0) && all(diff(run.arrivals)>0), ...
        'Event order or reference clock is invalid.');
    if spec.Group=="Negative"
        assert(run.status=="Timeout" && run.waypoints(1).status=="Timeout" && ...
            all(isnan([run.waypoints(2:end).boundary])),'A negative test released the next segment.');
    elseif run.status=="Complete"
        assert(abs(run.time(end)-r.edges(end)-sum([run.waypoints.added]))<1e-10 && ...
            run.refTime(end)==r.edges(end),'Completion wall time or reference time is invalid.');
    end
    if spec.Cfg.stops
        released = run.waypoints([run.waypoints.status]=="Released");
        assert(all([released.releasePositionError]<=spec.Cfg.positionTol) && ...
            all([released.releaseSpeed]<=spec.Cfg.velocityTol), ...
            'A waypoint released outside its configured gate.');
    end
    if keepTrace(spec)
        [rows{i},waypoints,trace] = pathProgressDriveMetrics(spec,r,run,spec.Cfg,frames,grid);
        traces.(spec.ID) = trace;
    else
        [rows{i},waypoints] = pathProgressDriveMetrics(spec,r,run,spec.Cfg,frames,grid);
    end
    stops = [stops;waypoints]; %#ok<AGROW>
    if mod(i,12)==0 || i==numel(configs), fprintf('%d/%d robot scenarios measured.\n',i,numel(configs)); end
end
summary = struct2table([rows{:}]);
summary.Group = inputs.Group;
writetable(summary,fullfile(output,'summary.csv'));
writetable(stops,fullfile(output,'waypoints.csv'));
normal = find(summary.Group~="Negative");
[~,pos] = max(summary.PeakTcp_m(normal));
[~,speed] = max(summary.PeakSpeedError_m_s(normal));
multi = find(summary.Group=="Matrix" & summary.Case=="Multi S-curve");
[~,multiPeak] = max(summary.PeakTcp_m(multi));
convergence = summary([],:);
for i = unique([normal(pos),normal(speed),multi(multiPeak)])
    spec = configs(i);
    k = spec.ReferenceIndex;
    r = reference{k};
    q0 = pathProgressEvaluateReference(r,0);
    if spec.Initial=="Offset", q0 = q0+saved.offset; end
    run = pathProgressRunDrive(r,q0,spec.Cfg,saved.limits,data{k}.rateLimits.velocityURDF);
    convergence = [convergence;summary(i,:)]; %#ok<AGROW>
    for fine = [grid/2,grid/4]
        row = pathProgressDriveMetrics(spec,r,run,spec.Cfg,frames,fine);
        next = struct2table(row);
        next.Group = spec.Group;
        convergence = [convergence;next]; %#ok<AGROW>
    end
end
writetable(convergence,fullfile(output,'grid_convergence.csv'));
save(fullfile(output,'drive_response.mat'),'summary','stops','inputs','configs', ...
    'convergence','traces','reference','frames','grid','nominal');
pathProgressPlotWaypointDrive(output,summary,traces);
pathProgressWriteDriveReport(output,summary,stops,convergence,nominal);
fprintf('Finished %d configured scenarios. Evidence saved.\n',height(summary));
end

function scenarios = makeScenarios(default)
scenarios = struct([]);
n = 0;
for k = 1:2
    for p = [.002,.01,.05]
        for g = [2,8]
            for tau = [0,.01,.05]
                for delay = [0,.01]
                    n = n+1;
                    s = specification(k,default,p,g,tau,delay,k==2,"Matched","Matrix",n);
                    if n==1, scenarios = s; else, scenarios(n) = s; end
                end
            end
        end
    end
    for p = [.01,.05]
        for g = [2,8]
            n = n+1;
            scenarios(n) = specification(k,default,p,g,.05,.01,k==2,"Offset","Offset subset",n);
        end
    end
end
for p = [.002,.05]
    for initial = ["Matched","Offset"]
        for enabled = [false,true]
            n = n+1;
            scenarios(n) = specification(2,default,p,8,0,0,enabled,initial,"Stop comparison",n);
        end
    end
end
n = n+1;
scenarios(n) = specification(2,default,.05,0,0,0,true,"Offset","Negative",n);
n = n+1;
tight = default;
tight.positionTol = 1e-12;
tight.velocityTol = 1e-10;
tight.timeout = .08;
scenarios(n) = specification(2,tight,.05,8,.05,.01,true,"Offset","Negative",n);
end

function s = specification(k,cfg,p,g,tau,delay,enabled,initial,group,n)
names = ["Exact-length circle","Multi S-curve"];
cfg.period = p; cfg.gain = g; cfg.tau(:) = tau; cfg.delay = delay; cfg.stops = enabled;
s = struct('ID',string(sprintf('run%03d',n)),'Case',names(k), ...
    'ReferenceIndex',k,'Group',group,'Initial',initial,'Stops',enabled, ...
    'Period_ms',1000*p,'Gain_per_s',g,'Tau_ms',1000*tau,'Delay_ms',1000*delay,'Cfg',cfg);
end

function keep = keepTrace(s)
keep = (s.Group=="Stop comparison" && s.Initial=="Offset" && s.Period_ms==50) || ...
    (s.Group=="Matrix" && s.ReferenceIndex==1 && s.Period_ms==50 && s.Gain_per_s==8 && ...
    (s.Tau_ms==50 || (s.Tau_ms==0 && s.Delay_ms==0)));
end

function verifyCore()
[q,v,lo,hi] = pathProgressDriveStep(0,0,2,.1,.2);
assert(abs(v-2*(1-exp(-2)))<1e-14 && abs(q-2*(.2-.1*(1-exp(-2))))<1e-14 && ...
    lo==0 && hi==q);
[q,v] = pathProgressDriveStep(0,0,1,1,1e-9);
assert(abs(q-0.5e-18)<1e-26 && abs(v-1e-9)<1e-17,'Small drive interval lost precision.');
[~,~,lo,hi] = pathProgressDriveStep(0,1,-1,.1,.2);
assert(hi>.030 && lo<0,'Internal position turning point was omitted.');

r = unitReference([0,1,1.2]);
cfg = struct('stops',false,'period',.073,'gain',2,'tau',0,'delay',0, ...
    'positionTol',.001,'velocityTol',.001,'dwell',.05,'timeout',1,'gateStep',.001);
ideal = pathProgressRunDrive(r,0,cfg,[-10 10],10);
[q,qd] = pathProgressEvaluateReference(r,ideal.sent);
[expected,commands] = pathProgressExecuteZoh(ideal.sent,q,qd,0,cfg.gain);
[observed,actual] = pathProgressObserveDrive(ideal,0,ideal.sent);
assert(max(abs(expected-observed))<1e-12 && max(abs(commands-actual))<1e-12 && ...
    ideal.sent(end)==r.edges(end) && diff(ideal.sent(end-1:end))<cfg.period, ...
    'Zero lag/delay or the terminal partial interval differs from the ideal executor.');

hold = unitReference([1,1]);
cfg.period = .1; cfg.gain = 1; cfg.delay = .035;
delayed = pathProgressRunDrive(hold,0,cfg,[-10 10],10);
[q,v,left] = pathProgressObserveDrive(delayed,0,[.034,.035,.1,.135]);
assert(max(abs(q-[0,0,.065,.1]))<1e-12 && ...
    max(abs(v-[0,1,1,.935]))<1e-12 && left(2)==0 && left(4)==1, ...
    'The first or fractional-tick command arrival was handled incorrectly.');
cfg.delay = .1;
coincident = pathProgressRunDrive(hold,0,cfg,[-10 10],10);
assert(nnz(coincident.time==.1)==1 && coincident.v(coincident.time==.1)==1 && ...
    coincident.commands(2)==1,'A coincident arrival and tick were not ordered deterministically.');
cfg.tau = .05;
lagged = pathProgressRunDrive(hold,0,cfg,[-10 10],10);
assert(max(abs(lagged.v-lagged.vBefore))==0 && lagged.q(lagged.time==.1)==0, ...
    'Positive-lag velocity or position jumped at a command arrival.');

cfg.stops = true; cfg.gain = 0; cfg.delay = 0;
cfg.positionTol = 10; cfg.velocityTol = 1e-12; cfg.timeout = .06;
failure = pathProgressRunDrive(r,0,cfg,[-10 10],10);
w = failure.waypoints(1);
j = find(failure.sent==w.boundary,1);
assert(failure.status=="Timeout" && failure.commands(j)==0 && ...
    w.speedAfter>cfg.velocityTol && isnan(failure.waypoints(2).boundary), ...
    'The gate accepted commanded zero velocity or continued after timeout.');

% Put a periodic transmit just before the boundary, with an arrival after
% it. Both that command and the extra boundary command must survive.
b = r.edges(2);
cfg.period = b-.01; cfg.delay = .035; cfg.gain = 2;
queued = pathProgressRunDrive(r,0,cfg,[-10 10],10);
assert(~queued.waypoints(1).periodic && abs(queued.sent(2)-(b-.01))<1e-12 && ...
    abs(queued.sent(3)-b)<1e-12 && max(abs(queued.arrivals-queued.sent-.035))<1e-12);
i = find(abs(queued.time-queued.arrivals(2))<1e-12,1);
j = find(abs(queued.time-queued.arrivals(3))<1e-12,1);
assert(~isempty(i) && ~isempty(j) && queued.u(i)==queued.commands(2) && queued.u(j)==queued.commands(3) && ...
    queued.v(i)==queued.vBefore(i) && queued.v(j)==queued.vBefore(j), ...
    'A boundary discarded an in-flight command or bypassed drive lag.');
cfg.period = b; cfg.delay = 0;
aligned = pathProgressRunDrive(r,0,cfg,[-10 10],10);
assert(aligned.waypoints(1).periodic && nnz(abs(aligned.sent-b)<1e-12)==1, ...
    'A coincident boundary and periodic tick sent duplicate commands.');
end

function reference = unitReference(points)
data.segments = struct([]);
data.fit = struct([]);
[~,~,~,~,~,profile] = sCurveTimeScaling(1.7,4,15,.05);
for k = 1:numel(points)-1
    data.segments(k).profile = profile;
    data.fit(k).nodes = [0 1];
    data.fit(k).coefficients = reshape([points(k),points(k+1)-points(k),0,0],1,1,4);
end
reference = pathProgressPrepareReference(data);
end
