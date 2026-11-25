function summary = verifyPathProgressEndpointSettling(gate)
%VERIFYPATHPROGRESSENDPOINTSETTLING Audit the shared drive stop gate on all paths.
% Optional gate fields: positionTol [rad], velocityTol [rad/s], dwell and
% timeout [s], gateStep [s]. Omitted fields retain the simulation defaults.
if nargin<1, gate = struct(); end
root = fileparts(fileparts(mfilename('fullpath')));
results = fullfile(root,'results');
output = fullfile(results,'endpoint_settling');
if ~isfolder(output), mkdir(output); end
previousPath = path;
cleanup = onCleanup(@() path(previousPath));
addpath(fullfile(root,'helpers'),fullfile(root,'tests'));
saved = load(fullfile(results,'velocity_execution','execution.mat'), ...
    'data','reference','frames','limits','offset');
prior = load(fullfile(results,'waypoint_and_drive_response','drive_response.mat'), ...
    'nominal','configs','summary','stops');
cfg = prior.nominal;
gateNames = ["positionTol","velocityTol","dwell","timeout","gateStep"];
for field = string(fieldnames(gate)).'
    assert(any(field==gateNames),'Unknown endpoint gate field %s.',field);
    cfg.(field) = gate.(field);
end

names = ["Line","Exact-length arc","Exact-length circle","Multi S-curve"];
% Columns: period [s], gain [1/s], lag [s], delay [s], use saved offset.
presets = [.05 8 0 0 0; .01 2 .01 .01 0; .05 8 .05 .01 1];
rows = {};
traces = struct();
events = struct();
seen = [false,false,false]; % aligned, partial interval, pre-boundary command in flight
for k = 1:3
    ref = saved.reference{k};
    assert(numel(ref.edges)==2,'A single-segment path must have only one stop boundary.');
    for j = 1:size(presets,1)
        p = presets(j,:);
        c = cfg;
        c.period = p(1); c.gain = p(2); c.tau(:) = p(3); c.delay = p(4);
        initial = "Matched";
        if p(5), initial = "Offset"; end
        for enabled = [false true]
            c.stops = enabled;
            id = string(sprintf('path%d_preset%d_stop%d',k,j,enabled));
            e = execute(saved,k,c,initial);
            seen = seen | auditSchedule(e,ref,c);
            auditGate(e,ref,c);
            rows{end+1} = measure(id,names(k),"Comparison",initial,e,ref,c,saved.frames); %#ok<AGROW>
            if k==3 && ~enabled
                verifyPeriodicCircle(prior,rows{end});
            end
            if j>=2
                events.(id) = e;
            end
            if j==3
                traces.(id) = stopTrace(e,ref,c);
            end
        end
    end
    fprintf('%s: periodic and endpoint-gated runs measured.\n',names(k));
end
assert(all(seen),'The sweep did not cover an aligned endpoint, partial interval, and in-flight endpoint command.');

% Unreachable in the allotted time, with a positive dwell shorter than timeout.
c = cfg;
c.period = .05; c.gain = 8; c.tau(:) = .05; c.delay = .01; c.stops = true;
c.positionTol = 1e-12; c.velocityTol = 1e-10; c.timeout = .08; c.dwell = .05;
e = execute(saved,1,c,"Offset");
auditSchedule(e,saved.reference{1},c);
auditGate(e,saved.reference{1},c);
assert(e.status=="Timeout" && isnan(e.waypoints(1).release), ...
    'The deliberately failing endpoint incorrectly reported completion.');
rows{end+1} = measure("forced_timeout",names(1),"Negative","Offset",e,saved.reference{1},c,saved.frames);
traces.forced_timeout = stopTrace(e,saved.reference{1},c);
events.forced_timeout = e;

for id = ["run071","run076","run070"]
    spec = prior.configs([prior.configs.ID]==id);
    assert(isscalar(spec),'Missing saved multi-segment regression configuration.');
    c = spec.Cfg;
    e = execute(saved,4,c,spec.Initial);
    auditGate(e,saved.reference{4},c);
    verifyMulti(prior,id,e);
    rows{end+1} = measure("regression_"+id,names(4),"Regression",spec.Initial,e, ...
        saved.reference{4},c,saved.frames); %#ok<AGROW>
end
fprintf('Three multi-segment replays matched the saved results.\n');
summary = struct2table([rows{:}]);
writetable(summary,fullfile(output,'summary.csv'));
save(fullfile(output,'endpoint_settling.mat'),'summary','events','traces','presets','cfg','seen');
pathProgressPlotEndpointSettling(output,summary,traces);
disp(summary(:,{'Case','Group','Stops','Outcome','BoundaryJointError_rad', ...
    'BoundaryActualSpeed_rad_s','EndpointSettling_s','EndJointError_rad','EndActualSpeed_rad_s'}));
end

function e = execute(saved,k,c,initial)
ref = saved.reference{k};
q0 = pathProgressEvaluateReference(ref,0);
if initial=="Offset", q0 = q0+saved.offset; end
assert(all(q0>=saved.limits(:,1) & q0<=saved.limits(:,2)),'The start is outside provisional limits.');
e = pathProgressRunDrive(ref,q0,c,saved.limits,saved.data{k}.rateLimits.velocityURDF);
end

function coverage = auditSchedule(e,ref,c)
assert(all(diff(e.time)>0) && all(diff(e.sent)>0) && isequal(e.arrivals,e.sent+c.delay), ...
    'The event trace is duplicated, out of order, or bypassed the delay queue.');
boundary = e.waypoints(end).boundary;
tol = 64*eps(max(1,e.time(end)));
assert(nnz(abs(e.sent-boundary)<=tol)==1,'Endpoint must send exactly one boundary command.');
event = find(abs(e.time-boundary)<=tol,1);
command = find(abs(e.sent-boundary)<=tol,1);
target = pathProgressEvaluateReference(ref,ref.edges(end));
assert(max(abs(e.commands(:,command)-c.gain*(target-e.q(:,event))))<=1e-12 && ...
    all(e.refTime(event:end)==ref.edges(end)), ...
    'The endpoint did not command zero feedforward and freeze its target.');
[q,v] = pathProgressDriveStep(e.q(:,1:end-1),e.v(:,1:end-1), ...
    e.u(:,1:end-1),c.tau,diff(e.time));
assert(max(abs(q-e.q(:,2:end)),[],'all')<=1e-12 && ...
    max(abs(v-e.vBefore(:,2:end)),[],'all')<=1e-12, ...
    'The simulated state changed outside the closed-form drive update.');
assert(all(abs(e.sent-round(e.sent/c.period)*c.period)<=tol | abs(e.sent-boundary)<=tol), ...
    'The periodic wall clock changed after the endpoint.');
ticks = (0:floor((e.time(end)+tol)/c.period))*c.period;
assert(all(arrayfun(@(t) nnz(abs(e.sent-t)<=tol)==1,ticks)), ...
    'A periodic command was missed or duplicated.');
earlier = e.sent(e.sent<boundary-tol);
partial = boundary-earlier(end)<c.period-tol;
inFlight = any(e.sent<boundary-tol & e.arrivals>boundary+tol & e.arrivals<=e.time(end)+tol);
for j = find(e.arrivals<=e.time(end)+tol & e.arrivals>=boundary-tol)
    i = find(abs(e.time-e.arrivals(j))<=tol,1);
    assert(~isempty(i) && max(abs(e.u(:,i)-e.commands(:,j)))<=1e-12, ...
        'A command scheduled to arrive during settling was lost.');
end
if all(c.tau>0)
    assert(isequal(e.v,e.vBefore),'A positive-lag actual velocity jumped.');
end
coverage = [e.waypoints(end).periodic,partial,inFlight];
end

function auditGate(e,ref,c)
if ~c.stops
    assert(e.status=="Complete" && e.time(end)==e.waypoints(end).boundary && ...
        e.waypoints(end).status=="Periodic completion",'Periodic completion changed.');
    return;
end
for w = e.waypoints
    if w.status=="Not reached", continue; end
    if w.status=="Timeout"
        assert(e.status=="Timeout" && isnan(w.release) && ...
            abs(w.added-c.timeout)<=1e-11 && all(isnan([e.waypoints(w.segment+1:end).boundary])), ...
            'Timeout released a waypoint or continued the trajectory.');
        continue;
    end
    assert(w.status=="Released" && w.added>=c.dwell-1e-12 && w.added<=c.timeout+1e-12, ...
        'A waypoint released outside the dwell or timeout.');
    start = w.release-c.dwell;
    % Subtraction can place the closed interval's start one ULP before an
    % exactly coincident tick. Use its right limit at that same clock event.
    [gap,j] = min(abs(e.time-start));
    if gap<=64*eps(max(1,start)), start = e.time(j); end
    times = unique([start:.00025:w.release,w.release,e.time(e.time>start & e.time<w.release)]);
    [q,v,left,s] = pathProgressObserveDrive(e,c.tau,times);
    endpoint = ref.edges(w.segment+1);
    target = pathProgressEvaluateReference(ref,endpoint);
    assert(max(abs(q-target),[],'all')<=c.positionTol+1e-12 && ...
        max(abs([v,left(:,2:end)]),[],'all')<=c.velocityTol+1e-12 && ...
        max(abs(s-endpoint))<=1e-12,'The final dwell was not at rest within both tolerances.');
    % Reconstruct the gate's continuous interval bounds on a grid independent
    % of its 1 ms monitor. All drive-input events are included.
    [~,index] = histc(times,e.time);
    for i = 1:numel(times)-1
        [~,~,lo,hi] = pathProgressDriveStep(q(:,i),v(:,i),e.u(:,index(i)), ...
            c.tau,times(i+1)-times(i));
        assert(max(abs([lo-target,hi-target]),[],'all')<=c.positionTol+1e-12, ...
            'An interval between dwell observations exceeded the position tolerance.');
    end
end
if e.status=="Complete"
    assert(all([e.waypoints.status]=="Released") && e.refTime(end)==ref.edges(end), ...
        'Verified COMPLETE was reported without a terminal gate release.');
else
    assert(any([e.waypoints.status]=="Timeout"),'Unexpected incomplete outcome.');
end
end

function r = measure(id,name,group,initial,e,ref,c,frames)
last = e.waypoints(end);
activeGate = e.waypoints(find(~isnan([e.waypoints.boundary]),1,'last'));
target = pathProgressEvaluateReference(ref,ref.edges(end));
[p,R] = pathProgressBatchTcp(frames,e.q(:,end),e.v(:,end),e.v(:,end));
[pd,Rd] = pathProgressBatchTcp(frames,target,zeros(size(target)),zeros(size(target)));
rel = Rd.'*R;
skew = [rel(3,2)-rel(2,3);rel(1,3)-rel(3,1);rel(2,1)-rel(1,2)];
r = struct('Scenario',id,'Case',name,'Group',group,'Initial',initial,'Stops',c.stops, ...
    'Period_ms',1000*c.period,'Gain_per_s',c.gain,'Tau_ms',1000*max(c.tau),'Delay_ms',1000*c.delay, ...
    'PositionTolerance_rad',c.positionTol,'VelocityTolerance_rad_s',c.velocityTol, ...
    'Dwell_s',c.dwell,'Timeout_s',c.timeout,'GateCheck_ms',1000*c.gateStep, ...
    'Outcome',e.status,'Planned_s',ref.edges(end),'WallTime_s',e.time(end), ...
    'ReferenceReached_s',e.refTime(end),'ReachedPlannedEndpoint',~isnan(last.boundary), ...
    'EndpointWallTime_s',last.boundary,'BoundaryAligned',last.periodic, ...
    'BoundaryJointError_rad',last.positionError,'BoundaryActualSpeed_rad_s',last.speedBefore, ...
    'BoundaryAfterActualSpeed_rad_s',last.speedAfter, ...
    'EndpointSettling_s',last.added,'TotalSettling_s',sum([e.waypoints.added]), ...
    'GateEndJointError_rad',activeGate.releasePositionError, ...
    'GateEndActualSpeed_rad_s',activeGate.releaseSpeed, ...
    'EndJointError_rad',max(abs(e.q(:,end)-target)), ...
    'EndActualSpeed_rad_s',max(abs(e.v(:,end))), ...
    'EndTcpError_m',norm(p-pd),'EndTcpRotation_rad',atan2(norm(skew),trace(rel)-1), ...
    'PendingCommands',e.pending,'PositionViolation',e.positionViolation, ...
    'CommandVelocityViolation',e.commandVelocityViolation,'ActualVelocityViolation',e.actualVelocityViolation);
end

function verifyPeriodicCircle(prior,r)
base = prior.summary(prior.summary.Case=="Exact-length circle" & ...
    prior.summary.Initial==r.Initial & prior.summary.Period_ms==r.Period_ms & ...
    prior.summary.Gain_per_s==r.Gain_per_s & prior.summary.Tau_ms==r.Tau_ms & ...
    prior.summary.Delay_ms==r.Delay_ms,:);
assert(height(base)==1 && base.Outcome==r.Outcome && ...
    max(abs([base.Executed_s-r.WallTime_s,base.EndJointError_rad-r.EndJointError_rad, ...
    base.EndActualSpeed_rad_s-r.EndActualSpeed_rad_s,base.EndTcpError_m-r.EndTcpError_m]))<=1e-11, ...
    'The previous periodic circle behavior changed.');
end

function verifyMulti(prior,id,e)
base = prior.summary(prior.summary.Scenario==id,:);
assert(height(base)==1 && base.Outcome==e.status && ...
    max(abs([base.Executed_s-e.time(end),base.ReferenceReached_s-e.refTime(end), ...
    base.Settling_s-sum([e.waypoints.added])]))<=1e-11,'Multi-segment wall time or status changed.');
reached = e.waypoints(~isnan([e.waypoints.boundary]));
w = prior.stops(prior.stops.Scenario==id,:);
assert(isequal(w.Outcome,[reached.status].') && isequal(w.PeriodicTickAtBoundary,[reached.periodic].') && ...
    max(abs(w.WallBoundary_s-[reached.boundary].'))<=1e-11 && ...
    max(abs(w.BoundaryJointError_rad-[reached.positionError].'))<=1e-11 && ...
    max(abs(w.GateEndActualSpeed_rad_s-[reached.releaseSpeed].'))<=1e-11 && ...
    max(abs(w.GateEndJointError_rad-[reached.releasePositionError].'))<=1e-11, ...
    'A multi-segment waypoint stop changed.');
end

function values = stopTrace(e,ref,c)
boundary = e.waypoints(end).boundary;
time = unique([max(0,boundary-.15):.001:e.time(end),e.time(e.time>=boundary-.15),e.time(end)]);
[q,v,~,s] = pathProgressObserveDrive(e,c.tau,time);
goal = pathProgressEvaluateReference(ref,ref.edges(end));
values = struct('TimeFromEndpoint_s',time-boundary,'ReferenceTime_s',s, ...
    'PositionError_rad',max(abs(q-goal),[],1),'ActualSpeed_rad_s',max(abs(v),[],1));
end
