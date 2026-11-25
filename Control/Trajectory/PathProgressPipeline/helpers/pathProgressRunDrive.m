function run = pathProgressRunDrive(reference,q,cfg,positionLimits,velocityLimits)
%PATHPROGRESSRUNDRIVE Numeric event-driven delayed-drive and waypoint state machine.
% cfg fields: period, delay, dwell, timeout and gateStep [s], tau [s/joint],
% gain [1/s], positionTol [rad], velocityTol [rad/s], stops (logical).
% reference is packed by pathProgressPrepareReference. Trace events are
% right-continuous, with actual velocity immediately before each event saved.
n = numel(q);
tau = cfg.tau(:);
values = [cfg.period,cfg.delay,cfg.dwell,cfg.timeout,cfg.gateStep, ...
    cfg.gain,cfg.positionTol,cfg.velocityTol,tau.'];
assert(numel(tau)==n && all(isfinite(values)&values>=0) && cfg.period>0 && ...
    cfg.timeout>0 && cfg.gateStep>0,'Invalid drive or stop configuration.');
target = pathProgressEvaluateReference(reference,reference.edges(2:end));
nseg = size(target,2);
blank = struct('segment',0,'boundary',NaN,'periodic',false,'positionError',NaN, ...
    'speedBefore',NaN,'speedAfter',NaN,'maxDeviation',NaN,'release',NaN, ...
    'added',0,'releasePositionError',NaN,'releaseSpeed',NaN,'status',"Not reached");
waypoints = repmat(blank,1,nseg);
t = 0; lastRef = 0; movingStart = 0; segment = 1; tick = 0; check = inf;
v = zeros(n,1); u = v; head = 1; count = 0; good = NaN; boundary = NaN;
mode = 1; % 1 MOVING, 2 SETTLING, 3 COMPLETE, 4 TIMEOUT
violation = [false,false,false];
peakCommand = 0; peakActual = 0;
arrival = zeros(1,0); queued = zeros(n,0); sent = zeros(1,0);
run.time = zeros(1,0); run.refTime = zeros(1,0); run.mode = zeros(1,0);
run.q = zeros(n,0); run.v = zeros(n,0); run.u = zeros(n,0); run.vBefore = zeros(n,0);
while mode<3
    periodic = tick*cfg.period;
    nextArrival = inf;
    if head<=count, nextArrival = arrival(head); end
    segmentEnd = inf; deadline = inf; dwellEnd = inf;
    if mode==1
        segmentEnd = movingStart+(reference.edges(segment+1)-reference.edges(segment));
    else
        deadline = boundary+cfg.timeout;
        if ~isnan(good), dwellEnd = good+cfg.dwell; end
    end
    next = min([periodic,nextArrival,segmentEnd,deadline,check,dwellEnd]);
    tol = 32*eps(max(1,next));
    [q,v,lo,hi] = pathProgressDriveStep(q,v,u,tau,next-t);
    peakActual = max(peakActual,max(abs(v)));
    violation(1) = violation(1) || any(lo<positionLimits(:,1) | hi>positionLimits(:,2));
    oldV = v;
    t = next;
    if mode==1
        lastRef = min(reference.edges(segment+1),reference.edges(segment)+(t-movingStart));
    else
        goal = target(:,segment);
        waypointError = max(max(abs([lo-goal,hi-goal])));
        waypoints(segment).maxDeviation = max(waypoints(segment).maxDeviation,waypointError);
        if waypointError>cfg.positionTol || max(abs(v))>cfg.velocityTol
            good = NaN;
        end
    end

    % Already queued arrivals take effect before the controller samples q.
    while head<=count && arrival(head)<=t+tol
        u = queued(:,head);
        v(tau==0) = u(tau==0);
        head = head+1;
    end
    peakActual = max(peakActual,max(abs(v)));
    violation(2) = violation(2) || any(abs(v)>velocityLimits);
    onTick = periodic<=t+tol;
    if onTick, tick = tick+1; end
    atBoundary = segmentEnd<=t+tol;
    send = onTick;
    ended = 0;
    if atBoundary
        lastRef = reference.edges(segment+1);
        ended = segment;
        w = blank;
        w.segment = segment; w.boundary = t; w.periodic = onTick;
        w.positionError = max(abs(q-target(:,segment)));
        w.speedBefore = max(abs(oldV));
        w.maxDeviation = w.positionError;
        if cfg.stops
            mode = 2; boundary = t; check = t+cfg.gateStep; good = NaN;
            w.status = "Settling"; send = true;
        elseif segment==nseg
            mode = 3; w.status = "Periodic completion"; send = true;
        else
            movingStart = t;
            segment = segment+1;
            w.status = "Periodic passage";
        end
        waypoints(ended) = w;
    end
    if send
        if mode==2
            qd = target(:,segment);
            vd = zeros(n,1);
        else
            [qd,vd] = pathProgressEvaluateReference(reference,lastRef);
        end
        command = vd+cfg.gain*(qd-q);
        count = count+1;
        queued(:,count) = command;
        arrival(count) = t+cfg.delay;
        sent(count) = t;
        peakCommand = max(peakCommand,max(abs(command)));
        violation(3) = violation(3) || any(abs(command)>velocityLimits);
        if cfg.delay==0
            u = command;
            v(tau==0) = u(tau==0);
            head = head+1;
        end
    end
    peakActual = max(peakActual,max(abs(v)));
    violation(2) = violation(2) || any(abs(v)>velocityLimits);
    if ended>0, waypoints(ended).speedAfter = max(abs(v)); end
    if mode==2
        e = max(abs(q-target(:,segment)));
        actualSpeed = max(abs(v));
        if e<=cfg.positionTol && actualSpeed<=cfg.velocityTol
            if isnan(good), good = t; end
        else
            good = NaN;
        end
        if ~isnan(good) && t+tol>=good+cfg.dwell
            waypoints(segment).release = t;
            waypoints(segment).added = t-boundary;
            waypoints(segment).releasePositionError = e;
            waypoints(segment).releaseSpeed = actualSpeed;
            waypoints(segment).status = "Released";
            check = inf;
            good = NaN;
            if segment==nseg
                mode = 3;
            else
                mode = 1; movingStart = t; segment = segment+1;
            end
        elseif t+tol>=boundary+cfg.timeout
            waypoints(segment).added = t-boundary;
            waypoints(segment).releasePositionError = e;
            waypoints(segment).releaseSpeed = actualSpeed;
            waypoints(segment).status = "Timeout";
            mode = 4;
        elseif check<=t+tol
            check = t+cfg.gateStep;
        end
    end
    run.time(end+1) = t;
    run.refTime(end+1) = lastRef;
    run.mode(end+1) = mode;
    run.q(:,end+1) = q;
    run.v(:,end+1) = v;
    run.vBefore(:,end+1) = oldV;
    run.u(:,end+1) = u;
end
run.status = "Complete";
if mode==4, run.status = "Timeout"; end
run.waypoints = waypoints;
run.sent = sent; run.arrivals = arrival; run.commands = queued;
run.pending = count-head+1;
run.peakCommand = peakCommand; run.peakActual = peakActual;
run.positionViolation = violation(1);
run.actualVelocityViolation = violation(2); run.commandVelocityViolation = violation(3);
end
