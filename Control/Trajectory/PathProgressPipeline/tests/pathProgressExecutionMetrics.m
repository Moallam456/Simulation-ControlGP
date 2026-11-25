function [row,stops,trace] = pathProgressExecutionMetrics(data,reference,frames, ...
    desired,period,gain,initial,observation,grid,positionLimits,velocityLimits)
%PATHPROGRESSEXECUTIONMETRICS Compare equal-time held motion with reference FK.
duration = reference.edges(end);
finish = duration+observation;
assert(finish>duration,'The observation interval is below the clock resolution.');
ticks = (0:ceil(duration/period))*period;
ticks = ticks(ticks<duration-8*eps(duration));
post = duration+(1:ceil(observation/period))*period;
ticks = [ticks,duration,post(post<finish-8*eps(finish))];
% The final observation is not a command event. Append an integration
% boundary only; its newly calculated command is never applied or counted.
[target,velocity] = pathProgressEvaluateReference(reference,min([ticks,finish],duration));
[state,command] = pathProgressExecuteZoh([ticks,finish],target,velocity, ...
    target(:,1)+initial,gain*ones(size(initial)));
command(:,end) = command(:,end-1);
terminal = find(ticks==duration,1);
postIntervals = diff([ticks(terminal:end),finish]);
completionError = target(:,terminal)-state(:,terminal);
observedError = target(:,end)-state(:,end);
assert(max(abs(observedError-completionError*prod(1-gain*postIntervals)))<=1e-12, ...
    'The observed post-trajectory state disagrees with held-command integration.');
times = unique([0:grid:finish,ticks,finish,reference.edges,desired.phaseTimes]);
trajectoryTime = min(times,duration);
[q,v,vLeft] = pathProgressObserveZoh([ticks,finish],state,command,times);
[qr,vr] = pathProgressEvaluateReference(reference,trajectoryTime);
[p,R,speed,leftSpeed] = pathProgressBatchTcp(frames,q,v,vLeft);
[pr,Rr,desiredSpeed] = pathProgressBatchTcp(frames,qr,vr,vr);
rel = pagemtimes(Rr,'transpose',R,'none');
skew = [reshape(rel(3,2,:)-rel(2,3,:),1,[]); ...
    reshape(rel(1,3,:)-rel(3,1,:),1,[]);reshape(rel(2,1,:)-rel(1,2,:),1,[])];
rotation = atan2(vecnorm(skew,2,1),reshape(rel(1,1,:)+rel(2,2,:)+rel(3,3,:),1,[])-1);
joint = q-qr;
position = vecnorm(p-pr,2,1);
crossTrack = distanceToPath(data,p);
atEnd = find(times==duration,1);
motion = 1:atEnd;
speedError = abs(speed-desiredSpeed);
leftError = abs(leftSpeed-desiredSpeed);
% At a command discontinuity use both sides for the motion peak and
% integrate the squared speed error using each interval's interior limits.
dt = diff(times(motion));
rmsSpeed = sqrt(sum(dt.*(speedError(1:atEnd-1).^2+leftError(2:atEnd).^2)/2)/duration);
peakSpeed = max([speedError(1:atEnd-1),leftError(2:atEnd)]);
active = command(:,1:end-1);
badPosition = any(state<positionLimits(:,1) | state>positionLimits(:,2),1);
badVelocity = any(abs(active)>velocityLimits,1);
mode = "Feedforward";
if gain>0, mode = "Position feedback"; end
condition = "Matched";
if any(initial), condition = "Offset"; end
row = struct('Case',data.demo,'Period_ms',round(period*1000),'Mode',mode,'Kp_per_s',gain, ...
    'Initial',condition,'GainTimesPeriod',gain*period,'ObservationGrid_ms',grid*1000, ...
    'PeakJoint_rad',max(abs(joint(:,motion)),[],'all'), ...
    'RmsJoint_rad',timeRms(mean(joint(:,motion).^2,1),dt,duration), ...
    'PeakTcp_m',max(position(motion)),'RmsTcp_m',timeRms(position(motion).^2,dt,duration), ...
    'PeakRotation_rad',max(rotation(motion)),'RmsRotation_rad',timeRms(rotation(motion).^2,dt,duration), ...
    'PeakSpeedError_m_s',peakSpeed,'RmsSpeedError_m_s',rmsSpeed, ...
    'PeakCrossPath_m',max(crossTrack(motion)),'RmsCrossPath_m',timeRms(crossTrack(motion).^2,dt,duration), ...
    'CompletionJoint_rad',max(abs(joint(:,atEnd))),'CompletionTcp_m',position(atEnd), ...
    'CompletionRotation_rad',rotation(atEnd),'SettledJoint_rad',max(abs(joint(:,end))), ...
    'SettledTcp_m',position(end),'SettledRotation_rad',rotation(end), ...
    'MaxCommand_rad_s',max(abs(active),[],'all'), ...
    'MaxCommandIncrement_rad_s',max(abs(diff(active,1,2)),[],'all'), ...
    'PositionViolationEvents',nnz(badPosition),'VelocityViolationTicks',nnz(badVelocity), ...
    'ProvisionalVelocityKnown',all(isfinite(velocityLimits)), ...
    'TerminalInterval_ms',1000*(duration-ticks(find(ticks<duration,1,'last'))));

indices = find(ismember(times,reference.edges(2:end-1)));
stops = table();
for k = 1:numel(indices)
    i = indices(k);
    t = times(i);
    tickIndex = find(abs(ticks-t)<=8*eps(t),1);
    tick = ~isempty(tickIndex);
    before = vLeft(:,i);
    after = v(:,i);
    beforeSpeed = leftSpeed(i);
    afterSpeed = speed(i);
    if tick
        % Treat ticks coincident to clock roundoff as the same event for the
        % left/right report, while leaving the periodic schedule untouched.
        before = active(:,tickIndex-1);
        after = active(:,tickIndex);
        [~,~,afterSpeed,beforeSpeed] = pathProgressBatchTcp(frames,q(:,i),after,before);
    end
    waypoint = table(data.demo,round(period*1000),mode,gain,condition,string(char('A'+k)),t,tick, ...
        max(abs(joint(:,i))),position(i),rotation(i),max(abs(before)), ...
        max(abs(after)),beforeSpeed,afterSpeed, ...
        'VariableNames',{'Case','Period_ms','Mode','Kp_per_s','Initial','Waypoint', ...
        'Time_s','TickAtWaypoint','JointError_rad','TcpError_m','RotationError_rad', ...
        'JointSpeedBefore_rad_s','JointSpeedAfter_rad_s','TcpSpeedBefore_m_s','TcpSpeedAfter_m_s'});
    stops = [stops;waypoint]; %#ok<AGROW>
end
if nargout>2
    trace = struct('time',times,'ticks',ticks,'command',active,'q',q,'target',qr, ...
        'jointError',joint,'positionError',position,'rotationError',rotation, ...
        'desiredSpeed',desiredSpeed,'executedSpeed',speed,'duration',duration, ...
        'waypoints',reference.edges(2:end-1));
end
end

function value = timeRms(squared,dt,duration)
value = sqrt(sum(dt.*(squared(1:end-1)+squared(2:end))/2)/duration);
end

function distance = distanceToPath(data,p)
% Distance to the finite desired path in 3-D. For linear moves use the
% union of closed line segments. For circles project onto the circle plane,
% then use the nearest point on the signed arc or one of its endpoints.
distance = inf(1,size(p,2));
for segment = data.segments
    if isfield(segment,'exactCircle')
        c = segment.exactCircle;
        radial = p-c.center;
        theta = mod(sign(c.thetaTotal)*atan2(c.e2.'*radial,c.e1.'*radial),2*pi);
        inside = theta<=abs(c.thetaTotal);
        closest = pathProgressCircleGeometry(c,theta(inside)/abs(c.thetaTotal));
        distance(inside) = vecnorm(p(:,inside)-closest,2,1);
        endpoints = pathProgressCircleGeometry(c,[0 1]);
        distance(~inside) = min(vecnorm(p(:,~inside)-endpoints(:,1),2,1), ...
            vecnorm(p(:,~inside)-endpoints(:,2),2,1));
    else
        a = segment.rawGeometry(:,1);
        direction = segment.rawGeometry(:,end)-a;
        u = min(1,max(0,direction.'*(p-a)/dot(direction,direction)));
        distance = min(distance,vecnorm(p-(a+direction*u),2,1));
    end
end
end
