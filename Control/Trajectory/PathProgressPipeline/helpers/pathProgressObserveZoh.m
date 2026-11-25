function [q,v,vLeft] = pathProgressObserveZoh(ticks,state,command,time)
%PATHPROGRESSOBSERVEZOH Observe an exact held-command trace at sorted times.
% q [rad] and v [rad/s] are joints-by-query. At a tick v is right-continuous
% and vLeft is the command that was active just before the tick.
if isempty(time) || any(~isfinite(time)) || time(1)<ticks(1) || ...
        time(end)>ticks(end) || any(diff(time)<0)
    error('PathProgress:InvalidObservationTime','Observation times must be sorted inside the trace.');
end
q = zeros(size(state,1),numel(time));
v = q;
vLeft = q;
k = 1;
for i = 1:numel(time)
    while k<numel(ticks) && ticks(k+1)<=time(i)
        k = k+1;
    end
    q(:,i) = state(:,k)+command(:,k)*(time(i)-ticks(k));
    v(:,i) = command(:,k);
    if k>1 && time(i)==ticks(k)
        vLeft(:,i) = command(:,k-1);
    else
        vLeft(:,i) = v(:,i);
    end
end
end
