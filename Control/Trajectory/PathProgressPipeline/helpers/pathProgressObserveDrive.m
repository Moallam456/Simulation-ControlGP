function [q,v,left,referenceTime,mode] = pathProgressObserveDrive(run,tau,time)
%PATHPROGRESSOBSERVEDRIVE Observe exact drive states between recorded events.
% Queries are sorted wall-clock seconds. Positions are continuous. left is
% the pre-event actual joint velocity, which differs only for zero lag.
assert(~isempty(time) && all(isfinite(time)) && all(diff(time)>=0) && ...
    time(1)>=run.time(1) && time(end)<=run.time(end),'Invalid drive observation times.');
index = ones(size(time));
k = 1;
for i = 1:numel(time)
    while k<numel(run.time) && run.time(k+1)<=time(i)
        k = k+1;
    end
    index(i) = k;
end
h = time-run.time(index);
mode = run.mode(index);
referenceTime = run.refTime(index)+h.*(mode==1);
% Keep roundoff at the final moving endpoint in the closed reference domain.
referenceTime = min(referenceTime,run.refTime(end));
[q,v] = pathProgressDriveStep(run.q(:,index),run.v(:,index),run.u(:,index),tau,h);
left = v;
atEvent = h==0;
left(:,atEvent) = run.vBefore(:,index(atEvent));
end
