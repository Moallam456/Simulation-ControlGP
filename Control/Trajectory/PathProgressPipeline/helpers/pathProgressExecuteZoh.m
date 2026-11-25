function [state,command] = pathProgressExecuteZoh(ticks,qTarget,qdTarget,qInitial,kp)
%PATHPROGRESSEXECUTEZOH Exact integration of piecewise-constant joint commands.
% ticks: strictly increasing seconds, starting at zero. Targets and outputs
% are joints-by-ticks. qInitial and kp are joint columns, in rad and 1/s.
% kp=0 runs feedforward. Command i acts from tick i up to tick i+1.
if isempty(ticks) || ticks(1)~=0 || any(~isfinite(ticks)) || any(diff(ticks)<=0) || ...
        ~isequal(size(qTarget),size(qdTarget),[numel(qInitial),numel(ticks)]) || ...
        ~isequal(size(qInitial),size(kp),[numel(qInitial),1]) || ...
        any(~isfinite([qTarget,qdTarget,qInitial,kp]),'all') || any(kp<0)
    error('PathProgress:InvalidExecutorInput','Invalid velocity executor times, references, or gains.');
end
state = zeros(size(qTarget));
command = state;
state(:,1) = qInitial;
for k = 1:numel(ticks)
    if k>1
        state(:,k) = state(:,k-1)+command(:,k-1)*(ticks(k)-ticks(k-1));
    end
    command(:,k) = qdTarget(:,k)+kp.*(qTarget(:,k)-state(:,k));
end
end
