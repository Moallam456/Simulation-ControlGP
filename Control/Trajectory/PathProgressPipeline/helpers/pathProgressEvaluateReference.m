function [q,qd,qdd,timing] = pathProgressEvaluateReference(reference,time)
%PATHPROGRESSEVALUATEREFERENCE Evaluate numeric S-curve and joint cubic data.
% Query times [s] lie in [0, reference.edges(end)]. Outputs are 6-by-N in
% rad, rad/s and rad/s^2. Timing rows are [segment; s; sd; sdd; sddd].
% Waypoints belong to the preceding segment. The S-curve defines exact
% rest states at segment endpoints and owns a phase boundary on its left.
time = time(:).';
if isempty(time) || ~isreal(time) || any(~isfinite(time) | time<0 | time>reference.edges(end))
    error('PathProgress:InvalidReferenceTime','Query time is outside the reference domain.');
end
n = numel(time);
q = zeros(size(reference.segment(1).coefficients,1),n);
qd = q;
qdd = q;
timing = zeros(5,n);
for i = 1:n
    k = 1;
    while time(i)>reference.edges(k+1)
        k = k+1;
    end
    phase = reference.segment(k).phase;
    local = time(i)-reference.edges(k);
    j = 1;
    while j<size(phase,1) && local>phase(j,2)+1e-12
        j = j+1;
    end
    h = local-phase(j,1);
    sn = phase(j,3)+h*(phase(j,4)+h*(phase(j,5)/2+h*phase(j,6)/6));
    sd = phase(j,4)+h*(phase(j,5)+h*phase(j,6)/2);
    sdd = phase(j,5)+h*phase(j,6);
    if time(i)==reference.edges(k+1)
        sn = 1; sd = 0; sdd = 0;
    elseif time(i)==reference.edges(k)
        sn = 0; sd = 0; sdd = 0;
    end
    % Allow only representational roundoff at the closed domain endpoints.
    if sn<0 && sn>=-8*eps, sn = 0; end
    if sn>1 && sn<=1+8*eps, sn = 1; end
    timing(:,i) = [k;sn;sd;sdd;phase(j,6)];
end
for k = 1:numel(reference.segment)
    selected = timing(1,:)==k;
    if ~any(selected), continue; end
    segment = reference.segment(k);
    [q(:,selected),qs,qss,qsss] = pathProgressEvaluateCubic( ...
        segment.nodes,segment.coefficients,timing(2,selected));
    [qd(:,selected),qdd(:,selected)] = pathProgressChainRule( ...
        qs,qss,qsss,timing(3,selected),timing(4,selected),timing(5,selected));
end
end
