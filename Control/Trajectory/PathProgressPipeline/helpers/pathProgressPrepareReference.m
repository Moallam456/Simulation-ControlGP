function reference = pathProgressPrepareReference(data)
%PATHPROGRESSPREPAREREFERENCE Pack verified S-curves for continuous evaluation.
% Numeric segment records hold global time bounds, spline arrays and phase
% rows [localStart, localEnd, s0, sd0, sdd0, sddd]. Time is in seconds.
n = numel(data.segments);
reference.edges = zeros(1,n+1);
for k = 1:n
    profile = data.segments(k).profile;
    assert(profile.type=="S-Curve" && numel(profile.segmentDurations)==7, ...
        'PathProgress:UnsupportedTiming','Continuous evaluation requires the existing seven-phase S-curve.');
    h = profile.segmentDurations;
    jerk = profile.jPeak*[1 0 -1 0 -1 0 1];
    phases = zeros(7,6);
    s = 0; sd = 0; sdd = 0; local = 0;
    for phase = 1:7
        dt = h(phase);
        j = jerk(phase);
        phases(phase,:) = [local,local+dt,s,sd,sdd,j];
        s = s+dt*(sd+dt*(sdd/2+dt*j/6));
        sd = sd+dt*(sdd+dt*j/2);
        sdd = sdd+dt*j;
        local = local+dt;
    end
    assert(all(isfinite(phases(:))) && all(h>=0) && abs(local-profile.T)<=1e-12 && ...
        max(abs([s-1,sd,sdd]))<=1e-12,'Invalid continuous S-curve profile.');
    reference.edges(k+1) = reference.edges(k)+profile.T;
    reference.segment(k) = struct('phase',phases,'nodes',data.fit(k).nodes, ...
        'coefficients',data.fit(k).coefficients);
end
end
