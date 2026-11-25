function pathProgressWriteExecutionReport(output,S,W,C,T,observation,offset)
%PATHPROGRESSWRITEEXECUTIONREPORT Write the concise numeric run report.
file = fopen(fullfile(output,'report.md'),'w','n','UTF-8');
assert(file>=0,'Could not write the results report.');
cleanup = onCleanup(@() fclose(file));
fprintf(file,'# Ideal velocity-command execution\n\nMATLAB %s. Post-trajectory observation %.3g s. Initial offset [deg] %s.\n\n', ...
    version('-release'),observation,mat2str(rad2deg(offset).',4));
fprintf(file,'## Exact-length timing cleanup\n\n');
for i = 1:height(T)
    fprintf(file,'- %s used %d → %d samples. Duration %.9f → %.9f s and peak intended speed %.9f → %.9f m/s.\n', ...
        T.Case(i),T.PriorSamples(i),T.ExactSamples(i),T.PriorDuration_s(i),T.ExactDuration_s(i), ...
        T.PriorPeakSpeed_m_s(i),T.ExactPeakSpeed_m_s(i));
end
fprintf(file,'\n## Tracking results\n\nEach row selects the lowest motion RMS TCP position error in its group. Completion and post-observation errors are separate.\n\n');
fprintf(file,'| Case | Start | Period (ms) | Kp (1/s) | RMS TCP (mm) | Peak TCP (mm) | Final TCP (mm) | Observed TCP (mm) |\n');
fprintf(file,'|---|---|---:|---:|---:|---:|---:|---:|\n');
for name = unique(S.Case,'stable').'
    for initial = ["Matched","Offset"]
        rows = S(S.Case==name & S.Initial==initial,:);
        [~,best] = min(rows.RmsTcp_m);
        r = rows(best,:);
        fprintf(file,'| %s | %s | %d | %g | %.5g | %.5g | %.5g | %.5g |\n',name,initial, ...
            r.Period_ms,r.Kp_per_s,1000*[r.RmsTcp_m,r.PeakTcp_m,r.CompletionTcp_m,r.SettledTcp_m]);
    end
end
fprintf(file,'\nMatched starts show the sampling trend most directly. Across the four paths, feedforward RMS TCP error ranged from %.4g–%.4g mm at 50 ms and %.4g–%.4g mm at 1 ms.\n', ...
    rmsRange(S,50,0,"Matched"),rmsRange(S,1,0,"Matched"));
fprintf(file,'At 1 ms with offset starts, feedforward RMS TCP error ranged from %.4g–%.4g mm and Kp=8/s reduced it to %.4g–%.4g mm. The feedback correction can raise peak speed error at the start.\n', ...
    rmsRange(S,1,0,"Offset"),rmsRange(S,1,8,"Offset"));
fprintf(file,'The largest command magnitude was %.6g rad/s, and the largest adjacent command increment was %.6g rad/s.\n', ...
    max(S.MaxCommand_rad_s),max(S.MaxCommandIncrement_rad_s));
fprintf(file,'\n%d cases flagged position violations and %d flagged velocity violations against the model’s provisional limits.\n\n', ...
    nnz(S.PositionViolationEvents),nnz(S.VelocityViolationTicks));
fprintf(file,'At %d of %d internal waypoint observations a periodic tick coincided with the waypoint. Maximum held TCP speed at a waypoint was %.5g m/s before and %.5g m/s after.\n\n', ...
    nnz(W.TickAtWaypoint),height(W),max(W.TcpSpeedBefore_m_s),max(W.TcpSpeedAfter_m_s));
fprintf(file,'## Observation-grid convergence\n\n');
for i = 1:3:height(C)
    r = C(i,:);
    fprintf(file,'- %s, %d ms, Kp=%g/s, %s. Peak TCP %.6g → %.6g mm and peak speed error %.6g → %.6g m/s from 0.5 to 0.125 ms.\n', ...
        r.Case,r.Period_ms,r.Kp_per_s,r.Initial,1000*r.PeakTcp_m,1000*C.PeakTcp_m(i+2), ...
        r.PeakSpeedError_m_s,C.PeakSpeedError_m_s(i+2));
end

fprintf(file,['\n## Interpretation\n\n' ...
    'Motion errors compare equal-time FK of simulated and reference joints. Cross-path error is distance to the finite requested geometry. ' ...
    'Speed uses the Jacobian at simulated position and the held command. Command changes are velocity increments in an instantaneous kinematic model. ' ...
    'Application acceptance limits and hardware acceleration and jerk ratings remain unspecified. Before choosing hardware rates or gains, obtain drive command semantics, command and feedback timing, and measured velocity-loop response.\n']);
end

function range = rmsRange(S,period,gain,initial)
values = 1000*S.RmsTcp_m(S.Period_ms==period & S.Kp_per_s==gain & S.Initial==initial);
range = [min(values),max(values)];
end
