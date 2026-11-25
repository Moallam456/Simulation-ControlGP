function pathProgressWriteDriveReport(folder,S,W,C,cfg)
%PATHPROGRESSWRITEDRIVEREPORT Write a concise report from measured cases.
file = fopen(fullfile(folder,'report.md'),'w','n','UTF-8');
assert(file>=0,'Could not write report.');
cleanup = onCleanup(@() fclose(file));
fprintf(file,'# Waypoint stops and drive-response sensitivity\n\nMATLAB %s. Drive parameters are illustrative simulation values.\n\n',version('-release'));
fprintf(file,'The stop gate used %.5g rad position error, %.5g rad/s actual joint speed, %.3g s dwell and %.3g s timeout. The %.3g ms gate task conservatively certified the constant-input interval between events. Periodic commands stayed on the wall clock.\n\n', ...
    cfg.positionTol,cfg.velocityTol,cfg.dwell,cfg.timeout,1000*cfg.gateStep);
fprintf(file,'| Group | Case | Complete | Timeout | Provisional violations |\n|---|---|---:|---:|---:|\n');
for group = unique(S.Group,'stable').'
    names = unique(S.Case(S.Group==group),'stable');
    for name = names.'
        rows = S(S.Group==group & S.Case==name,:);
        violations = rows.PositionViolation | rows.CommandVelocityViolation | rows.ActualVelocityViolation;
        fprintf(file,'| %s | %s | %d | %d | %d |\n',group,name,nnz(rows.Outcome=="Complete"), ...
            nnz(rows.Outcome=="Timeout"),nnz(violations));
    end
end
fprintf(file,'\n## 50 ms command examples\n\nValues compare ideal and 50 ms lag plus 10 ms delay, in that order. All rows start matched. Timeout metrics cover only the executed prefix. Periodic circle completion marks the end of planned motion.\n\n');
fprintf(file,'| Case | Gain (1/s) | Outcome | RMS TCP (mm) | Peak speed error (mm/s) | Added settling (s) |\n|---|---:|---|---:|---:|---:|\n');
for name = unique(S.Case,'stable').'
    for gain = [2,8]
        group = S(S.Group=="Matrix" & S.Case==name & S.Period_ms==50 & S.Gain_per_s==gain,:);
        a = group(group.Tau_ms==0 & group.Delay_ms==0,:);
        b = group(group.Tau_ms==50 & group.Delay_ms==10,:);
        fprintf(file,'| %s | %g | %s → %s | %.4g → %.4g | %.4g → %.4g | %.4g → %.4g |\n',name,gain,a.Outcome,b.Outcome, ...
            1000*[a.RmsTcp_m,b.RmsTcp_m,a.PeakSpeedError_m_s,b.PeakSpeedError_m_s], ...
            a.Settling_s,b.Settling_s);
    end
end
fprintf(file,'\n## Boundary comparison with the fixed offset at 50 ms\n\n');
fprintf(file,'| Stop handling | Wall time (s) | Added settling (s) | Peak TCP (mm) | First boundary aligned | Second boundary aligned |\n|---|---:|---:|---:|---|---|\n');
pair = S(S.Group=="Stop comparison" & S.Initial=="Offset" & S.Period_ms==50,:);
for i = 1:height(pair)
    r = pair(i,:);
    w = W(W.Scenario==r.Scenario,:);
    fprintf(file,'| %s | %.6g | %.6g | %.6g | %s | %s |\n',string(r.Stops), ...
        r.Executed_s,r.Settling_s,1000*r.PeakTcp_m,string(w.PeriodicTickAtBoundary(1)), ...
        string(w.PeriodicTickAtBoundary(2)));
end
fprintf(file,'\nBoth negative tests timed out at the first waypoint and the next segment was never started. The delayed test used the same queue for boundary and periodic commands.\n');
fprintf(file,'\n## Observation convergence\n\n');
for i = 1:3:height(C)
    a = C(i,:); b = C(i+2,:);
    fprintf(file,'- %s, %s. At 1 → 0.25 ms observation spacing, peak TCP error %.6g → %.6g mm and peak speed error %.6g → %.6g mm/s.\n', ...
        a.Scenario,a.Case,1000*[a.PeakTcp_m,b.PeakTcp_m,a.PeakSpeedError_m_s,b.PeakSpeedError_m_s]);
end
fprintf(file,['\nThe plant starts at rest with zero held input. A boundary command uses the same delay as a periodic command. ' ...
    'The simulation ends at the terminal event and reports any still-pending commands. ' ...
    'Rate steps in the zero-lag model do not represent physical acceleration or jerk. ' ...
    'Hardware work requires measured velocity-loop response, transport latency and jitter, limits, and a defined stop acceptance criterion.\n']);
end
