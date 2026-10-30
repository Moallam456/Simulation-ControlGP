function metrics = ft_lab_metrics(t, Wtrue, filtered, cfg)
%FT_LAB_METRICS Compare noise, residual offset, step response, error.
% Force and torque RMSE use separate units; do not mix them in one norm.
t = t(:);
assert(size(Wtrue,1) == numel(t) && size(Wtrue,2) == 6);
quiet = t >= cfg.quiet_window_s(1) & t < cfg.quiet_window_s(2);
step = t >= cfg.step_window_s(1) & t < cfg.step_window_s(2);
assert(nnz(quiet) >= 20 && nnz(step) >= 20, ...
    'Windows must contain at least 20 samples.');
assert(all(abs(Wtrue(quiet,:)) < 1e-12,'all'), ...
    'Quiet window is not truly unloaded in the synthetic input.');

methods = ["Raw";"EMA";"Butterworth"];
names = {'raw','ema','butterworth'};
noiseStd = zeros(3,1);
offset = zeros(3,1);
t90ms = nan(3,1);
forceRMSE = zeros(3,1);
torqueRMSE = zeros(3,1);

for j = 1:3
    W = filtered.(names{j});
    assert(isequal(size(W),size(Wtrue)), 'Filter shape mismatch.');
    baseline = mean(W(quiet,1));
    noiseStd(j) = std(W(quiet,1),0);
    offset(j) = baseline;
    cutoff = baseline + 0.90*cfg.step_amplitude_N;
    ids = find(step & W(:,1) >= cutoff,1,'first');
    if ~isempty(ids)
        t90ms(j) = (t(ids)-cfg.step_window_s(1))*1000;
    end
    E = W-Wtrue;
    forceRMSE(j) = sqrt(mean(E(:,1:3).^2,'all'));
    torqueRMSE(j) = sqrt(mean(E(:,4:6).^2,'all'));
end

metrics = table(methods,noiseStd,offset,t90ms,forceRMSE,torqueRMSE, ...
    'VariableNames',{'Method','FxNoiseStd_N','FxOffset_N', ...
    'FxTime90_ms','ForceRMSE_N','TorqueRMSE_Nm'});
end
