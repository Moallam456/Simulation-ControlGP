function out = ft_compare_filters(Wmeas, cfg)
%FT_COMPARE_FILTERS Three causal, sample-aligned measurement paths.
% Filters operate on each channel independently; startup states are zero.
% Requires Signal Processing Toolbox (butter, filter).
assert(isnumeric(Wmeas) && isreal(Wmeas) && ...
    size(Wmeas,2) == 6 && size(Wmeas,1) >= 2 && ...
    all(isfinite(Wmeas(:))), 'Expected finite N-by-6 measurements.');
fs = 1/cfg.Ts;
fc = cfg.filter_cutoff_hz;
validateattributes(fc, {'numeric'}, ...
    {'scalar','real','finite','positive','<',fs/2});

% Exponential filter: pole-matched analog cutoff (not exact digital -3 dB).
alpha = 1-exp(-2*pi*fc/fs);
out.raw = Wmeas;
out.ema = filter(alpha,[1 -(1-alpha)],Wmeas,[],1);

% Butterworth: -3 dB at specified digital cutoff, 2nd order.
[b,a] = butter(2,fc/(fs/2),'low');
out.butterworth = filter(b,a,Wmeas,[],1);
out.ema_alpha = alpha;
out.butter_b = b;
out.butter_a = a;
assert(all(isfinite(out.ema(:))) && all(isfinite(out.butterworth(:))));
end
