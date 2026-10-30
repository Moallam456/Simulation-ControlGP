function Wmeas = ft_sensor_model(t, Wtrue, cfg)
%FT_SENSOR_MODEL Six-axis synthetic measurement, not a physical sensor twin.
% Wmeas = quantize(saturate(C*Wtrue + bias + drift*t + white noise)).
% Every random draw uses a local seeded stream; caller RNG is unchanged.
assert(isvector(t) && isnumeric(t) && isreal(t) && ...
    all(isfinite(t(:))) && numel(t) >= 2, 'Invalid timebase.');
t = t(:);
assert(isnumeric(Wtrue) && isreal(Wtrue) && ...
    size(Wtrue,1) == numel(t) && size(Wtrue,2) == 6 && ...
    all(isfinite(Wtrue(:))), 'Wtrue must be a finite N-by-6 matrix.');
assert(all(diff(t) > 0), 'Timebase must be strictly increasing.');
s = cfg.sensor;
validateattributes(cfg.seed, {'numeric'}, ...
    {'scalar','integer','nonnegative','finite'});
validateattributes(s.coupling, {'numeric'}, ...
    {'size',[6 6],'real','finite'});
fields = {'bias','noise_std','drift_per_s','quantization_step','saturation'};
for i = 1:numel(fields)
    v = s.(fields{i});
    assert(isnumeric(v) && isreal(v) && isequal(size(v),[6 1]), ...
        '%s must be a 6-by-1 vector.', fields{i});
end
assert(all(isfinite(s.bias)) && all(isfinite(s.drift_per_s)));
assert(all(isfinite(s.noise_std)) && all(s.noise_std >= 0));
assert(all(isfinite(s.quantization_step)) && all(s.quantization_step >= 0));
assert(all(s.saturation > 0) && all(~isnan(s.saturation)));

stream = RandStream('mt19937ar','Seed',cfg.seed);
noise = randn(stream,numel(t),6) .* s.noise_std.';
Wmeas = Wtrue*s.coupling.' + s.bias.' + ...
    (t-t(1))*s.drift_per_s.' + noise;

% Quantize after analog saturation. Zero resolution disables quantization.
for axis = 1:6
    limit = s.saturation(axis);
    Wmeas(:,axis) = max(-limit,min(limit,Wmeas(:,axis)));
    step = s.quantization_step(axis);
    if step > 0
        Wmeas(:,axis) = step*round(Wmeas(:,axis)/step);
    end
end
assert(all(isfinite(Wmeas(:))), 'Non-finite synthetic measurement.');
end
