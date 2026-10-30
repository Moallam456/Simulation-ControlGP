function [t, Wtrue] = ft_generate_wrench(cfg)
%FT_GENERATE_WRENCH Six-axis known input, sampled at exactly cfg.Ts.
% Wtrue is N-by-6, with forces [N] and torques [N*m].
validateattributes(cfg.Ts, {'double'}, {'scalar','real','finite','positive'});
validateattributes(cfg.duration_s, {'double'}, ...
    {'scalar','real','finite','positive'});
samplesExact = cfg.duration_s / cfg.Ts;
assert(abs(samplesExact - round(samplesExact)) < 1e-9, ...
    'Duration must be an integer number of 2 ms samples.');
t = (0:round(samplesExact))' * cfg.Ts;
Wtrue = zeros(numel(t),6);

% Known X-axis force step, followed by reversal and release.
Wtrue(t >= 0.50 & t < 1.50,1) = cfg.step_amplitude_N;
Wtrue(t >= 2.00 & t < 2.30,1) = -3.0;

% Independent varying inputs on the other five channels.
idx = t >= 0.70 & t < 2.20;
Wtrue(idx,2) = 2.0 * sin(2*pi*1.2*(t(idx)-0.70));
Wtrue(t >= 1.30 & t < 1.80,3) = 1.5;
Wtrue(t >= 1.10 & t < 1.60,4) = 0.25;
idx = t >= 0.80 & t < 2.80;
Wtrue(idx,5) = 0.20 * sin(2*pi*0.8*(t(idx)-0.80));
Wtrue(t >= 2.40 & t < 2.90,6) = -0.15;
end
