
%% HAND-GUIDING LABORATORY — ANALYSIS
% Requires setup_handguiding_lab.m to have been run first.
% MATLAB + Simulink | 500 Hz | Synthetic experiments only

%% 1. Validate workspace and models
repoRoot = fileparts(fileparts(mfilename('fullpath')));

addpath(repoRoot);
addpath(fullfile(repoRoot,'6D','models'));
addpath(fullfile(repoRoot,'6D','Simulink'));
addpath(fullfile(repoRoot,'6D','FT_Sensor'));
addpath(fullfile(repoRoot,'6D','FT_Sensor','simulink'));

assert(exist('cfg','var') == 1 && ...
       exist('A6','var') == 1 && ...
       exist('B6','var') == 1 && ...
       exist('ft_wrench_ts','var') == 1, ...
       'Run setup_handguiding_lab.m first.');

assert(abs(cfg.Ts - Ts) < 1e-12);

sensorModel = 'FT_Sensor_Lab';
systemModel = 'HandGuiding_Testbench';

load_system(sensorModel);
load_system(systemModel);

t = ft_wrench_ts.Time(:);
Wtrue = ft_wrench_ts.Data;
N = numel(t);

assert(isequal(size(Wtrue),[N 6]));
assert(max(abs(diff(t)-Ts)) < 1e-10);

%% 2. Simulation parameters
p = struct( ...
    'cfg',cfg, ...
    'M',M, 'D',D, 'Ts',Ts, ...
    'A6',A6, 'B6',B6, ...
    'ft_wrench_ts',ft_wrench_ts, ...
    'noise_mean',noise_mean, ...
    'noise_variance',noise_variance, ...
    'noise_seed',noise_seed, ...
    'alpha',alpha, ...
    'bButter',bButter, ...
    'aButter',aButter, ...
    'bias_est',zeros(6,1));

%% 3. Estimate sensor bias from unloaded samples
out0 = sim(makeInput(sensorModel,p,t(end)));
raw0 = getSignal(out0,'ft_raw_ts',t);

quiet = t >= 0.1 & t < 0.4;
assert(nnz(quiet) >= 20);

bias_est = mean(raw0(quiet,:),1)';
p.bias_est = bias_est;

%% 4. Run calibrated sensor laboratory
outSensor = sim(makeInput(sensorModel,p,t(end)));

Wraw = getSignal(outSensor,'ft_raw_ts',t);
Wema = getSignal(outSensor,'ft_ema_ts',t);
Wbw  = getSignal(outSensor,'ft_butter_ts',t);

%% 5. Sensor noise and response metrics
method = ["Raw";"EMA";"Butterworth"];
signals = {Wraw,Wema,Wbw};

noiseStd = zeros(3,1);
offset = zeros(3,1);
time90 = zeros(3,1);

steady = t >= 1.2 & t < 1.4;

for i = 1:3
    Fx = signals{i}(:,1);

    noiseStd(i) = std(Fx(quiet));
    offset(i) = mean(Fx(quiet));

    finalValue = mean(Fx(steady));
    threshold = offset(i) + ...
        0.9*(finalValue-offset(i));

    idx = find(t >= 0.5 & t < 1.5 & ...
        Fx >= threshold,1,'first');

    assert(~isempty(idx),'Step threshold not reached.');
    time90(i) = 1000*(t(idx)-0.5);
end

SensorMetrics = table( ...
    method,noiseStd,offset,time90, ...
    'VariableNames', ...
    {'Method','NoiseStd_N','Offset_N','Time90_ms'});

%% 6. Calculate ideal MATLAB admittance response
Videal = zeros(N,6);

for k = 2:N
    Videal(k,:) = admittance_6d_tustin( ...
        Videal(k-1,:)', ...
        Wtrue(k-1,:)',Wtrue(k,:)', ...
        M,D,Ts)';
end

%% 7. Simulate both filters automatically
switches = find_system(systemModel, ...
    'SearchDepth',1,'BlockType','ManualSwitch');

assert(numel(switches)==1, ...
    'Expected one top-level Manual Switch.');

% Port 1 = Butterworth, Port 2 = EMA
inBW = makeInput(systemModel,p,t(end));
inBW = setBlockParameter(inBW, ...
    switches{1},'sw','1');

inEMA = makeInput(systemModel,p,t(end));
inEMA = setBlockParameter(inEMA, ...
    switches{1},'sw','0');

outBW = sim(inBW);
outEMA = sim(inEMA);

Vbw = getSignal(outBW,'twist_integrated_ts',t);
Vema = getSignal(outEMA,'twist_integrated_ts',t);

%% 8. Six-axis controller error metrics
Eema = Vema - Videal;
Ebw = Vbw - Videal;

maxEMA = max(abs(Eema),[],1)';
maxBW = max(abs(Ebw),[],1)';

rmsEMA = sqrt(mean(Eema.^2,1))';
rmsBW = sqrt(mean(Ebw.^2,1))';

% Translation in mm/s, rotation in rad/s
maxEMA(1:3) = 1000*maxEMA(1:3);
maxBW(1:3) = 1000*maxBW(1:3);
rmsEMA(1:3) = 1000*rmsEMA(1:3);
rmsBW(1:3) = 1000*rmsBW(1:3);

axisNames = ["X";"Y";"Z";"Rx";"Ry";"Rz"];
units = ["mm/s";"mm/s";"mm/s"; ...
         "rad/s";"rad/s";"rad/s"];

ControllerMetrics = table( ...
    axisNames,maxEMA,maxBW,rmsEMA,rmsBW,units);

%% 9. Generate comparison plot
fig = figure('Name','Hand-Guiding Filter Comparison');

plot(t,1000*Videal(:,1),'k','LineWidth',1.5);
hold on;
plot(t,1000*Vema(:,1),'b--','LineWidth',1.2);
plot(t,1000*Vbw(:,1),'r--','LineWidth',1.2);

legend('Ideal','EMA','Butterworth');
xlabel('Time (s)');
ylabel('X velocity (mm/s)');
title('Sensor Filtering Effect on Admittance');
grid on;

%% 10. Results summary
disp('=== SENSOR PERFORMANCE ===');
disp(SensorMetrics);

disp('=== SIX-AXIS CONTROLLER PERFORMANCE ===');
disp(ControllerMetrics);

fprintf('\nAnalysis complete. Tables and plots are available in MATLAB. No files exported.\n');

%% Local helper functions
function in = makeInput(model,p,stopTime)
    in = Simulink.SimulationInput(model);
    names = fieldnames(p);

    for j = 1:numel(names)
        in = setVariable(in,names{j},p.(names{j}));
    end

    in = setModelParameter(in, ...
        'StopTime',num2str(stopTime,17), ...
        'ReturnWorkspaceOutputs','on');
end

function data = getSignal(out,name,expectedTime)
    ts = out.get(name);
    assert(isa(ts,'timeseries'), ...
        'Missing Timeseries output: %s',name);

    data = squeeze(ts.Data);
    n = numel(expectedTime);

    if isequal(size(data),[6 n])
        data = data';
    end

    assert(isequal(size(data),[n 6]), ...
        'Unexpected dimensions for %s',name);

    assert(numel(ts.Time)==n && ...
        max(abs(ts.Time(:)-expectedTime(:))) < 1e-9, ...
        'Sampling mismatch for %s',name);

    assert(all(isfinite(data(:))), ...
        'Nonfinite samples in %s',name);
end
