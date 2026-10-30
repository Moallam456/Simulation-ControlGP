%% FILTER CUTOFF SWEEP — SIX-AXIS HAND-GUIDING LAB
% Run setup_handguiding_lab.m, then run this file from experiments/.
% Click Run in the MATLAB Editor; do not change any Simulink blocks.
%
% Requires existing models:
%   6D/FT_Sensor/simulink/FT_Sensor_Lab.slx
%   6D/simulink/HandGuiding_Testbench.slx
% and their referenced subsystems.
%
% Measures the filter/sensor/controller tradeoff for six cutoff frequencies.
% All cases reuse the same wrench trajectory, fixed M/D/Ts, tare estimate,
% and seeded sensor-noise realization.
%
% This is a synthetic, model-level experiment, not physical safety validation.

%% 1. Project paths and setup (without repeated optimization)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(repoRoot);
addpath(fullfile(repoRoot,'6D'));
addpath(fullfile(repoRoot,'6D','models'));
addpath(fullfile(repoRoot,'6D','Simulink'));
addpath(fullfile(repoRoot,'6D','FT_Sensor'));
addpath(fullfile(repoRoot,'6D','FT_Sensor','simulink'));

required = {'cfg','M','D','Ts','A6','B6','ft_wrench_ts', ...
            'noise_mean','noise_variance','noise_seed'};
for i = 1:numel(required)
    assert(exist(required{i},'var')==1, ...
        'Run setup_handguiding_lab.m first (missing %s).',required{i});
end

%% 2. Experiment configuration and validation
fcList_Hz = [2, 5, 10, 15, 20, 30];
Fs = 1/Ts;
assert(all(fcList_Hz > 0 & fcList_Hz < Fs/2), ...
    'Cutoff frequencies must be between 0 and Nyquist.');
assert(abs(Ts-0.002)<1e-12 && abs(cfg.Ts-Ts)<1e-12, ...
    'This laboratory expects matching 500 Hz sampling.');

Wtrue = squeeze(ft_wrench_ts.Data);
t = ft_wrench_ts.Time(:);
N = numel(t);
assert(isequal(size(Wtrue),[N,6]), ...
    'ft_wrench_ts must contain N-by-6 wrench samples.');
assert(all(isfinite(Wtrue(:))) && all(isfinite(t)), ...
    'Wrench samples and time must be finite.');
assert(max(abs(diff(t)-Ts)) < 1e-9, ...
    'Wrench time vector must be sampled uniformly at Ts.');
assert(abs(t(1))<1e-12, 'Wrench time must begin at 0.');
assert(numel(A6)==6 && numel(B6)==6 && numel(M)==6 && numel(D)==6, ...
    'Expected six-axis admittance parameters.');

quiet = t>=0.1 & t<0.4;       % Unloaded tare/noise window
plateau = t>=1.2 & t<1.4;    % Fx=5 N plateau window
stepStart_s = 0.5;
assert(nnz(quiet)>=20 && nnz(plateau)>=20, ...
    'Test signal must include the specified quiet and plateau windows.');
if max(abs(Wtrue(quiet,:)),[],'all') > 1e-5
    warning(['Nominal wrench is not zero on all axes in the tare window. ' ...
        'Check ft_generate_wrench.m; the tare/noise comparison may be biased.']);
end

sensorModel = 'FT_Sensor_Lab';
systemModel = 'HandGuiding_Testbench';
load_system(sensorModel);
load_system(systemModel);

switchBlocks = find_system(systemModel, 'SearchDepth',1, ...
    'BlockType','ManualSwitch');
assert(numel(switchBlocks)==1, ...
    'Expected exactly one top-level Manual Switch in %s.',systemModel);
switchPath = switchBlocks{1};

% Same parameter names as the working analyze_handguiding_lab.m script.
p = struct('cfg',cfg, 'M',M, 'D',D, 'Ts',Ts, ...
    'A6',A6, 'B6',B6, 'ft_wrench_ts',ft_wrench_ts, ...
    'noise_mean',noise_mean, ...
    'noise_variance',noise_variance, ...
    'noise_seed',noise_seed, ...
    'alpha',1-exp(-2*pi*10/Fs), ...
    'bButter',[], 'aButter',[], ...
    'bias_est',zeros(6,1));
[p.bButter,p.aButter] = butter(2,10/(Fs/2),'low');

%% 3. Calibrate once using the raw synthetic measurement
% Do NOT re-estimate tare for each cutoff: that would confound comparisons.
rawRun = sim(localInput(sensorModel,p,t(end)));
raw = localSignal(rawRun,'ft_raw_ts',t);
bias_est = mean(raw(quiet,:),1)';
p.bias_est = bias_est;

%% 4. Ideal six-axis admittance response (no sensor/filter)
% This is a comparison baseline, NOT a MATLAB-vs-Simulink equivalence test.
a = A6(:)';
b = B6(:)';
Videal = zeros(N,6);
Videal(1,:) = b.*Wtrue(1,:);  % Zero previous force and velocity
for k = 2:N
    Videal(k,:) = a.*Videal(k-1,:) + ...
        b.*(Wtrue(k,:)+Wtrue(k-1,:));
end

%% 5. Cutoff sweep; same synthetic inputs and noise for each case
nF = numel(fcList_Hz);
nCases = 2*nF;
filterNames = ["EMA", "Butterworth"];

Filter = strings(nCases,1);
Cutoff_Hz = zeros(nCases,1);
FxNoiseStd_N = zeros(nCases,1);
TxNoiseStd_Nm = zeros(nCases,1);
FxT90_ms = NaN(nCases,1);
TransMaxError_mm_s = zeros(nCases,1);
TransRMS_mm_s = zeros(nCases,1);
RotMaxError_rad_s = zeros(nCases,1);
RotRMS_rad_s = zeros(nCases,1);

axisNames = ["X";"Y";"Z";"Rx";"Ry";"Rz"];
units = ["mm/s";"mm/s";"mm/s";"rad/s";"rad/s";"rad/s"];
AxisFilter = strings(6*nCases,1);
AxisCutoff_Hz = zeros(6*nCases,1);
Axis = strings(6*nCases,1);
AxisMaxError = zeros(6*nCases,1);
AxisRMS = zeros(6*nCases,1);
AxisUnits = strings(6*nCases,1);

% Keep detailed traces for later plots without rerunning Simulink.
Wfiltered = zeros(N,6,2,nF);
Vfiltered = zeros(N,6,2,nF);
WrawBaseline = [];

for j = 1:nF
    fc = fcList_Hz(j);
    fprintf('Cutoff %g Hz (%d/%d)\n',fc,j,nF);

    % Keep original lab's EMA definition for comparability with 10 Hz tests.
    % EMA fc denotes matched continuous-time pole, not exact digital -3 dB.
    p.alpha = 1-exp(-2*pi*fc/Fs);
    [p.bButter,p.aButter] = butter(2,fc/(Fs/2),'low');

    sensorRun = sim(localInput(sensorModel,p,t(end)));
    WrawCase = localSignal(sensorRun,'ft_raw_ts',t);
    Wema = localSignal(sensorRun,'ft_ema_ts',t);
    Wbw = localSignal(sensorRun,'ft_butter_ts',t);

    if isempty(WrawBaseline)
        WrawBaseline = WrawCase;
    elseif max(abs(WrawCase-WrawBaseline),[],'all') > 1e-10
        error('Raw sensor noise changed between cutoffs: check RNG seed.');
    end

    % Port 2 => EMA ('sw'='0'); port 1 => Butterworth ('sw'='1').
    for m = 1:2
        row = (j-1)*2+m;
        Filter(row) = filterNames(m);
        Cutoff_Hz(row) = fc;

        in = localInput(systemModel,p,t(end));
        if m==1
            in = setBlockParameter(in,switchPath,'sw','0');
            Wcase = Wema;
        else
            in = setBlockParameter(in,switchPath,'sw','1');
            Wcase = Wbw;
        end

        out = sim(in);
        Vcase = localSignal(out,'twist_integrated_ts',t);
        Wfiltered(:,:,m,j) = Wcase;
        Vfiltered(:,:,m,j) = Vcase;

        % Sensor metrics on unloaded samples; Fx force and Tx torque.
        FxNoiseStd_N(row) = std(Wcase(quiet,1));
        TxNoiseStd_Nm(row) = std(Wcase(quiet,4));
        FxT90_ms(row) = localT90(t,Wcase(:,1),quiet,plateau,stepStart_s);

        % Six-axis output error relative to the same ideal wrench response.
        e = Vcase-Videal;
        assert(all(isfinite(e(:))), ...
            'Non-finite controller output at %g Hz (%s).',fc,Filter(row));
        TransMaxError_mm_s(row) = 1e3*max(abs(e(:,1:3)),[],'all');
        TransRMS_mm_s(row) = 1e3*sqrt(mean(e(:,1:3).^2,'all'));
        RotMaxError_rad_s(row) = max(abs(e(:,4:6)),[],'all');
        RotRMS_rad_s(row) = sqrt(mean(e(:,4:6).^2,'all'));

        axisMax = max(abs(e),[],1)';
        axisRMS = sqrt(mean(e.^2,1))';
        axisMax(1:3) = 1e3*axisMax(1:3);
        axisRMS(1:3) = 1e3*axisRMS(1:3);
        ids = (row-1)*6+(1:6);
        AxisFilter(ids) = Filter(row);
        AxisCutoff_Hz(ids) = fc;
        Axis(ids) = axisNames;
        AxisMaxError(ids) = axisMax;
        AxisRMS(ids) = axisRMS;
        AxisUnits(ids) = units;
    end
end

%% 6. Tables
SweepSummary = table(Filter,Cutoff_Hz,FxNoiseStd_N,TxNoiseStd_Nm, ...
    FxT90_ms,TransMaxError_mm_s,TransRMS_mm_s, ...
    RotMaxError_rad_s,RotRMS_rad_s);
SweepByAxis = table(AxisFilter,AxisCutoff_Hz,Axis,AxisMaxError, ...
    AxisRMS,AxisUnits, ...
    'VariableNames', {'Filter','Cutoff_Hz','Axis','MaxError','RMS','Unit'});

% Results are kept in the MATLAB workspace; no exports or directories.
Metadata = struct('sampling_Hz',Fs, 'Ts',Ts, 'cutoffs_Hz',fcList_Hz, ...
    'seed',noise_seed, 'M',M, 'D',D, 'bias_est',bias_est, ...
    'sensor_model',sensorModel, 'system_model',systemModel, ...
    'quiet_interval_s',[0.1,0.4], ...
    'step_time_s',stepStart_s, ...
    'experiment_type','synthetic, nominal sensor signals', ...
    'ema_frequency_definition','alpha=1-exp(-2*pi*fc/Fs)');

%% 7. Tradeoff figure: use measured quantities; no arbitrary score
ixEMA = SweepSummary.Filter=="EMA";
ixBW = SweepSummary.Filter=="Butterworth";
fig = figure('Name','Filter cutoff tradeoffs','Color','w');
tiledlayout(2,2,'Padding','compact');

nexttile;
plot(fcList_Hz,FxNoiseStd_N(ixEMA),'o-','LineWidth',1.4); hold on;
plot(fcList_Hz,FxNoiseStd_N(ixBW),'s-','LineWidth',1.4);
xlabel('Nominal cutoff (Hz)'); ylabel('Unloaded Fx std (N)');
title('Remaining sensor noise'); legend('EMA','Butterworth','Location','best'); grid on;

nexttile;
plot(fcList_Hz,FxT90_ms(ixEMA),'o-','LineWidth',1.4); hold on;
plot(fcList_Hz,FxT90_ms(ixBW),'s-','LineWidth',1.4);
xlabel('Nominal cutoff (Hz)'); ylabel('Fx step T90 (ms)');
title('Filter response time'); legend('EMA','Butterworth','Location','best'); grid on;

nexttile;
plot(fcList_Hz,TransRMS_mm_s(ixEMA),'o-','LineWidth',1.4); hold on;
plot(fcList_Hz,TransRMS_mm_s(ixBW),'s-','LineWidth',1.4);
xlabel('Nominal cutoff (Hz)'); ylabel('RMS translation error (mm/s)');
title('Three-axis translation'); legend('EMA','Butterworth','Location','best'); grid on;

nexttile;
plot(fcList_Hz,RotRMS_rad_s(ixEMA),'o-','LineWidth',1.4); hold on;
plot(fcList_Hz,RotRMS_rad_s(ixBW),'s-','LineWidth',1.4);
xlabel('Nominal cutoff (Hz)'); ylabel('RMS rotation error (rad/s)');
title('Three-axis rotation'); legend('EMA','Butterworth','Location','best'); grid on;


%% 8. Keep a representative 10 Hz velocity comparison
k10 = find(fcList_Hz==10,1,'first');
if ~isempty(k10)
    fig2 = figure('Name','10 Hz controller response','Color','w');
    plot(t,1e3*Videal(:,1),'k','LineWidth',1.5); hold on;
    plot(t,1e3*Vfiltered(:,1,1,k10),'--','LineWidth',1.2);
    plot(t,1e3*Vfiltered(:,1,2,k10),'--','LineWidth',1.2);
    xlabel('Time (s)'); ylabel('X velocity (mm/s)');
    title('Admittance output at 10 Hz nominal cutoff');
    legend('Ideal','EMA','Butterworth','Location','best'); grid on;
end

fprintf('\n=== CUTOFF SWEEP SUMMARY ===\n');
disp(SweepSummary);
fprintf('Sweep complete. Results and interactive figures remain in MATLAB; no files exported.\n');
fprintf(['Interpretation: lower noise, lower step delay and lower control ' ...
    'error are competing objectives. No single optimum is declared ' ...
    'without design limits.\n']);

%% Local helpers (no changes to referenced subsystems)
function in = localInput(model,p,stopTime)
    in = Simulink.SimulationInput(model);
    fields = fieldnames(p);
    for n = 1:numel(fields)
        in = setVariable(in,fields{n},p.(fields{n}));
    end
    in = setModelParameter(in, ...
        'StopTime',num2str(stopTime,17), ...
        'ReturnWorkspaceOutputs','on');
end

function data = localSignal(out,varName,t)
    ts = out.get(varName);
    assert(isa(ts,'timeseries'), ...
        'Missing timeseries output "%s"; check To Workspace block.',varName);
    data = squeeze(ts.Data);
    N = numel(t);
    if isequal(size(data),[6 N]), data = data'; end
    assert(isequal(size(data),[N 6]), ...
        'Unexpected dimensions for "%s"; expected %d-by-6.',varName,N);
    assert(numel(ts.Time)==N && max(abs(ts.Time(:)-t(:)))<1e-9, ...
        'Sample times do not match for "%s".',varName);
    assert(all(isfinite(data(:))), 'Nonfinite data for "%s".',varName);
end

function time90_ms = localT90(t,signal,quiet,plateau,stepStart)
    baseline = mean(signal(quiet));
    finalLevel = mean(signal(plateau));
    delta = finalLevel-baseline;
    assert(delta>0.5, ...
        'Fx step appears absent; verify the synthetic wrench trajectory.');
    threshold = baseline+0.9*delta;
    idx = find(t>=stepStart & t<1.5 & signal>=threshold,1,'first');
    if isempty(idx)
        time90_ms = NaN;
        warning('Fx did not reach 90%% of its measured plateau.');
    else
        time90_ms = 1000*(t(idx)-stepStart);
    end
end
