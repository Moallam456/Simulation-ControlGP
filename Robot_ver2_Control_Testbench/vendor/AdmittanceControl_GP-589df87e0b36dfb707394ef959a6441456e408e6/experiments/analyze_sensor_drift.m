%% SENSOR DRIFT — CALIBRATION DOES NOT LAST FOREVER
% Experiment: true force = 0 for 4 s, but sensor bias slowly changes.
% Compare drift OFF vs ON using your existing referenced Simulink models.
% Run setup_handguiding_lab.m once before running this script.
% No model edits, no automatic saved files, and no new filter tuning.

%% 1. Prepare the existing 500 Hz experiment
assert(exist('cfg','var')==1 && exist('A6','var')==1 && ...
       exist('ft_wrench_ts','var')==1, ...
       'Run setup_handguiding_lab.m first.');

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'6D','Simulink'));
addpath(fullfile(root,'6D','FT_Sensor','simulink'));
load_system('FT_Sensor_Lab');
load_system('HandGuiding_Testbench');

t = ft_wrench_ts.Time(:);
N = numel(t);
assert(abs(Ts-0.002)<1e-12 && abs(t(end)-4)<1e-9, ...
       'Expected 500 Hz sampling over 4 seconds.');

driftRate = cfg.sensor.drift_per_s(:);  % [N/s; N.m/s], six channels
assert(numel(driftRate)==6 && any(driftRate~=0), ...
       'Configure a nonzero six-axis drift rate first.');

% The operator does NOT touch the handle during this experiment.
zeroWrench = timeseries(zeros(N,6),t);
tareWindow = t>=0.1 & t<0.4;
cutoffHz = 20;                         % Keep the EMA setting FIXED.

%% 2. Share one controller/noise configuration across both cases
% Simulink.SimulationInput overrides variables for each run; it doesn't
% permanently change the Ramp or the Manual Switch in your saved models.
p = struct('cfg',cfg,'M',M,'D',D,'Ts',Ts, ...
    'A6',A6,'B6',B6,'ft_wrench_ts',zeroWrench, ...
    'noise_mean',noise_mean,'noise_variance',noise_variance, ...
    'noise_seed',noise_seed,'bias_est',zeros(6,1), ...
    'alpha',1-exp(-2*pi*cutoffHz*Ts), ...
    'bButter',bButter,'aButter',aButter);

switches = find_system('HandGuiding_Testbench', ...
    'SearchDepth',1,'BlockType','ManualSwitch');
assert(numel(switches)==1, 'Expected one Manual Switch.');

% Columns/cases: 1 = no drift, 2 = drift enabled.
V = zeros(N,6,2);       % Cartesian twist [m/s; rad/s]
Q = zeros(N,6,2);       % Integrated command [m; rad]
raw = zeros(N,6,2);     % Uncalibrated sensor measurements

%% 3. Calibrate at startup, then evaluate with independent noise
for caseID = 1:2
    p.cfg.sensor.drift_per_s = (caseID-1)*driftRate;

    % Calibration starts with bias_est=0. Measure the mean unloaded offset
    % over 0.1-0.4 s; calibration also absorbs a LITTLE early drift.
    p.bias_est = zeros(6,1);
    p.noise_seed = noise_seed;
    tareRun = sim(makeRun('FT_Sensor_Lab',p,t(end)));
    rawTS = tareRun.get('ft_raw_ts');
    assert(isa(rawTS,'timeseries'), 'Missing raw F/T timeseries.');
    raw(:,:,caseID) = squeeze(rawTS.Data);
    p.bias_est = mean(raw(tareWindow,:,caseID),1)';

    % A different seed represents fresh measurements after calibration.
    % BOTH cases use this SAME seed, so differences isolate the drift.
    p.noise_seed = noise_seed + 1000;
    runInput = makeRun('HandGuiding_Testbench',p,t(end));
    runInput = setBlockParameter(runInput,switches{1},'sw','0');
    output = sim(runInput);
    twist = output.get('twist_integrated_ts');
    assert(isa(twist,'timeseries') && ...
           numel(twist.Time)==N && max(abs(twist.Time(:)-t))<1e-9, ...
           'Velocity logging or sample times changed.');
    Vcase = squeeze(twist.Data);
    assert(isequal(size(Vcase),[N 6]), ...
           'Expected N-by-6 twist output.');
    V(:,:,caseID) = Vcase;

    % ZOH integration estimates the requested position change.
    % It is NOT a measurement of the real robot's position.
    Q(:,:,caseID) = [zeros(1,6);cumsum(Vcase(1:end-1,:)*Ts,1)];
end

%% 4. Check that the new Ramp actually changes the sensor measurement
% With identical calibration-run noise, the final raw difference should
% equal driftRate * 4 s. If not, inspect the Ramp slope in FT_Sensor_6D.
rawDifference = squeeze(raw(end,:,2)-raw(end,:,1))';
assert(max(abs(rawDifference-driftRate*t(end)))<1e-7, ...
       'Drift did not reach the sensor output; check Ramp parameter/wiring.');

%% 5. Measure the drift-only effect (subtract the no-drift case)
deltaV = V(:,:,2)-V(:,:,1);
deltaQ = Q(:,:,2)-Q(:,:,1);

DriftSummary = table( ...
    rawDifference(1), ...
    1000*deltaV(end,1), ...
    1000*max(abs(deltaV(:,1))), ...
    1000*deltaQ(end,1), ...
    1000*norm(deltaQ(end,1:3)), ...
    'VariableNames',{'FinalExtraFx_N','FinalExtraVx_mm_s', ...
    'PeakExtraVx_mm_s','FinalExtraX_mm','FinalExtraTranslation_mm'});

disp('=== DRIFT ON MINUS DRIFT OFF (20 Hz EMA) ===');
disp(DriftSummary);

%% 6. Visualize what changed (nothing is written to disk)
figure('Name','Sensor drift — with and without');
tiledlayout(3,1);

nexttile;
plot(t,squeeze(raw(:,1,2)-raw(:,1,1)),'LineWidth',1.3);
ylabel('Extra Fx (N)'); title('Added raw sensor bias from drift'); grid on;

nexttile;
plot(t,1000*V(:,1,1),'LineWidth',1.2); hold on;
plot(t,1000*V(:,1,2),'LineWidth',1.2);
ylabel('Vx (mm/s)'); title('Commanded X velocity');
legend('Drift OFF','Drift ON','Location','best'); grid on;

nexttile;
plot(t,1000*deltaQ(:,1),'LineWidth',1.3);
xlabel('Time (s)'); ylabel('Extra X (mm)');
title('Additional integrated command caused by drift'); grid on;

%% Helper — supply data to existing models, not new Simulink blocks
function in = makeRun(model,p,tEnd)
    in = Simulink.SimulationInput(model);
    fields = fieldnames(p);
    for i = 1:numel(fields)
        in = setVariable(in,fields{i},p.(fields{i}));
    end
    in = setModelParameter(in,'StopTime',num2str(tEnd,17), ...
        'ReturnWorkspaceOutputs','on');
end
