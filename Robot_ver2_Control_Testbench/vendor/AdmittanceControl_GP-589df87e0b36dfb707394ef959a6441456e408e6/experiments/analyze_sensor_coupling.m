%% CROSS-AXIS COUPLING — FORCE IN X, FALSE MOTION IN Y
% Question: Can a pure X push appear as Y force and command Y motion?
% Uses YOUR FT_Sensor_6D + Admittance_Controller_6D Simulink references.
% Run setup_handguiding_lab.m once first. No model edits or file exports.
%
% We compare two otherwise IDENTICAL experiments:
%   OFF: perfect channel separation (coupling = eye(6))
%   ON:  measured channels mix using cfg.sensor.coupling
% Noise/bias stay identical; drift is disabled to isolate ONLY coupling.

%% 1. Prepare a known input: 5 N along X, zero on the other axes
assert(exist('cfg','var')==1 && exist('ft_wrench_ts','var')==1 && ...
       exist('A6','var')==1 && exist('noise_seed','var')==1, ...
       'Run setup_handguiding_lab.m first.');

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'6D','Simulink'));
addpath(fullfile(root,'6D','FT_Sensor','simulink'));
load_system('FT_Sensor_Lab');
load_system('HandGuiding_Testbench');

t = ft_wrench_ts.Time(:);
N = numel(t);
assert(abs(Ts-0.002)<1e-12 && abs(t(end)-4)<1e-9, ...
    'Expected 500 Hz and a four-second test.');

W = zeros(N,6);                     % [Fx Fy Fz Tx Ty Tz]
push = t>=0.5 & t<1.5;               % One-second push
W(push,1) = 5;                       % Fx = 5 N, Fy = 0 N
wrenchTS = timeseries(W,t);

% A 1.5% X -> Y coupling should produce 0.075 N of FALSE Y force.
C = cfg.sensor.coupling;
assert(isequal(size(C),[6 6]) && C(2,1)~=0, ...
    'Expected the six-axis coupling matrix from ft_lab_config.m.');
expectedExtraFy_N = C(2,1)*5;

%% 2. Build the variables consumed by your existing Simulink blocks
p = struct('cfg',cfg,'M',M,'D',D,'Ts',Ts, ...
    'A6',A6,'B6',B6,'ft_wrench_ts',wrenchTS, ...
    'noise_mean',noise_mean,'noise_variance',noise_variance, ...
    'noise_seed',noise_seed,'bias_est',zeros(6,1), ...
    'alpha',1-exp(-2*pi*20*Ts), ...  % Use 20 Hz EMA throughout
    'bButter',bButter,'aButter',aButter);

% Disable drift in BOTH runs: we're testing coupling, not time-varying bias.
p.cfg.sensor.drift_per_s = zeros(6,1);

% Use the SAME random sequence in sensor-only runs so noise cancels
% when we subtract OFF from ON.
raw = zeros(N,6,2);
V = zeros(N,6,2);
S = find_system('HandGuiding_Testbench','SearchDepth',1, ...
    'BlockType','ManualSwitch');
assert(numel(S)==1, 'Expected one Manual Switch (EMA on lower port).');

%% 3. Run coupling OFF and ON
for k = 1:2
    if k==1
        p.cfg.sensor.coupling = eye(6);  % OFF
    else
        p.cfg.sensor.coupling = C;       % ON
    end

    % Sensor-only run exposes raw data BEFORE tare and filtering.
    sensorOut = sim(runModel('FT_Sensor_Lab',p,t(end)));
    rawTS = sensorOut.get('ft_raw_ts');
    assert(isa(rawTS,'timeseries'), 'Missing raw sensor timeseries.');
    rawCase = squeeze(rawTS.Data);
    if isequal(size(rawCase),[6 N]), rawCase = rawCase'; end
    assert(isequal(size(rawCase),[N 6]), 'Expected N-by-6 raw wrench.');
    raw(:,:,k) = rawCase;

    % Calibrate once using the OFF case during the unloaded interval.
    % The true wrench is zero there, so coupling cannot affect the tare.
    if k==1
        quiet = t>=0.1 & t<0.4;
        tare = mean(raw(quiet,:,1),1)';
    end
    p.bias_est = tare;

    % New (but identical between cases) noise after calibration.
    p.noise_seed = noise_seed + 1000;
    in = runModel('HandGuiding_Testbench',p,t(end));
    in = setBlockParameter(in,S{1},'sw','0'); % Select EMA
    systemOut = sim(in);
    vTS = systemOut.get('twist_integrated_ts');
    assert(isa(vTS,'timeseries') && numel(vTS.Time)==N && ...
        max(abs(vTS.Time(:)-t))<1e-9,'Twist logging mismatch.');
    Vcase = squeeze(vTS.Data);
    assert(isequal(size(Vcase),[N 6]), 'Expected N-by-6 twist.');
    V(:,:,k) = Vcase;

    % Reset to calibration seed for next sensor-only run.
    p.noise_seed = noise_seed;
end

%% 4. Verify wiring, then measure the UNWANTED Y-axis command
extraFy = raw(:,2,2)-raw(:,2,1);
assert(max(abs(extraFy(push)-expectedExtraFy_N)) < 1e-7, ...
    'Coupling did not create expected Fy; check Gain matrix mode (K*u).');
assert(max(abs(extraFy(~push))) < 1e-7, ...
    'Extra Fy outside the push: check noise seeds and wiring.');

extraVy = V(:,2,2)-V(:,2,1);        % m/s, coupling-induced only
extraY = [0; cumsum(extraVy(1:end-1)*Ts)]; % m, ZOH integral

CouplingSummary = table(expectedExtraFy_N, ...
    1000*max(abs(extraVy)),1000*extraY(end), ...
    'VariableNames',{'FalseFy_N','PeakExtraVy_mm_s','ExtraY_mm'});
disp('=== X TO Y CROSS-AXIS COUPLING (20 Hz EMA) ===');
disp(CouplingSummary);

%% 5. See the cause and effect; leave everything inside MATLAB
figure('Name','Cross-axis coupling — X force to Y motion');
tiledlayout(4,1);

nexttile;
plot(t,W(:,1),'LineWidth',1.3);
ylabel('True Fx (N)'); title('Push only in X'); grid on;

nexttile;
plot(t,extraFy,'LineWidth',1.3);
ylabel('False Fy (N)'); title('Coupling-induced Y measurement'); grid on;

nexttile;
plot(t,1000*extraVy,'LineWidth',1.3);
ylabel('Extra Vy (mm/s)'); title('Unwanted Y velocity'); grid on;

nexttile;
plot(t,1000*extraY,'LineWidth',1.3);
xlabel('Time (s)'); ylabel('Extra Y (mm)');
title('Integrated unwanted Y displacement command'); grid on;

%% Helper: change simulation variables without changing saved SLX models
function in = runModel(model,p,stopTime)
    in = Simulink.SimulationInput(model);
    keys = fieldnames(p);
    for i=1:numel(keys)
        in = setVariable(in,keys{i},p.(keys{i}));
    end
    in = setModelParameter(in,'StopTime',num2str(stopTime,17), ...
        'ReturnWorkspaceOutputs','on');
end
