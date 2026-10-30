%% DYNAMIC HAND-GUIDING — ONE CONTROLLED EXPERIMENT
% Run setup_handguiding_lab.m once, then run this file in MATLAB Editor.
%
% Question: When the operator pushes, reverses, and releases the handle,
% how much do EMA filters (10/20/30 Hz) change the X-axis velocity command?
%
% Scope: synthetic sensor + 500 Hz admittance, NOT measured robot motion.
% We excite ONLY Fx to understand one response; other five channels contain
% sensor noise but no intentional wrench. Nothing is saved to disk.

%% 1. Use the existing controller and Simulink models
% These values come from setup_handguiding_lab.m. We do NOT optimize again.
assert(exist('A6','var')==1 && exist('B6','var')==1 && ...
       exist('ft_wrench_ts','var')==1 && exist('cfg','var')==1, ...
       'Run setup_handguiding_lab.m first.');

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'6D','Simulink'));
addpath(fullfile(root,'6D','FT_Sensor','simulink'));

t = ft_wrench_ts.Time(:);  % Existing 0:0.002:4 s experiment clock
N = numel(t);
assert(abs(Ts-0.002)<1e-12 && N==2001, ...
       'This experiment assumes 500 Hz for 4 seconds.');

%% 2. Create ONE understandable operator-force trajectory
% Time (s):   0-0.5    no contact
%             0.5-1.0  gradually push to +5 N
%             1.0-1.4  hold the push
%             1.4-1.9  reverse smoothly to -3 N
%             1.9-2.2  hold the reverse push
%             2.2-2.7  release to zero; then stay unloaded
% Linear interpolation avoids instantaneous jumps in applied force.
timeKnots = [0 0.5 1.0 1.4 1.9 2.2 2.7 4.0];
forceKnots = [0 0 5 5 -3 -3 0 0];  % Newtons, X only
W = zeros(N,6);                   % [Fx Fy Fz Tx Ty Tz]
W(:,1) = interp1(timeKnots,forceKnots,t,'linear');
wrenchTS = timeseries(W,t);

%% 3. Estimate tare (zero-force offset), independently of evaluation noise
% The first 0.5 s is unloaded, so 0.1-0.4 s is our calibration window.
% Use different random seeds for calibration and evaluation; otherwise
% we'd accidentally calibrate away part of the exact same noise sequence.
p = struct('cfg',cfg,'M',M,'D',D,'Ts',Ts, ...
    'A6',A6,'B6',B6,'ft_wrench_ts',wrenchTS, ...
    'noise_mean',noise_mean,'noise_variance',noise_variance, ...
    'noise_seed',noise_seed, ...
    'bias_est',zeros(6,1), ...
    'alpha',alpha,'bButter',bButter,'aButter',aButter);

sensorModel = 'FT_Sensor_Lab';
systemModel = 'HandGuiding_Testbench';
load_system(sensorModel);
load_system(systemModel);

tareRun = sim(modelInput(sensorModel,p,t(end)));
rawTS = tareRun.get('ft_raw_ts');
assert(isa(rawTS,'timeseries'), 'Raw F/T output is missing.');
raw = squeeze(rawTS.Data);              % Before calibration
assert(isequal(size(raw),[N 6]), 'Unexpected raw sensor dimensions.');
quiet = t>=0.1 & t<0.4;
p.bias_est = mean(raw(quiet,:),1)';

% SAME evaluation-noise sequence for every cutoff: fair comparison.
p.noise_seed = noise_seed + 1000;

%% 4. Ideal reference: no measurement error and no sensor filter
% Tustin equation for each axis:
% v[k] = A6*v[k-1] + B6*(w[k] + w[k-1])
% Every term is element-wise; velocity starts at zero.
Videal = zeros(N,6);
a = A6(:)'; b = B6(:)';
for k=2:N
    Videal(k,:) = a.*Videal(k-1,:) + b.*(W(k,:)+W(k-1,:));
end

%% 5. Run the SAME Simulink testbench for three EMA cutoffs
% Manual Switch port 2 is EMA: 'sw'='0' in our tested wiring.
switches = find_system(systemModel,'SearchDepth',1, ...
    'BlockType','ManualSwitch');
assert(numel(switches)==1, 'Expected exactly one Manual Switch.');
cutoffs = [10 20 30];  % Hz — candidates from the earlier sweep
Vema = zeros(N,6,numel(cutoffs));

for j=1:numel(cutoffs)
    % EMA coefficient: faster response when cutoff frequency increases.
    p.alpha = 1-exp(-2*pi*cutoffs(j)*Ts);

    input = modelInput(systemModel,p,t(end));
    input = setBlockParameter(input,switches{1},'sw','0');
    result = sim(input);

    twist = result.get('twist_integrated_ts');
    assert(isa(twist,'timeseries') && ...
        numel(twist.Time)==N && ...
        max(abs(twist.Time(:)-t))<1e-9, ...
        'Velocity logging or sample time changed.');
    Vcase = squeeze(twist.Data);
    assert(isequal(size(Vcase),[N 6]), ...
        'Expected one velocity column per Cartesian axis.');
    Vema(:,:,j) = Vcase;
end

%% 6. Answer two practical questions
% (a) How closely does the velocity follow the IDEAL response during motion?
% (b) After release, how much EXTRA velocity error remains?
% We measure (b) after 3.3 s, giving the nominal admittance time to decay.
% It is not the total velocity: the ideal controller also decays after release.
moving = t>=0.5 & t<=2.7;
afterRelease = t>=3.3;
RMS_TrackingError_mm_s = zeros(3,1);
Peak_TrackingError_mm_s = zeros(3,1);
PostRelease_RMS_Error_mm_s = zeros(3,1);

for j=1:3
    errorX = Vema(:,1,j)-Videal(:,1);
    RMS_TrackingError_mm_s(j) = 1000*sqrt(mean(errorX(moving).^2));
    Peak_TrackingError_mm_s(j) = 1000*max(abs(errorX(moving)));
    PostRelease_RMS_Error_mm_s(j) = ...
        1000*sqrt(mean(errorX(afterRelease).^2));
end

DynamicSummary = table(cutoffs(:), RMS_TrackingError_mm_s, ...
    Peak_TrackingError_mm_s, PostRelease_RMS_Error_mm_s, ...
    'VariableNames',{'EMA_Hz','TrackingRMS_mm_s', ...
                     'TrackingPeak_mm_s','AfterReleaseRMS_mm_s'});
disp('=== PUSH / REVERSE / RELEASE — X AXIS ===');
disp(DynamicSummary);

%% 7. See the physics, not just the numbers
% Top: applied operator force.
% Middle: ideal controller vs filtered controller outputs.
% Bottom: late velocity error after force release (zoomed in time).
figure('Name','Dynamic hand-guiding — EMA comparison');
tiledlayout(3,1);

nexttile;
plot(t,W(:,1),'LineWidth',1.4);
ylabel('Applied Fx (N)'); title('Push, reverse, release'); grid on;

nexttile;
plot(t,1000*Videal(:,1),'LineWidth',1.5); hold on;
for j=1:3
    plot(t,1000*Vema(:,1,j),'--','LineWidth',1.1);
end
ylabel('X velocity (mm/s)'); title('Commanded admittance response');
legend(["Ideal",compose('EMA %g Hz',cutoffs)],'Location','best');
grid on;

nexttile;
for j=1:3
    plot(t(afterRelease),1000*(Vema(afterRelease,1,j)- ...
        Videal(afterRelease,1)),'LineWidth',1.1); hold on;
end
xlabel('Time (s)'); ylabel('Velocity error (mm/s)');
title('Remaining error after release'); grid on;

% Leave figures and DynamicSummary in MATLAB; no automatic files/export.

%% Helper: pass experiment variables without changing saved Simulink models
function in = modelInput(model,p,stopTime)
    in = Simulink.SimulationInput(model);
    names = fieldnames(p);
    for i=1:numel(names)
        in = setVariable(in,names{i},p.(names{i}));
    end
    in = setModelParameter(in,'StopTime',num2str(stopTime,17), ...
        'ReturnWorkspaceOutputs','on');
end
