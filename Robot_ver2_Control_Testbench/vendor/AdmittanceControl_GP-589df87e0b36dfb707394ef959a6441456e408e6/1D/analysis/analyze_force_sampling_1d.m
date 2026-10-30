
%% Sampling of a Continuously Varying Force
clearvars;
clc;
close all;

%% 1. Sampling parameters
Ts = 0.002;             % Control sampling period [s]
fs = 1/Ts;              % Control sampling frequency [Hz]

samplesPerInterval = 40;
dt = Ts/samplesPerInterval;

tEnd = 0.7;

% Fine time grid for continuous-signal illustration
tFine = (0:dt:tEnd)';

% Actual controller sampling instants
tSample = (0:Ts:tEnd)';

%% 2. Synthetic continuous force

% Smooth force application and release
envelope = @(t) 0.5 * ( ...
    tanh((t-0.10)/0.01) - ...
    tanh((t-0.55)/0.015));

% Nominal push + slow variation + faster vibration
forceSignal = @(t) envelope(t) .* ( ...
    8 + ...
    1.5*sin(2*pi*3*t) + ...
    0.5*sin(2*pi*60*t));

Ftrue = forceSignal(tFine);

%% 3. Sample the force at 500 Hz
Fsample = forceSignal(tSample);

%% 4. Hold the latest available measurement

% Each sample occupies 40 points on our fine time grid
sampleIndex = floor( ...
    (0:numel(tFine)-1)'/samplesPerInterval) + 1;

Fheld = Fsample(sampleIndex);

%% 5. Quantify sampling/hold approximation error
error = Fheld - Ftrue;

maxError = max(abs(error));
rmse = sqrt(mean(error.^2));

fprintf('Sampling frequency: %.0f Hz\n', fs);
fprintf('Sampling period: %.3f ms\n', Ts*1000);
fprintf('Maximum force difference: %.4f N\n', maxError);
fprintf('Force RMSE: %.4f N\n', rmse);

% Verify exact agreement at sampling instants
assert(max(abs( ...
    Fheld(1:samplesPerInterval:end)-Fsample)) < 1e-12);

%% 6. Plot
figure;
tiledlayout(2,1);

nexttile;
plot(tFine,Ftrue,'b-','LineWidth',1.5);
hold on;
stairs(tSample,Fsample,'r-','LineWidth',1);
plot(tSample,Fsample,'k.','MarkerSize',5);

xlabel('Time [s]');
ylabel('Force [N]');
title('Continuous Force and 500 Hz Sampling');
legend('Continuous','Sample-and-hold','Samples');
grid on;

nexttile;
plot(tFine,Ftrue,'b-','LineWidth',1.5);
hold on;
stairs(tSample,Fsample,'r-','LineWidth',1);
plot(tSample,Fsample,'ko','MarkerSize',3);

xlim([0.28 0.32]);

xlabel('Time [s]');
ylabel('Force [N]');
title('Zoom: Behavior Between Samples');
legend('Continuous','Sample-and-hold','Samples');
grid on;


%% 7. Effect of Force Sampling on Admittance Velocity

% Illustrative controller parameters
m = 10;                 % Virtual mass [kg]
b = 100;                % Virtual damping [N*s/m]

%% Continuous-force reference

Gv = admittance_1d_continuous(m, b);

% High-resolution continuous-time response
vTrue = lsim(Gv, Ftrue, tFine);

% Extract response at the controller sampling instants
vTrueSampled = vTrue(1:samplesPerInterval:end);

%% Response to sampled-and-held force

% Exact coefficients for a constant input over each interval
a = exp(-b*Ts/m);
c = (1-a)/b;

vHeld = zeros(size(tSample));

for k = 1:length(tSample)-1

    vHeld(k+1) = a*vHeld(k) + c*Fsample(k);

end

%% Compare responses

velocityError = vHeld - vTrueSampled;

maxVelocityError = max(abs(velocityError));
velocityRMSE = sqrt(mean(velocityError.^2));

fprintf('\nAdmittance response comparison:\n');
fprintf('Maximum velocity error: %.6f mm/s\n', ...
    maxVelocityError*1000);
fprintf('Velocity RMSE: %.6f mm/s\n', ...
    velocityRMSE*1000);

%% Visualize

figure;
tiledlayout(2,1);

nexttile;
plot(tFine, vTrue*1000, 'b-', 'LineWidth', 1.5);
hold on;
plot(tSample, vHeld*1000, 'r--', 'LineWidth', 1.2);

xlabel('Time [s]');
ylabel('Velocity [mm/s]');
title('Admittance Response: Continuous vs Held Force');
legend('True continuous force', 'Sampled-and-held force');
grid on;

nexttile;
plot(tSample, velocityError*1000, 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Velocity error [mm/s]');
title('Velocity Error Due to Force Sampling');
grid on;


%% 8. Compare Discretization Methods With Varying Force

N = length(tSample);

vFE = zeros(N,1);
vBE = zeros(N,1);
vT  = zeros(N,1);
vZOH = zeros(N,1);

%% Simulate the four methods

for k = 2:N

    % Forward Euler: previous force sample
    [~, vFE(k)] = admittance_1d_forward_euler( ...
        0, vFE(k-1), Fsample(k-1), m, b, Ts);

    % Backward Euler: current force sample
    [~, vBE(k)] = admittance_1d_backward_euler( ...
        0, vBE(k-1), Fsample(k), m, b, Ts);

    % Tustin: previous and current force samples
    [~, vT(k)] = admittance_1d_tustin( ...
        0, vT(k-1), Fsample(k-1), Fsample(k), ...
        m, b, Ts);

    % Exact integration of held force
    vZOH(k) = a*vZOH(k-1) + c*Fsample(k-1);

end

%% Compare with continuous-force reference

methods = {'Forward Euler'; 'Backward Euler'; ...
           'Tustin'; 'Exact ZOH'};

responses = [vFE, vBE, vT, vZOH];

errors = responses - vTrueSampled;

maxErrors = max(abs(errors), [], 1)*1000;
rmseErrors = sqrt(mean(errors.^2, 1))*1000;

results = table(methods, maxErrors', rmseErrors', ...
    'VariableNames', ...
    {'Method', 'MaxError_mm_s', 'RMSE_mm_s'});

disp(results);

%% Verify exact ZOH against our previous held-force model

assert(max(abs(vZOH-vHeld)) < 1e-10);

%% Plot velocity errors

figure;
plot(tSample, errors*1000, 'LineWidth', 1.2);

xlabel('Time [s]');
ylabel('Velocity Error [mm/s]');
title('Discretization Methods: Varying Force Input');

legend(methods, 'Location', 'best');
grid on;

%% 9. Isolate Numerical Integration Error
% All methods integrate the SAME ZOH input.

N = length(tSample);

vFE_hold = zeros(N,1);
vBE_hold = zeros(N,1);
vT_hold  = zeros(N,1);

for k = 2:N

    % Force held throughout this interval
    Fk = Fsample(k-1);

    % Forward Euler
    [~, vFE_hold(k)] = admittance_1d_forward_euler( ...
        0, vFE_hold(k-1), Fk, m, b, Ts);

    % Backward Euler
    [~, vBE_hold(k)] = admittance_1d_backward_euler( ...
        0, vBE_hold(k-1), Fk, m, b, Ts);

    % Tustin: both endpoint inputs equal the held force
    [~, vT_hold(k)] = admittance_1d_tustin( ...
        0, vT_hold(k-1), Fk, Fk, m, b, Ts);

end

%% Compare against exact held-input response
% vHeld was calculated in Step 25.

responses = [vFE_hold, vBE_hold, vT_hold];

errors = responses - vHeld;

method = {'Forward Euler'; 'Backward Euler'; 'Tustin'};

maxError = max(abs(errors),[],1)*1000;
rmseError = sqrt(mean(errors.^2,1))*1000;

results = table(method, maxError', rmseError', ...
    'VariableNames', ...
    {'Method','MaxError_mm_s','RMSE_mm_s'});

disp(results);

%% Plot
figure;

plot(tSample, errors*1000, 'LineWidth', 1.3);

xlabel('Time [s]');
ylabel('Velocity Error [mm/s]');

title('Numerical Integration Error — Identical ZOH Input');

legend(method, 'Location', 'best');
grid on;

%% 10. Effect of Measurement Delay

delaySamples = [0 1 2 5];
N = length(tSample);

vDelayed = zeros(N, length(delaySamples));

for j = 1:length(delaySamples)

    d = delaySamples(j);

    for k = 2:N

        % Sample available at the start of this interval
        sampleIdx = k - 1 - d;

        if sampleIdx >= 1
            Finput = Fsample(sampleIdx);
        else
            Finput = 0;
        end

        % Exact held-input integration
        vDelayed(k,j) = a*vDelayed(k-1,j) ...
                      + c*Finput;

    end
end

%% Verify zero-delay baseline
assert(max(abs(vDelayed(:,1)-vHeld)) < 1e-10);

%% Measure additional error caused by delay
delayError = vDelayed - vDelayed(:,1);

maxDelayError = max(abs(delayError),[],1)*1000;

disp('Additional measurement delay effects:');

for j = 1:length(delaySamples)

    fprintf('%2d ms delay: %.5f mm/s max deviation\n', ...
        delaySamples(j)*Ts*1000, maxDelayError(j));

end

%% Plot comparison
figure;
tiledlayout(2,1);

nexttile;
plot(tSample, vDelayed*1000, 'LineWidth', 1.3);

xlabel('Time [s]');
ylabel('Velocity [mm/s]');
title('Admittance Response with Measurement Delay');

legend('0 ms','2 ms','4 ms','10 ms');
grid on;

nexttile;
plot(tSample, delayError*1000, 'LineWidth', 1.3);

xlabel('Time [s]');
ylabel('Velocity difference [mm/s]');
title('Additional Error Relative to Zero Delay');

legend('0 ms','2 ms','4 ms','10 ms');
grid on;