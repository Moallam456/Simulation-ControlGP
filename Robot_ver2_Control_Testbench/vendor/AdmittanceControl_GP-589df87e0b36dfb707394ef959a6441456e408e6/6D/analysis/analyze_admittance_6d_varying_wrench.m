
%% Step 33 - Six-Axis Time-Varying Wrench
clearvars;
clc;
close all;

%% 1. Controller parameters (illustrative)
M = [10; 12; 8; 0.5; 0.6; 0.4];
B = [100; 90; 120; 5; 4; 6];

Ts = 0.002;         % Controller period [s]
tEnd = 0.85;        % Simulation duration [s]

%% 2. Time vectors

% Fine grid to approximate continuous-time reference
samplesPerInterval = 20;
dt = Ts / samplesPerInterval;

tFine = (0:dt:tEnd)';

% Controller sampling instants
sampleIdx = 1:samplesPerInterval:length(tFine);
tSample = tFine(sampleIdx);

N = length(tSample);

%% 3. Generate time-varying wrench

% Smooth transition function
smooth = @(t,t0) 0.5*(1+tanh((t-t0)/0.012));

% Smooth force/torque pulse
pulse = @(t,tOn,tOff) ...
    smooth(t,tOn) - smooth(t,tOff);

% Translational forces [N]
Fx = 10*pulse(tFine,0.10,0.36) ...
   - 6*pulse(tFine,0.48,0.68);

Fy = -5*pulse(tFine,0.18,0.58);

Fz = 3*pulse(tFine,0.08,0.32) ...
   - 2*pulse(tFine,0.50,0.76);

% Rotational torques [N*m]
Tx = 0.9*pulse(tFine,0.25,0.62);

Ty = -0.55*pulse(tFine,0.16,0.50);

Tz = 0.35*pulse(tFine,0.10,0.32) ...
   - 0.45*pulse(tFine,0.45,0.70);

% Complete wrench matrix: Nfine x 6
Wfine = [Fx Fy Fz Tx Ty Tz];

% Sample at 500 Hz
Wsample = Wfine(sampleIdx,:);

%% 4. Continuous-time reference

sys6 = admittance_6d_continuous(M,B);

VcontFine = lsim(sys6,Wfine,tFine);

% Reference velocity at control sampling instants
Vcont = VcontFine(sampleIdx,:);

%% 5. Six-axis Tustin simulation

VT = zeros(N,6);

for k = 2:N

    VT(k,:) = admittance_6d_tustin( ...
        VT(k-1,:).', ...
        Wsample(k-1,:).', ...
        Wsample(k,:).', ...
        M,B,Ts).';

end

%% 6. Numerical error analysis

error = VT - Vcont;

maxError = max(abs(error),[],1);
rmseError = sqrt(mean(error.^2,1));

% Relative errors based on peak reference magnitude
peakRef = max(abs(Vcont),[],1);
relativeError = 100*maxError./peakRef;

axisName = ["X";"Y";"Z";"Rx";"Ry";"Rz"];
unit = ["m/s";"m/s";"m/s"; ...
        "rad/s";"rad/s";"rad/s"];

results = table( ...
    axisName, maxError.', rmseError.', ...
    relativeError.', unit, ...
    'VariableNames', ...
    {'Axis','MaxError_SI','RMSE_SI', ...
     'RelativeError_percent','Unit'});

disp(results);

%% 7. Verify the Tustin governing equation

residual = zeros(N-1,6);

for k = 2:N

    residual(k-1,:) = ...
        (M.*((VT(k,:)-VT(k-1,:)).'/Ts) ...
        + B.*((VT(k,:)+VT(k-1,:)).'/2) ...
        - ((Wsample(k,:)+Wsample(k-1,:)).'/2)).';

end

maxResidual = max(abs(residual),[],1);

disp('Maximum Tustin equation residual per axis:');
disp(maxResidual);

assert(all(maxResidual < 1e-10));

%% 8. Visualization

figure;
tiledlayout(3,2);

% Input forces
nexttile;
plot(tFine,Wfine(:,1:3),'LineWidth',1.3);
title('Applied Forces');
ylabel('Force [N]');
legend('Fx','Fy','Fz');
grid on;

% Input torques
nexttile;
plot(tFine,Wfine(:,4:6),'LineWidth',1.3);
title('Applied Torques');
ylabel('Torque [N*m]');
legend('Tx','Ty','Tz');
grid on;

% Translational response
nexttile;
plot(tSample,Vcont(:,1:3)*1000,'LineWidth',1.5);
hold on;
plot(tSample,VT(:,1:3)*1000,'--','LineWidth',1);
title('Translational Response');
ylabel('Velocity [mm/s]');
grid on;

% Rotational response
nexttile;
plot(tSample,Vcont(:,4:6),'LineWidth',1.5);
hold on;
plot(tSample,VT(:,4:6),'--','LineWidth',1);
title('Rotational Response');
ylabel('Angular velocity [rad/s]');
grid on;

% Translation errors
nexttile;
plot(tSample,error(:,1:3)*1000,'LineWidth',1.3);
title('Translation Error');
xlabel('Time [s]');
ylabel('Error [mm/s]');
legend('X','Y','Z');
grid on;

% Rotation errors
nexttile;
plot(tSample,error(:,4:6),'LineWidth',1.3);
title('Rotation Error');
xlabel('Time [s]');
ylabel('Error [rad/s]');
legend('Rx','Ry','Rz');
grid on;

sgtitle('Six-Axis Admittance - Varying Wrench');
