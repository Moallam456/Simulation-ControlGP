
%% Step 40 - Six-Axis Admittance Frequency Response
clearvars;
clc;
close all;

%% 1. Design requirements from Step 39

axisName = ["X"; "Y"; "Z"; "Rx"; "Ry"; "Rz"];

w_design = [10; 8; 6; 0.9; 0.7; 0.5];

V_design = [0.10; 0.08; 0.06; ...
            0.15; 0.12; 0.10];

t90_design = [0.30; 0.32; 0.35; ...
              0.28; 0.32; 0.30];

%% 2. Derive virtual inertia and damping

B = w_design ./ V_design;

M = B .* t90_design / log(10);

%% 3. Calculate frequency-domain properties

tau = M ./ B;

poles = -B ./ M;

omega_c = B ./ M;       % Corner frequency [rad/s]

f_c = omega_c/(2*pi);   % Corner frequency [Hz]

DC_gain = 1 ./ B;

%% 4. Display theoretical results

results = table( ...
    axisName, M, B, tau, poles, omega_c, f_c, DC_gain, ...
    'VariableNames', { ...
    'Axis','VirtualInertia','Damping', ...
    'TimeConstant_s','Pole_rad_s', ...
    'Corner_rad_s','Corner_Hz','DCGain'});

disp('=== SIX-AXIS FREQUENCY-DOMAIN PROPERTIES ===');
disp(results);

%% 5. Frequency vector

f = logspace(-2,2,600);   % 0.01 Hz to 100 Hz

omega = 2*pi*f;

%% 6. Analytical frequency response

H = zeros(6,length(f));

for i = 1:6

    H(i,:) = 1 ./ ...
        (B(i) + 1j*M(i)*omega);

end

%% 7. Normalize magnitude by DC gain

% Each axis has different physical units.
% Normalization allows shape comparison.

H_normalized = abs(H) .* B;

magnitude_dB = 20*log10(H_normalized);

phase_deg = rad2deg(angle(H));

%% 8. Verify -3 dB corner and -45 degree phase

magCorner_dB = zeros(6,1);
phaseCorner_deg = zeros(6,1);

for i = 1:6

    H_corner = 1 / ...
        (B(i) + 1j*M(i)*omega_c(i));

    magCorner_dB(i) = ...
        20*log10(abs(H_corner)*B(i));

    phaseCorner_deg(i) = ...
        rad2deg(angle(H_corner));

end

cornerResults = table( ...
    axisName, f_c, magCorner_dB, phaseCorner_deg, ...
    'VariableNames', { ...
    'Axis','Corner_Hz','Magnitude_dB','Phase_deg'});

disp('=== CORNER FREQUENCY VERIFICATION ===');
disp(cornerResults);

assert(all(abs(magCorner_dB + ...
    20*log10(sqrt(2))) < 1e-10));

assert(all(abs(phaseCorner_deg + 45) < 1e-10));

%% 9. Cross-check against MATLAB state-space model

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot,'models'));

sys6 = admittance_6d_continuous(M,B);

modelError = zeros(6,1);

for i = 1:6

    G = freqresp(sys6,omega_c(i));

    H_analytical = 1 / ...
        (B(i) + 1j*M(i)*omega_c(i));

    modelError(i) = abs(G(i,i)-H_analytical);

end

assert(all(modelError < 1e-12));

disp('Analytical and state-space frequency responses agree.');

%% 10. Plot normalized Bode responses

figure;
tiledlayout(2,2);

% Translational magnitude
nexttile;
semilogx(f,magnitude_dB(1:3,:).','LineWidth',1.5);
hold on;
yline(-3.0103,'k--','-3 dB');
title('Translational Admittance - Magnitude');
ylabel('Normalized magnitude [dB]');
legend('X','Y','Z','Location','southwest');
grid on;

% Rotational magnitude
nexttile;
semilogx(f,magnitude_dB(4:6,:).','LineWidth',1.5);
hold on;
yline(-3.0103,'k--','-3 dB');
title('Rotational Admittance - Magnitude');
ylabel('Normalized magnitude [dB]');
legend('Rx','Ry','Rz','Location','southwest');
grid on;

% Translational phase
nexttile;
semilogx(f,phase_deg(1:3,:).','LineWidth',1.5);
hold on;
yline(-45,'k--','-45 deg');
title('Translational Admittance - Phase');
xlabel('Frequency [Hz]');
ylabel('Phase [deg]');
legend('X','Y','Z','Location','southwest');
grid on;

% Rotational phase
nexttile;
semilogx(f,phase_deg(4:6,:).','LineWidth',1.5);
hold on;
yline(-45,'k--','-45 deg');
title('Rotational Admittance - Phase');
xlabel('Frequency [Hz]');
ylabel('Phase [deg]');
legend('Rx','Ry','Rz','Location','southwest');
grid on;

sgtitle('Six-Axis Virtual Admittance Frequency Response');

%% 11. Verification status

disp('Six-axis frequency-domain analysis PASSED.');
