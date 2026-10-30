
%% Step 41C - Six-Axis Sensor Disturbance Analysis
clearvars;
clc;
close all;

%% 1. Existing controller parameters

w_design = [10; 8; 6; 0.9; 0.7; 0.5];
V_design = [0.10; 0.08; 0.06; 0.15; 0.12; 0.10];
t90 = [0.30; 0.32; 0.35; 0.28; 0.32; 0.30];

B = w_design ./ V_design;
M = B .* t90 / log(10);

Ts = 0.002;
t = 0:Ts:10;
N = numel(t);

%% 2. Same combined constraints from Step 41B

speedMax = [0.13; 0.24];
accMax = [1.10; 1.80];

%% 3. Sensor disturbance scenarios

rng(17); % Reproducible noise

biasN = 0.2;
noiseStdN = 0.05;

% Dimensions: 6 axes x N samples x 4 scenarios
W = zeros(6,N,4);

% Scenario 1: ideal zero input
% Scenario 2: constant bias on X
W(1,:,2) = biasN;

% Scenario 3: zero-mean random noise on X
W(1,:,3) = noiseStdN*randn(1,N);

% Scenario 4: bias plus the same noise
W(1,:,4) = W(1,:,2) + W(1,:,3);

%% 4. Six-axis Tustin + combined motion limits

V = zeros(6,N,4);
X = zeros(6,N,4);

for scenario = 1:4

    for k = 2:N

        % Tustin admittance update
        raw = ( ...
            (2*M-B*Ts).*V(:,k-1,scenario) + ...
            Ts*(W(:,k,scenario)+W(:,k-1,scenario)) ...
            ) ./ (2*M+B*Ts);

        % Translation and rotation constraints
        for group = 1:2

            if group == 1
                idx = 1:3;
            else
                idx = 4:6;
            end

            previous = V(idx,k-1,scenario);
            target = raw(idx);

            % Combined velocity limit
            speed = norm(target);
            if speed > speedMax(group)
                target = target*speedMax(group)/speed;
            end

            % Combined acceleration limit
            delta = target - previous;
            deltaNorm = norm(delta);
            maxDelta = accMax(group)*Ts;

            if deltaNorm > maxDelta
                delta = delta*maxDelta/deltaNorm;
            end

            V(idx,k,scenario) = previous + delta;

        end

        % Trapezoidal position integration
        X(:,k,scenario) = X(:,k-1,scenario) + ...
            (Ts/2)*(V(:,k-1,scenario)+V(:,k,scenario));

    end

end

%% 5. Numerical results

names = ["Ideal"; "Bias"; "Noise"; "Bias + Noise"];

fprintf('\n=== SENSOR DISTURBANCE ANALYSIS ===\n');

for scenario = 1:4

    vx = squeeze(V(1,:,scenario));
    xpos = squeeze(X(1,:,scenario));

    fprintf('\n%s\n',names(scenario));
    fprintf('Final X velocity: %.6f mm/s\n',vx(end)*1000);
    fprintf('Maximum |X velocity|: %.6f mm/s\n', ...
        max(abs(vx))*1000);
    fprintf('Final X displacement: %.6f mm\n', ...
        xpos(end)*1000);

end

%% 6. Model consistency checks

assert(all(isfinite(V(:))));
assert(all(isfinite(X(:))));

% Ideal zero input must produce zero motion
assert(all(V(:,:,1) == 0,'all'));

% Constant bias should approach F_bias / B
assert(abs(V(1,end,2)-biasN/B(1)) < 1e-8);

disp('Disturbance simulation COMPLETED.');

%% 7. Plot results

figure;
tiledlayout(3,1);

nexttile;
plot(t,squeeze(W(1,:,:)),'LineWidth',1);
title('Measured X Force');
ylabel('Force [N]');
legend(names,'Location','best');
grid on;

nexttile;
plot(t,squeeze(V(1,:,:))*1000,'LineWidth',1.3);
title('Commanded X Velocity');
ylabel('Velocity [mm/s]');
legend(names,'Location','best');
grid on;

nexttile;
plot(t,squeeze(X(1,:,:))*1000,'LineWidth',1.3);
title('Accumulated X Position');
xlabel('Time [s]');
ylabel('Displacement [mm]');
legend(names,'Location','best');
grid on;

sgtitle('Six-Axis Admittance - Sensor Bias and Noise');
