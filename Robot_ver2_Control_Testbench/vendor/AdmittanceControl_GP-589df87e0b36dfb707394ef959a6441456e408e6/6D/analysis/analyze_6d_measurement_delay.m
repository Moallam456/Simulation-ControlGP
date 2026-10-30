
%% Step 41D - Six-Axis Measurement Delay
clearvars;
clc;
close all;

%% 1. Existing controller design

w_design = [10; 8; 6; 0.9; 0.7; 0.5];
V_design = [0.10; 0.08; 0.06; 0.15; 0.12; 0.10];
t90 = [0.30; 0.32; 0.35; 0.28; 0.32; 0.30];

B = w_design ./ V_design;
M = B .* t90 / log(10);

Ts = 0.002;
t = 0:Ts:1.2;
N = numel(t);

%% 2. Existing combined-motion limits

speedMax = [0.13; 0.24];
accMax = [1.10; 1.80];

%% 3. Same force-reversal experiment

W_true = zeros(6,N);

push = t >= 0.05 & t < 0.50;
reverse = t >= 0.50 & t < 0.75;

W_true(:,push) = repmat(w_design,1,nnz(push));
W_true(:,reverse) = repmat(-w_design,1,nnz(reverse));

%% 4. Measurement delays

delay_ms = [0 2 4 10];

delaySamples = round(delay_ms/(Ts*1000));

numCases = numel(delay_ms);

W_received = zeros(6,N,numCases);
V = zeros(6,N,numCases);

%% 5. Simulate Tustin controller with constraints

for c = 1:numCases

    d = delaySamples(c);

    % Delay the measured wrench by d samples
    W_delayed = zeros(6,N);

    if d == 0
        W_delayed = W_true;
    else
        W_delayed(:,d+1:end) = W_true(:,1:end-d);
    end

    W_received(:,:,c) = W_delayed;

    for k = 2:N

        % Tustin admittance update
        raw = ( ...
            (2*M-B*Ts).*V(:,k-1,c) + ...
            Ts*(W_delayed(:,k)+W_delayed(:,k-1)) ...
            ) ./ (2*M+B*Ts);

        % Combined translation and rotation limits
        for group = 1:2

            if group == 1
                idx = 1:3;
            else
                idx = 4:6;
            end

            previous = V(idx,k-1,c);
            target = raw(idx);

            % Speed limiting
            speed = norm(target);

            if speed > speedMax(group)
                target = target*speedMax(group)/speed;
            end

            % Acceleration limiting
            delta = target-previous;
            maxDelta = accMax(group)*Ts;

            if norm(delta) > maxDelta
                delta = delta*maxDelta/norm(delta);
            end

            V(idx,k,c) = previous+delta;

        end
    end
end

%% 6. Compare against zero-delay reference

transError = zeros(numCases,N);
rotError = zeros(numCases,N);

fprintf('\n=== MEASUREMENT DELAY RESULTS ===\n');

for c = 1:numCases

    difference = V(:,:,c)-V(:,:,1);

    transError(c,:) = vecnorm(difference(1:3,:),2,1);
    rotError(c,:) = vecnorm(difference(4:6,:),2,1);

    fprintf('\nDelay: %d ms\n',delay_ms(c));

    fprintf('Maximum translation velocity deviation: %.6f mm/s\n', ...
        max(transError(c,:))*1000);

    fprintf('Maximum rotation velocity deviation: %.6f rad/s\n', ...
        max(rotError(c,:)));

end

%% 7. Verify constraints for every delay

for c = 1:numCases

    for group = 1:2

        if group == 1
            idx = 1:3;
        else
            idx = 4:6;
        end

        speed = vecnorm(V(idx,:,c),2,1);

        acc = [zeros(3,1), ...
            diff(V(idx,:,c),1,2)/Ts];

        acceleration = vecnorm(acc,2,1);

        assert(all(speed <= speedMax(group)+1e-10));
        assert(all(acceleration <= accMax(group)+1e-10));

    end
end

assert(all(isfinite(V(:))));
assert(all(transError(1,:) == 0));
assert(all(rotError(1,:) == 0));

disp('Measurement delay simulation COMPLETED.');

%% 8. Plots

labels = ["0 ms","2 ms","4 ms","10 ms"];

figure;
tiledlayout(2,2);

nexttile;
plot(t,squeeze(W_received(1,:,:)),'LineWidth',1.2);
xlim([0.48 0.54]);
title('Received X Force at Reversal');
xlabel('Time [s]');
ylabel('Force [N]');
legend(labels,'Location','best');
grid on;

nexttile;
plot(t,squeeze(V(1,:,:))*1000,'LineWidth',1.5);
xlim([0.40 0.80]);
title('Commanded X Velocity');
xlabel('Time [s]');
ylabel('Velocity [mm/s]');
legend(labels,'Location','best');
grid on;

nexttile;
plot(t,transError'*1000,'LineWidth',1.5);
title('Translation Deviation from Zero Delay');
xlabel('Time [s]');
ylabel('Velocity deviation [mm/s]');
legend(labels,'Location','best');
grid on;

nexttile;
plot(t,rotError','LineWidth',1.5);
title('Rotation Deviation from Zero Delay');
xlabel('Time [s]');
ylabel('Angular velocity deviation [rad/s]');
legend(labels,'Location','best');
grid on;

sgtitle('Six-Axis Admittance - Measurement Delay Sensitivity');
