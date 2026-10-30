
%% Step 41B - Six-Axis Motion Saturation
clearvars;
clc;
close all;

%% 1. Existing virtual admittance parameters

w_design = [10; 8; 6; 0.9; 0.7; 0.5];
V_design = [0.10; 0.08; 0.06; 0.15; 0.12; 0.10];
t90 = [0.30; 0.32; 0.35; 0.28; 0.32; 0.30];

B = w_design ./ V_design;
M = B .* t90 / log(10);

Ts = 0.002;
t = 0:Ts:1.2;
N = numel(t);

%% 2. Illustrative combined-motion limits

speedMax = [0.13; 0.24];  % m/s, rad/s
accMax = [1.10; 1.80];    % m/s^2, rad/s^2

%% 3. Same force reversal as Step 41A

W = zeros(6,N);

push = t >= 0.05 & t < 0.50;
reverse = t >= 0.50 & t < 0.75;

W(:,push) = repmat(w_design,1,nnz(push));
W(:,reverse) = repmat(-w_design,1,nnz(reverse));

%% 4. Compare unconstrained and constrained Tustin

V_ideal = zeros(6,N);
V_limited = zeros(6,N);

speedActive = false(2,N);
accActive = false(2,N);

for k = 2:N

    % Original unconstrained Tustin
    V_ideal(:,k) = ( ...
        (2*M-B*Ts).*V_ideal(:,k-1) + ...
        Ts*(W(:,k)+W(:,k-1)) ...
        ) ./ (2*M+B*Ts);

    % Tustin update using previous constrained state
    raw = ( ...
        (2*M-B*Ts).*V_limited(:,k-1) + ...
        Ts*(W(:,k)+W(:,k-1)) ...
        ) ./ (2*M+B*Ts);

    % Independently limit translation and rotation
    for group = 1:2

        if group == 1
            idx = 1:3;
        else
            idx = 4:6;
        end

        previous = V_limited(idx,k-1);
        target = raw(idx);

        % Combined speed constraint
        targetNorm = norm(target);

        if targetNorm > speedMax(group)
            target = target * ...
                (speedMax(group)/targetNorm);
            speedActive(group,k) = true;
        end

        % Combined acceleration constraint
        delta = target - previous;
        deltaNorm = norm(delta);

        maxDelta = accMax(group)*Ts;

        if deltaNorm > maxDelta
            delta = delta*(maxDelta/deltaNorm);
            accActive(group,k) = true;
        end

        V_limited(idx,k) = previous + delta;

    end
end

%% 5. Calculate combined speeds and accelerations

S_ideal = [vecnorm(V_ideal(1:3,:),2,1);
           vecnorm(V_ideal(4:6,:),2,1)];

S_limited = [vecnorm(V_limited(1:3,:),2,1);
             vecnorm(V_limited(4:6,:),2,1)];

A_ideal = [zeros(6,1),diff(V_ideal,1,2)/Ts];

A_limited = [zeros(6,1),diff(V_limited,1,2)/Ts];

Acc_ideal = [vecnorm(A_ideal(1:3,:),2,1);
             vecnorm(A_ideal(4:6,:),2,1)];

Acc_limited = [vecnorm(A_limited(1:3,:),2,1);
               vecnorm(A_limited(4:6,:),2,1)];

%% 6. Verify constraints

for group = 1:2

    assert(all(S_limited(group,:) <= ...
        speedMax(group) + 1e-10));

    assert(all(Acc_limited(group,:) <= ...
        accMax(group) + 1e-10));

end

assert(all(isfinite(V_limited(:))));

%% 7. Display results

labels = ["Translation";"Rotation"];

for group = 1:2

    fprintf('\n=== %s ===\n',labels(group));

    fprintf('Speed: %.5f -> %.5f | Limit %.5f\n', ...
        max(S_ideal(group,:)), ...
        max(S_limited(group,:)),speedMax(group));

    fprintf('Acceleration: %.5f -> %.5f | Limit %.5f\n', ...
        max(Acc_ideal(group,:)), ...
        max(Acc_limited(group,:)),accMax(group));

    fprintf('Speed-limited cycles: %d\n', ...
        nnz(speedActive(group,:)));

    fprintf('Acceleration-limited cycles: %d\n', ...
        nnz(accActive(group,:)));

end

%% 8. Plot comparison

figure;
tiledlayout(2,2);

for group = 1:2

    nexttile(group);

    plot(t,S_ideal(group,:),'LineWidth',1.5);
    hold on;
    plot(t,S_limited(group,:),'--','LineWidth',1.5);
    yline(speedMax(group),'r:','Limit');

    title(labels(group) + " Speed");
    xlabel('Time [s]');
    if group == 1
        ylabel('Speed [m/s]');
    else
        ylabel('Angular speed [rad/s]');
    end
    legend('Unconstrained','Constrained','Limit');
    grid on;

    nexttile(group+2);

    plot(t,Acc_ideal(group,:),'LineWidth',1.5);
    hold on;
    plot(t,Acc_limited(group,:),'--','LineWidth',1.5);
    yline(accMax(group),'r:','Limit');

    title(labels(group) + " Acceleration");
    xlabel('Time [s]');
    if group == 1
        ylabel('Acceleration [m/s^2]');
    else
        ylabel('Angular acceleration [rad/s^2]');
    end
    legend('Unconstrained','Constrained','Limit');
    grid on;

end

sgtitle('Six-Axis Admittance: Motion Constraint Verification');

disp('Combined motion constraint verification PASSED.');
