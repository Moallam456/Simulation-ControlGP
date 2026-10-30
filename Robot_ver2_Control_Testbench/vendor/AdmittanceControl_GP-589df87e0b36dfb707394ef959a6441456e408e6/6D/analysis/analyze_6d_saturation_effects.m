
%% Step 41E - Saturation Effects on Admittance
clearvars;
clc;
close all;

%% 1. Existing six-axis design

w_design = [10;8;6;0.9;0.7;0.5];
V_design = [0.10;0.08;0.06;0.15;0.12;0.10];
t90 = [0.30;0.32;0.35;0.28;0.32;0.30];

B = w_design ./ V_design;
M = B .* t90 / log(10);

Ts = 0.002;
t = 0:Ts:1.8;
N = numel(t);

speedMax = [0.13;0.24];
accMax = [1.10;1.80];

%% 2. Two force scenarios

scales = [1.0,0.30];
caseNames = ["Full force","Gentle force"];

W = zeros(6,N,2);

push = t>=0.05 & t<0.50;
reverse = t>=0.50 & t<0.75;

for c = 1:2
    W(:,push,c) = scales(c)* ...
        repmat(w_design,1,nnz(push));

    W(:,reverse,c) = -scales(c)* ...
        repmat(w_design,1,nnz(reverse));
end

%% 3. Simulate both controllers

Videal = zeros(6,N,2);
Vcmd = zeros(6,N,2);

Xideal = zeros(3,N,2);
Xcmd = zeros(3,N,2);

speedActive = false(2,N,2);
accActive = false(2,N,2);

for c = 1:2
    for k = 2:N

        % Unconstrained Tustin
        Videal(:,k,c) = ( ...
            (2*M-B*Ts).*Videal(:,k-1,c) + ...
            Ts*(W(:,k,c)+W(:,k-1,c)) ...
            ) ./ (2*M+B*Ts);

        % Tustin with constrained previous state
        raw = ( ...
            (2*M-B*Ts).*Vcmd(:,k-1,c) + ...
            Ts*(W(:,k,c)+W(:,k-1,c)) ...
            ) ./ (2*M+B*Ts);

        for g = 1:2
            if g==1
                idx = 1:3;
            else
                idx = 4:6;
            end

            previous = Vcmd(idx,k-1,c);
            target = raw(idx);

            % Combined speed limit
            s = norm(target);

            if s > speedMax(g)
                target = target*speedMax(g)/s;
                speedActive(g,k,c) = true;
            end

            % Combined acceleration limit
            dv = target-previous;
            dvNorm = norm(dv);
            maxDv = accMax(g)*Ts;

            if dvNorm > maxDv
                dv = dv*maxDv/dvNorm;
                accActive(g,k,c) = true;
            end

            Vcmd(idx,k,c) = previous+dv;
        end

        % Integrate Cartesian translation
        Xideal(:,k,c) = Xideal(:,k-1,c) + ...
            Ts/2*(Videal(1:3,k-1,c)+ ...
                  Videal(1:3,k,c));

        Xcmd(:,k,c) = Xcmd(:,k-1,c) + ...
            Ts/2*(Vcmd(1:3,k-1,c)+ ...
                  Vcmd(1:3,k,c));
    end
end

%% 4. Verify limits and calculate tracking errors

fprintf('\n=== SATURATION IMPACT ===\n');

for c = 1:2
    fprintf('\n%s\n',caseNames(c));

    for g = 1:2
        if g==1
            idx=1:3;
        else
            idx=4:6;
        end

        vel = Vcmd(idx,:,c);
        accel = [zeros(3,1),diff(vel,1,2)/Ts];

        assert(all(vecnorm(vel,2,1) <= ...
            speedMax(g)+1e-10));

        assert(all(vecnorm(accel,2,1) <= ...
            accMax(g)+1e-10));

        fprintf('Group %d speed-limited cycles: %d\n', ...
            g,nnz(speedActive(g,:,c)));

        fprintf('Group %d accel-limited cycles: %d\n', ...
            g,nnz(accActive(g,:,c)));
    end

    eTrans = vecnorm( ...
        Vcmd(1:3,:,c)-Videal(1:3,:,c),2,1);

    eRot = vecnorm( ...
        Vcmd(4:6,:,c)-Videal(4:6,:,c),2,1);

    ePosition = vecnorm( ...
        Xcmd(:,:,c)-Xideal(:,:,c),2,1);

    fprintf('Maximum translational velocity error: %.4f mm/s\n', ...
        1000*max(eTrans));

    fprintf('Maximum angular velocity error: %.6f rad/s\n', ...
        max(eRot));

    fprintf('Final Cartesian position error: %.4f mm\n', ...
        1000*ePosition(end));
end

% Gentle force must not activate constraints
assert(~any(speedActive(:,:,2),'all'));
assert(~any(accActive(:,:,2),'all'));

assert(max(abs(Vcmd(:,:,2)-Videal(:,:,2)),[],'all') < 1e-12);

fprintf('\nGentle-force linear-response check PASSED.\n');

%% 5. Plot full-force experiment

c=1;

speedIdeal=vecnorm(Videal(1:3,:,c),2,1);
speedCmd=vecnorm(Vcmd(1:3,:,c),2,1);

aIdeal=[zeros(3,1),diff(Videal(1:3,:,c),1,2)/Ts];
aCmd=[zeros(3,1),diff(Vcmd(1:3,:,c),1,2)/Ts];

errVel=vecnorm(Vcmd(1:3,:,c)-Videal(1:3,:,c),2,1);

errPos=vecnorm(Xcmd(:,:,c)-Xideal(:,:,c),2,1);

figure;
tiledlayout(2,2);

nexttile;
plot(t,speedIdeal,'LineWidth',1.5);
hold on;
plot(t,speedCmd,'--','LineWidth',1.5);
yline(speedMax(1),'r:');
title('Translational Speed');
ylabel('Speed [m/s]');
legend('Ideal','Limited','Limit');
grid on;

nexttile;
plot(t,vecnorm(aIdeal,2,1),'LineWidth',1.5);
hold on;
plot(t,vecnorm(aCmd,2,1),'--','LineWidth',1.5);
yline(accMax(1),'r:');
title('Translational Acceleration');
ylabel('Acceleration [m/s^2]');
legend('Ideal','Limited','Limit');
grid on;

nexttile;
plot(t,errVel*1000,'LineWidth',1.5);
title('Velocity Deviation Caused by Saturation');
xlabel('Time [s]');
ylabel('Deviation [mm/s]');
grid on;

nexttile;
plot(t,errPos*1000,'LineWidth',1.5);
title('Accumulated Cartesian Position Error');
xlabel('Time [s]');
ylabel('Position error [mm]');
grid on;

sgtitle('Effect of Saturation on Virtual Admittance');

disp('Saturation impact analysis COMPLETED.');
