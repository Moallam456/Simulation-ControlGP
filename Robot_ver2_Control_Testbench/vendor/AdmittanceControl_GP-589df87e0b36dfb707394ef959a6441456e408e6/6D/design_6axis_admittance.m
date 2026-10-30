%% Six-Axis Admittance: Automatic Parameter Design
% Requires MATLAB Optimization Toolbox (fmincon).
% Design model: M * Vdot + D * V = wrench, with diagonal M and D.
% Values below are illustrative, not verified hardware limits.
% Keep previous workspace variables and figures intact on a repeat run.
% Recompute every design parameter below instead of clearing the session.

%% 1. EDIT ONLY THESE DESIGN REQUIREMENTS
axisNames = ["X"; "Y"; "Z"; "Rx"; "Ry"; "Rz"];
%Force / toruqe the operator is expected to apply
w_design = [10; 8; 6; 0.9; 0.7; 0.5]; % N; N*m for last three
%Desired steady velocity under those forces / torques
V_target = [0.10; 0.08; 0.06; 0.15; 0.12; 0.10]; % m/s; rad/s
% Desired time to reach 90% of steady velocity
rise90_target = [0.30; 0.32; 0.35; 0.28; 0.32; 0.30]; % s

speedMax = [0.13; 0.24]; % combined translation [m/s], rotation [rad/s]
accMax = [1.10; 1.80];   % combined translation [m/s^2], rotation [rad/s^2]
releaseMax = [0.022; 0.035]; % translation [m], angular displacement proxy [rad]

margin = 0.05; % 5% margin below the stated maximum limits
speedWeight = 2; % importance of matching requested steady speeds
riseWeight = 1;  % importance of matching requested 90% rise times
Ts = 0.002;      % s (500 Hz)

%% 2. First-principles initial design

% Checking that values are positive
assert(all(w_design > 0) && all(V_target > 0) && ...
       all(rise90_target > 0), 'Design values must be positive.');
assert(all(speedMax > 0) && all(accMax > 0) && ...
       all(releaseMax > 0) && margin >= 0 && margin < 1);
assert(speedWeight > 0 && riseWeight > 0 && Ts > 0);

D_initial = w_design ./ V_target;
M_initial = D_initial .* rise90_target / log(10);

% Initia; because they will satisfy each axis individually but may violate
% combined motion constraints

%% 3. Automatically optimize M and D, WITHOUT output clipping
% Logarithmic variables enforce positive virtual inertia and damping.
% Minimize squared relative errors in target speeds and rise times.
% Hard constraints: combined speed, reversal acceleration, release travel.

limitsSpeed = (1-margin)*speedMax;
limitsAcc = (1-margin)*accMax;
limitsRelease = (1-margin)*releaseMax;

% Combining the M and D values in 1 vector x0, log is for ensuring positive
% values to opyimize
x0 = log([M_initial; D_initial]);

% @(x) means anonymous function
objective = @(x) tuningCost(x,w_design,V_target, ...
                              rise90_target,speedWeight,riseWeight);
nonlcon = @(x) motionBounds(x,w_design,limitsSpeed, ...
                            limitsAcc,limitsRelease);

opts = optimoptions('fmincon','Algorithm','sqp', ...
    'Display','off','MaxIterations',1500, ...
    'MaxFunctionEvaluations',30000, ...
    'ConstraintTolerance',1e-9, ...
    'OptimalityTolerance',1e-9);

[xOpt,objectiveValue,exitflag,output] = ...
    fmincon(objective,x0,[],[],[],[],[],[],nonlcon,opts);

[c,~] = nonlcon(xOpt);
if exitflag <= 0 || any(c > 1e-7)
    error('Optimizer did not verify a feasible solution: %s',output.message);
end

M = exp(xOpt(1:6));
D = exp(xOpt(7:12));

%% 4. Performance of the optimized design
V_achieved = w_design ./ D;
rise90_achieved = log(10)*M./D;
continuousPoles = -D./M;
discretePoles = (2*M-D*Ts)./(2*M+D*Ts);
assert(all(abs(discretePoles) < 1));
assert(all(discretePoles >= 0), ...
    'Sampling is too coarse for the assumed non-oscillatory Tustin response.');

fprintf('\n=== AUTOMATIC SIX-AXIS ADMITTANCE DESIGN ===\n');
disp(table(axisNames,w_design,V_target,V_achieved, ...
    rise90_target,rise90_achieved,M,D, ...
    'VariableNames',{'Axis','DesignEffort','TargetSpeed', ...
    'AchievedSpeed','TargetT90','AchievedT90','VirtualInertia_M', ...
    'VirtualDamping_D'}));

fprintf('Optimization cost: %.6g\n',objectiveValue);
fprintf('Solver iterations: %d\n',output.iterations);

%% Additional transient-response specifications

tau = M ./ D;

riseTime_10_90 = log(9) * tau;

settlingTime_2percent = -log(0.02) * tau;

overshoot_percent = zeros(6,1); % Ideal first-order velocity step

transientResults = table( ...
    axisNames, tau, riseTime_10_90, ...
    rise90_achieved, settlingTime_2percent, ...
    overshoot_percent, ...
    'VariableNames', { ...
    'Axis', 'TimeConstant', 'RiseTime_10_90', ...
    'TimeTo90Percent', 'SettlingTime_2Percent', ...
    'OvershootPercent'});

disp('=== TRANSIENT RESPONSE SPECIFICATIONS ===');
disp(transientResults);

%% 5. Analytical combined-motion verification
% The factor 2 for acceleration bounds applies to reversal within
% the assumed componentwise wrench envelope (+/- w_design),
% starting from rest in this positive mass-damper model.

vBound = [norm(w_design(1:3)./D(1:3)); ...
          norm(w_design(4:6)./D(4:6))];
aBound = [2*norm(w_design(1:3)./M(1:3)); ...
          2*norm(w_design(4:6)./M(4:6))];
releaseBound = [norm(w_design(1:3).*M(1:3)./D(1:3).^2); ...
                norm(w_design(4:6).*M(4:6)./D(4:6).^2)];

fprintf('\n=== COMBINED CONSTRAINTS (with %.0f%% margin) ===\n',100*margin);
summary = table(["Translation";"Rotation"], ...
    vBound,limitsSpeed,aBound,limitsAcc, ...
    releaseBound,limitsRelease, ...
    'VariableNames',{'Motion','SpeedBound','SpeedLimit', ...
    'AccelerationBound','AccelerationLimit', ...
    'ReleaseBound','ReleaseLimit'});
disp(summary);

assert(all(vBound <= limitsSpeed+1e-9));
assert(all(aBound <= limitsAcc+1e-9));
assert(all(releaseBound <= limitsRelease+1e-9));

%% 6. Verify with the original force-reversal experiment (no clipping)
t = 0:Ts:1.2;
W = zeros(6,numel(t));
pushIdx = t>=0.05 & t<0.50;
reverseIdx = t>=0.50 & t<0.75;
W(:,pushIdx) = repmat(w_design,1,nnz(pushIdx));
W(:,reverseIdx) = repmat(-w_design,1,nnz(reverseIdx));

V_initial = runTustin(M_initial,D_initial,W,Ts);
V_optimized = runTustin(M,D,W,Ts);

A_initial = [zeros(6,1),diff(V_initial,1,2)/Ts];
A_optimized = [zeros(6,1),diff(V_optimized,1,2)/Ts];

speedOrig = [vecnorm(V_initial(1:3,:),2,1); ...
             vecnorm(V_initial(4:6,:),2,1)];
speedOpt  = [vecnorm(V_optimized(1:3,:),2,1); ...
             vecnorm(V_optimized(4:6,:),2,1)];
accOrig = [vecnorm(A_initial(1:3,:),2,1); ...
           vecnorm(A_initial(4:6,:),2,1)];
accOpt  = [vecnorm(A_optimized(1:3,:),2,1); ...
           vecnorm(A_optimized(4:6,:),2,1)];

assert(all(max(speedOpt,[],2) <= limitsSpeed+1e-9));
assert(all(max(accOpt,[],2) <= limitsAcc+1e-9));

fprintf('\n=== FORCE-REVERSAL SIMULATION (NO SATURATION) ===\n');
fprintf('Peak translation speed: %.4f -> %.4f m/s\n', ...
    max(speedOrig(1,:)),max(speedOpt(1,:)));
fprintf('Peak rotation speed: %.4f -> %.4f rad/s\n', ...
    max(speedOrig(2,:)),max(speedOpt(2,:)));
fprintf('Peak translation acceleration: %.4f -> %.4f m/s^2\n', ...
    max(accOrig(1,:)),max(accOpt(1,:)));
fprintf('Peak rotation acceleration: %.4f -> %.4f rad/s^2\n', ...
    max(accOrig(2,:)),max(accOpt(2,:)));

figure;
tiledlayout(2,2);
for g=1:2
    nexttile(g);
    plot(t,speedOrig(g,:),'LineWidth',1.4); hold on;
    plot(t,speedOpt(g,:),'--','LineWidth',1.4);
    yline(limitsSpeed(g),':','Limit with margin'); grid on;
    xlabel('Time [s]');
    if g==1
        title('Combined Translation Speed'); ylabel('m/s');
    else
        title('Combined Rotation Speed'); ylabel('rad/s');
    end
    legend('Initial design','Optimized','Constraint');

    nexttile(g+2);
    plot(t,accOrig(g,:),'LineWidth',1.4); hold on;
    plot(t,accOpt(g,:),'--','LineWidth',1.4);
    yline(limitsAcc(g),':','Limit with margin'); grid on;
    xlabel('Time [s]');
    if g==1
        title('Combined Translation Acceleration'); ylabel('m/s^2');
    else
        title('Combined Rotation Acceleration'); ylabel('rad/s^2');
    end
    legend('Initial design','Optimized','Constraint');
end
sgtitle('Automatic Six-Axis Admittance Parameter Design');

disp('OPTIMIZATION AND MODEL-LEVEL CONSTRAINT CHECKS PASSED.');
disp('This does not validate motor dynamics or physical robot safety.');

%% Local functions
function J = tuningCost(x,F,vTarget,tTarget,wV,wT)
    m = exp(x(1:6));
    d = exp(x(7:12));
    v = F./d;
    t90 = log(10)*m./d;
    J = wV*sum(log(v./vTarget).^2) + ...
        wT*sum(log(t90./tTarget).^2);
end

function [c,ceq] = motionBounds(x,F,vLim,aLim,rLim)
    m = exp(x(1:6));
    d = exp(x(7:12));
    speed = [norm(F(1:3)./d(1:3)); norm(F(4:6)./d(4:6))];
    accel = [2*norm(F(1:3)./m(1:3)); ...
             2*norm(F(4:6)./m(4:6))];
    release = [norm(F(1:3).*m(1:3)./d(1:3).^2); ...
               norm(F(4:6).*m(4:6)./d(4:6).^2)];
    c = [speed./vLim-1; accel./aLim-1; release./rLim-1];
    ceq = [];
end

function V = runTustin(m,d,W,Ts)
    V = zeros(size(W));
    for k=2:size(W,2)
        V(:,k) = ((2*m-d*Ts).*V(:,k-1) + ...
            Ts*(W(:,k)+W(:,k-1)))./(2*m+d*Ts);
    end
end
