
%% Comparison of Discretization Methods
clearvars;
clc;
close all;

%% Parameters
m = 10;
b = 100;
Ts = 0.002;
F0 = 10;

t = (0:Ts:0.5)';
N = length(t);

tau = m/b;
v_ss = F0/b;

%% Exact continuous-time reference
v_exact = v_ss*(1-exp(-t/tau));

x_exact = v_ss*(t-tau*(1-exp(-t/tau)));

%% Initialize discrete states
v_FE = zeros(N,1);
v_BE = zeros(N,1);
v_T  = zeros(N,1);

x_FE = zeros(N,1);
x_BE = zeros(N,1);
x_T  = zeros(N,1);

%% Simulate each method
for k = 2:N

    % Forward Euler
    [x_FE(k), v_FE(k)] = admittance_1d_forward_euler( ...
        x_FE(k-1), v_FE(k-1), F0, m, b, Ts);

    % Backward Euler
    [x_BE(k), v_BE(k)] = admittance_1d_backward_euler( ...
        x_BE(k-1), v_BE(k-1), F0, m, b, Ts);

    % Tustin
    [x_T(k), v_T(k)] = admittance_1d_tustin( ...
        x_T(k-1), v_T(k-1), F0, F0, m, b, Ts);

end

%% Maximum absolute errors
velErrors = [
    max(abs(v_FE-v_exact));
    max(abs(v_BE-v_exact));
    max(abs(v_T-v_exact))
];

posErrors = [
    max(abs(x_FE-x_exact));
    max(abs(x_BE-x_exact));
    max(abs(x_T-x_exact))
];

%% Display results
method = {'Forward Euler'; 'Backward Euler'; 'Tustin'};

results = table( ...
    method, ...
    velErrors/v_ss*100, ...
    posErrors*1000, ...
    'VariableNames', ...
    {'Method','VelocityError_percent','PositionError_mm'});

disp(results);

%% Plot comparisons
figure;
tiledlayout(3,1);

% Velocity
nexttile;
plot(t,v_exact,'k-','LineWidth',2);
hold on;
plot(t,v_FE,'--','LineWidth',1.3);
plot(t,v_BE,'--','LineWidth',1.3);
plot(t,v_T,'--','LineWidth',1.3);
ylabel('Velocity [m/s]');
title('Velocity Response');
legend('Exact','Forward','Backward','Tustin');
grid on;

% Displacement
nexttile;
plot(t,x_exact*1000,'k-','LineWidth',2);
hold on;
plot(t,x_FE*1000,'--','LineWidth',1.3);
plot(t,x_BE*1000,'--','LineWidth',1.3);
plot(t,x_T*1000,'--','LineWidth',1.3);
ylabel('Displacement [mm]');
title('Displacement Response');
grid on;

% Signed velocity error
nexttile;
plot(t,(v_FE-v_exact)*1000,'LineWidth',1.3);
hold on;
plot(t,(v_BE-v_exact)*1000,'LineWidth',1.3);
plot(t,(v_T-v_exact)*1000,'LineWidth',1.3);
xlabel('Time [s]');
ylabel('Velocity Error [mm/s]');
title('Velocity Approximation Error');
legend('Forward','Backward','Tustin');
grid on;
