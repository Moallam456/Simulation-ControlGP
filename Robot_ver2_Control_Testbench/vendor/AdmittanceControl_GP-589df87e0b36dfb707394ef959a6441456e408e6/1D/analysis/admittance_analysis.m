clear; clc; close all;

M = 2;
D = 20;

s = tf('s');

G = 1 / (M*s + D);

pole(G)
zero(G)

figure;
step(G, 0:0.002:0.5);
grid on;
title('Admittance Control - Velocity Step Response');
ylabel('Velocity (m/s)');

%% Effect of changing M 

M_values = [0.5, 1, 2, 4, 8];
t = 0:0.002:2;

figure;
hold on;

for i = 1:length(M_values)
    M_i = M_values(i);

    G_i = 1 / (M_i*s + D);

    v = step(G_i, t);

    plot(t, v, 'DisplayName', sprintf('M = %.1f kg', M_i));
end

grid on;
legend('Location', 'southeast');
title('Effect of Virtual Mass on Admittance');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

%% Numerical Discretization Comparison

M = 2;
D = 20;

Ts_values = [0.002, 0.05, 0.25];

figure;
tiledlayout(1, 3);

for j = 1:length(Ts_values)

    Ts = Ts_values(j);
    t = 0:Ts:1.5;

    F = 10 * (t < 0.5);

    v_FE = zeros(size(t));
    v_BE = zeros(size(t));
    v_ZOH = zeros(size(t));

    a = exp(-D*Ts/M);

    for k = 1:length(t)-1

        v_FE(k+1) = v_FE(k) + ...
            Ts*(F(k)-D*v_FE(k))/M;

        v_BE(k+1) = ...
            (M*v_BE(k)+Ts*F(k))/(M+D*Ts);

        v_ZOH(k+1) = ...
            a*v_ZOH(k)+(1-a)*F(k)/D;
    end

    nexttile;
    plot(t,v_FE,'-o',t,v_BE,'-s',t,v_ZOH,'-');
    grid on;
    title(sprintf('Ts = %.0f ms',Ts*1000));
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('Forward','Backward','Exact ZOH');
end