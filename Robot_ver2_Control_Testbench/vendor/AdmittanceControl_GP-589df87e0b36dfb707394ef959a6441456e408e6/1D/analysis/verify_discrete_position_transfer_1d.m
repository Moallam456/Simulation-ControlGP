
%% Verify Discrete Force-to-Position Transfer Function

clearvars;
clc;
close all;

%% Parameters
m = 10;
b = 100;
Ts = 0.002;
F0 = 10;

alpha = m/(m+b*Ts);
beta = Ts/(m+b*Ts);

%% 1. Transfer function from our state-space model

sys = admittance_1d_discrete_state_space(m,b,Ts);

Gx_ss = tf(sys(1,1));

%% 2. Independently derived transfer function

numerator = Ts*beta*[1 0];

denominator = conv([1 -1],[1 -alpha]);

Gx_derived = tf(numerator,denominator,Ts);

disp('Derived force-to-position transfer function:');
Gx_derived

%% 3. Compare their step responses

t = (0:Ts:0.5)';

x_ss = step(F0*Gx_ss,t);
x_derived = step(F0*Gx_derived,t);

maxError = max(abs(x_ss-x_derived));

fprintf('Maximum position difference: %.3e m\n',maxError);

assert(maxError < 1e-10);

%% 4. Verify poles

disp('Force-to-position poles:');
disp(pole(Gx_derived));

%% 5. Plot
figure;

plot(t,x_ss*1000,'b-','LineWidth',2);
hold on;
plot(t,x_derived*1000,'r--','LineWidth',1.5);

xlabel('Time [s]');
ylabel('Commanded displacement [mm]');
title('Discrete Force-to-Position Model Verification');

legend('State-space','Derived transfer function');
grid on;
