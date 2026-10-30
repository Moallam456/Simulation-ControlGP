
%% Verify Backward Euler Z-Domain Model

clearvars;
clc;
close all;

%% Parameters
m = 10;
b = 100;
Ts = 0.002;
F0 = 10;

%% Difference equation coefficients
alpha = m/(m+b*Ts);
beta = Ts/(m+b*Ts);

%% Discrete transfer function
% Interval-indexed model:
% v[k+1] = alpha*v[k] + beta*u[k]

Gz = tf(beta, [1 -alpha], Ts);

disp('Discrete transfer function:');
Gz

%% Pole and DC gain
z_pole = pole(Gz);
K_dc = dcgain(Gz);

fprintf('Discrete pole: %.8f\n', z_pole);
fprintf('DC gain: %.8f (m/s)/N\n', K_dc);

%% Compare transfer function with difference equation
t = (0:Ts:0.5)';
N = length(t);

v_tf = step(F0*Gz, t);
v_loop = zeros(N,1);

for k = 2:N
    v_loop(k) = alpha*v_loop(k-1) + beta*F0;
end

maxError = max(abs(v_tf(:)-v_loop));

fprintf('Maximum response difference: %.3e m/s\n', ...
    maxError);

%% Verify
assert(abs(z_pole-alpha) < 1e-12);
assert(abs(K_dc-1/b) < 1e-12);
assert(maxError < 1e-12);
