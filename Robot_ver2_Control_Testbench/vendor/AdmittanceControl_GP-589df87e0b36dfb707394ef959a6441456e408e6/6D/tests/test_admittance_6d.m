%% Test: Simultaneous Six-Axis Admittance

clearvars;
clc;

%% Illustrative controller parameters
Ts = 0.002;

M = [10; 12; 8; 0.5; 0.6; 0.4];

B = [100; 90; 120; 5; 4; 6];

%% Initial Cartesian twist
Vprev = zeros(6,1);

%% Simultaneous applied wrench
% First 3: forces [N]
% Last 3: torques [N*m]

wrench = [10; -6; 4; 1; -0.5; 0.2];

%% Run six-axis controller
Vnext = admittance_6d_backward_euler( ...
    Vprev, wrench, M, B, Ts);

disp('Updated Cartesian twist:');
disp(Vnext);

%% Verify each axis independently
for i = 1:6

    expected = (M(i)*Vprev(i) + Ts*wrench(i)) ...
             / (M(i) + Ts*B(i));

    assert(abs(Vnext(i)-expected) < 1e-12);

end

%% Verify the discrete governing equation
residual = M.*((Vnext-Vprev)/Ts) ...
         + B.*Vnext - wrench;

disp('Force/torque balance residuals:');
disp(residual);

assert(all(abs(residual) < 1e-12));

%% Verify axis independence
modifiedWrench = wrench;
modifiedWrench(1) = 20;

Vmodified = admittance_6d_backward_euler( ...
    Vprev, modifiedWrench, M, B, Ts);

assert(all(Vmodified(2:6) == Vnext(2:6)));

disp('All six-axis verification tests passed.');