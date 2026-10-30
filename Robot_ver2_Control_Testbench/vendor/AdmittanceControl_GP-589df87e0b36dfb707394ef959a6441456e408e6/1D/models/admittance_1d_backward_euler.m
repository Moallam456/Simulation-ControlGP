
function [xNext, vNext] = admittance_1d_backward_euler( ...
    xPrev, vPrev, F, m, b, Ts)
% Single-axis admittance controller using backward Euler.
%
% States:
%   xPrev - Previous displacement [m]
%   vPrev - Previous velocity [m/s]
%
% Input:
%   F  - Current force measurement [N]
%
% Parameters:
%   m  - Virtual mass [kg]
%   b  - Virtual damping [N*s/m]
%   Ts - Sampling period [s]
%
% Outputs:
%   xNext - Updated displacement [m]
%   vNext - Updated velocity [m/s]

    arguments
        xPrev (1,1) double {mustBeFinite}
        vPrev (1,1) double {mustBeFinite}
        F     (1,1) double {mustBeFinite}
        m     (1,1) double {mustBePositive}
        b     (1,1) double {mustBePositive}
        Ts    (1,1) double {mustBePositive}
    end

    %% Step 1: Update velocity
    vNext = (m*vPrev + Ts*F) / (m + b*Ts);

    %% Step 2: Update displacement
    xNext = xPrev + Ts*vNext;

end
