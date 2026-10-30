
function [xNext, vNext] = admittance_1d_tustin( ...
    xPrev, vPrev, Fprev, Fcurrent, m, b, Ts)
% Single-axis admittance using Tustin discretization.
%
% Fprev    - Force at previous sample [N]
% Fcurrent - Force at current sample [N]
% m        - Virtual mass [kg]
% b        - Virtual damping [N*s/m]
% Ts       - Sampling period [s]

    arguments
        xPrev (1,1) double {mustBeFinite}
        vPrev (1,1) double {mustBeFinite}
        Fprev (1,1) double {mustBeFinite}
        Fcurrent (1,1) double {mustBeFinite}
        m (1,1) double {mustBePositive}
        b (1,1) double {mustBePositive}
        Ts (1,1) double {mustBePositive}
    end

    %% Tustin velocity update
    numerator = (m - b*Ts/2)*vPrev ...
              + (Ts/2)*(Fprev + Fcurrent);

    denominator = m + b*Ts/2;

    vNext = numerator / denominator;

    %% Trapezoidal position integration
    xNext = xPrev + (Ts/2)*(vPrev + vNext);

end
