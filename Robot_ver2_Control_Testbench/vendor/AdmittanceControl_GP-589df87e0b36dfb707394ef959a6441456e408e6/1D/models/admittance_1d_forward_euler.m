function [xNext, vNext] = admittance_1d_forward_euler( ...
    xPrev, vPrev, Fprev, m, b, Ts)
% Forward Euler discretization of 1D admittance.
%
% All state and force inputs refer to the beginning
% of the integration interval.

    vNext = vPrev + (Ts/m)*(Fprev - b*vPrev);

    xNext = xPrev + Ts*vPrev;

end