
function Vnext = admittance_6d_backward_euler( ...
    Vprev, wrench, M, B, Ts)
% Six-axis Cartesian admittance using backward Euler.
%
% All vectors are 6x1 in the order:
% [X; Y; Z; Rx; Ry; Rz]
%
% Vprev:
%   [vx; vy; vz; wx; wy; wz]
%
% wrench:
%   [Fx; Fy; Fz; Tx; Ty; Tz]
%
% M:
%   [mx; my; mz; Jx; Jy; Jz]
%
% B:
%   [bx; by; bz; dx; dy; dz]

    arguments
        Vprev (6,1) double {mustBeFinite}
        wrench (6,1) double {mustBeFinite}
        M (6,1) double {mustBePositive}
        B (6,1) double {mustBePositive}
        Ts (1,1) double {mustBePositive}
    end

    %% Backward Euler velocity update
    Vnext = (M .* Vprev + Ts .* wrench) ...
          ./ (M + Ts .* B);

end
