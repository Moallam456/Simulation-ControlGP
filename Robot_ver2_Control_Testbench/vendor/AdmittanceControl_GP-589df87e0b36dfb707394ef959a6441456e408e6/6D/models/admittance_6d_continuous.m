
function sys = admittance_6d_continuous(M, B)
% Continuous-time six-axis Cartesian admittance.
%
% Model:
%   M * dV/dt + B * V = wrench
%
% States and outputs:
%   V = [vx; vy; vz; wx; wy; wz]
%
% Inputs:
%   wrench = [Fx; Fy; Fz; Tx; Ty; Tz]
%
% M contains diagonal virtual inertia values:
%   [mx; my; mz; Jx; Jy; Jz]
%
% B contains diagonal virtual damping values:
%   [bx; by; bz; dx; dy; dz]

    arguments
        M (6,1) double {mustBePositive, mustBeFinite}
        B (6,1) double {mustBePositive, mustBeFinite}
    end

    %% Continuous-time state-space matrices

    A = -diag(B ./ M);

    Bu = diag(1 ./ M);

    C = eye(6);

    D = zeros(6,6);

    %% Create continuous-time model

    sys = ss(A, Bu, C, D);

    %% Signal names

    sys.StateName = {'vx','vy','vz','wx','wy','wz'};

    sys.InputName = {'Fx','Fy','Fz','Tx','Ty','Tz'};

    sys.OutputName = {'vx','vy','vz','wx','wy','wz'};

end
