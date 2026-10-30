function Rnext = integrate_orientation_so3( ...
    Rprev, omegaPrev, omegaNext, Ts)
% Integrate 3D orientation on SO(3).
%
% Rprev:
%   Previous tool-to-base rotation matrix [3x3]
%
% omegaPrev:
%   Previous angular velocity in base frame [rad/s]
%
% omegaNext:
%   Current angular velocity in base frame [rad/s]
%
% Ts:
%   Sampling period [s]

    arguments
        Rprev (3,3) double {mustBeFinite}
        omegaPrev (3,1) double {mustBeFinite}
        omegaNext (3,1) double {mustBeFinite}
        Ts (1,1) double {mustBePositive}
    end

    %% Average angular velocity
    omega = (omegaPrev + omegaNext)/2;

    %% Skew-symmetric angular velocity matrix
    S = [       0, -omega(3),  omega(2);
         omega(3),         0, -omega(1);
        -omega(2),  omega(1),         0];

    %% Update orientation
    Rnext = expm(S*Ts) * Rprev;

end