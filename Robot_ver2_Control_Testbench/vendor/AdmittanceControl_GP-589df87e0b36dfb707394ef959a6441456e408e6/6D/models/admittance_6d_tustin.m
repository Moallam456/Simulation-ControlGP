function Vnext = admittance_6d_tustin( ...
    Vprev, wrenchPrev, wrenchCurrent, M, B, Ts)
% Six-axis Cartesian admittance using Tustin.
%
% Twist:  [vx; vy; vz; wx; wy; wz]
% Wrench: [Fx; Fy; Fz; Tx; Ty; Tz]
%
% M: [mx; my; mz; Jx; Jy; Jz]
% B: [bx; by; bz; dx; dy; dz]
%
% Translation: m/s, forces: N
% Rotation: rad/s, torques: N*m

    arguments
        Vprev         (6,1) double {mustBeFinite}
        wrenchPrev    (6,1) double {mustBeFinite}
        wrenchCurrent (6,1) double {mustBeFinite}
        M             (6,1) double {mustBePositive}
        B             (6,1) double {mustBePositive}
        Ts            (1,1) double {mustBePositive}
    end

    %% Tustin velocity update

    numerator = (M - B*Ts/2) .* Vprev ...
              + (Ts/2) .* (wrenchPrev + wrenchCurrent);

    denominator = M + B*Ts/2;

    Vnext = numerator ./ denominator;

end