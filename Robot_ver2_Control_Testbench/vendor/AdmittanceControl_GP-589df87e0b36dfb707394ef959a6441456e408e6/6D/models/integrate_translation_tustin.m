function pNext = integrate_translation_tustin( ...
    pPrev, Vprev, Vnext, Ts)
% Integrate Cartesian translation using trapezoidal rule.
%
% pPrev: Previous Cartesian position [m], 3x1
% Vprev: Previous Cartesian twist, 6x1
% Vnext: Current Cartesian twist, 6x1
% Ts: Sampling period [s]
%
% Twist order:
% [vx; vy; vz; wx; wy; wz]
%
% Assumption:
% Linear velocities are expressed in the fixed base frame.

    arguments
        pPrev (3,1) double {mustBeFinite}
        Vprev (6,1) double {mustBeFinite}
        Vnext (6,1) double {mustBeFinite}
        Ts (1,1) double {mustBePositive, mustBeFinite}
    end

    %% Extract translational velocity
    vPrev = Vprev(1:3);
    vNext = Vnext(1:3);

    %% Trapezoidal position integration
    pNext = pPrev + (Ts/2)*(vPrev + vNext);

end