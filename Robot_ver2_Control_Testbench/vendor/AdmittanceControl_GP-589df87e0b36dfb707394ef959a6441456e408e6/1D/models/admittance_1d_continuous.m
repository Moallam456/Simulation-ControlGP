function Gv = admittance_1d_continuous(m, b)
% Creates a continuous-time, single-axis admittance model.
%
% Input:  Applied force (N)
% Output: Commanded velocity (m/s)
%
% m: Virtual mass (kg)
% b: Virtual damping (N*s/m)

    arguments
        m (1,1) double {mustBePositive}
        b (1,1) double {mustBePositive}
    end

    Gv = tf(1, [m b]);

end