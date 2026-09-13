function A = dhTransform(a, d, alpha, theta)
% DHTRANSFORM
% Modified-DH homogeneous transform used by buildRobotCAD,
% controlFK, and analyticalIK_CAD.
%
% Convention:
%   A = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
%
% This is NOT the classical/standard DH matrix.

ca = cos(alpha);
sa = sin(alpha);
ct = cos(theta);
st = sin(theta);

A = [ ...
    ct,       -st,       0,        a;
    ca*st,    ca*ct,    -sa,   -d*sa;
    sa*st,    sa*ct,     ca,    d*ca;
    0,         0,        0,        1];

end
