function A = dhTransform(a,d,alpha,theta,convention)

% DHTRANSFORM Robot-specific DH transform helper for initial_trial.
%
% Supported conventions:
%   "modified": A = Rx(alpha) * Tx(a) * Rz(theta) * Tz(d)
%   "standard": A = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
%
% The current initial_trial rigidBodyTree is built from fixed transforms
% because the sketch contains compound offsets. This helper is kept in the
% robot folder for reference DH calculations and future analytical IK work.

if nargin < 5 || isempty(convention)
    convention = "modified";
end

switch lower(string(convention))
    case "modified"
        A = modifiedDH(a,d,alpha,theta);
    case "standard"
        A = standardDH(a,d,alpha,theta);
    otherwise
        error('initial_trial:UnknownDHConvention', ...
            'Unknown DH convention "%s".', convention);
end

end

function A = modifiedDH(a,d,alpha,theta)

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

function A = standardDH(a,d,alpha,theta)

ca = cos(alpha);
sa = sin(alpha);
ct = cos(theta);
st = sin(theta);

A = [ ...
    ct,   -st*ca,   st*sa,   a*ct;
    st,    ct*ca,  -ct*sa,   a*st;
    0,     sa,      ca,      d;
    0,     0,       0,       1];

end
