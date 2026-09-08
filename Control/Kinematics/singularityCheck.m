function isSingular = singularityCheck(robot, q)
%SINGULARITYCHECK Check if a robot configuration is kinematically singular.
%
% Inputs:
%   robot       - Shared robot configuration
%   q           - Joint configuration, 6x1 vector [rad]
%
% Output:
%   isSingular  - true if the configuration is singular

% Calculate the geometric Jacobian at the current configuration
J = controlJacobian(robot, q);

% Maximum possible rank of the Jacobian
maxRank = min(size(J));

% A configuration is singular if the Jacobian loses rank
isSingular = rank(J) < maxRank;

end