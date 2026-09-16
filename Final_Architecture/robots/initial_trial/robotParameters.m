function params = robotParameters()

% ROBOTPARAMETERS Authoritative numerical data for initial_trial.
%
% Units:
%   lengths: m
%   angles: rad
%   masses: kg
%
% Values marked as estimated must be replaced by CAD/team measurements
% before dynamics, collision, or final design decisions are trusted.

params.id = "initial_trial";
params.displayName = "initial_trial";
params.version = "v1";

%% Geometry

params.geometry.l1 = 0.525;  % O0 -> O1, base height [m].
params.geometry.l2 = 0.120;  % O1 -> shoulder offset level, estimated [m].
params.geometry.l3 = 0.150;  % shoulder horizontal offset to O2 [m].
params.geometry.l4 = 0.770;  % O2 -> O3 main vertical arm [m].
params.geometry.l5 = 0.100;  % O3 -> O4 world-Y offset, estimated [m].
params.geometry.l6 = 0.120;  % O3 -> O4 world-Z offset, estimated [m].
params.geometry.l7 = 0.160;  % O3 -> O4 world-X offset, estimated [m].
params.geometry.l8 = 0.740;  % O4 -> O5/O6 major horizontal reach [m].
params.geometry.l9 = 0.100;  % flange offset, estimated [m].
params.geometry.l10 = 0.200; % fixed welding tool/TCP offset [m].

%% Joints

params.joints.homePosition = zeros(1,6);

params.joints.positionLimits = deg2rad([
    -170,    170;
    -117.5,  117.5;
    -227.5,  227.5;
    -190,    190;
    -180,    180;
    -450,    450]);

params.joints.zeroOffset = zeros(1,6);

%% Tool

params.tool.name = "TIG_Torch";
params.tool.mass = 0.8;
params.tool.TFlangeTCP = eye(4);
params.tool.TFlangeTCP(3,4) = ...
    params.geometry.l9 + params.geometry.l10;

%% Links
%
% Dynamics data is intentionally left unknown until reliable CAD or
% measurements are available.

for i = 1:6
    params.links(i).mass = NaN;
    params.links(i).centerOfMass = [NaN NaN NaN];
    params.links(i).inertia = NaN(3,3);
end

%% Notes

params.notes.missingData = [
    "Replace estimated l2, l5, l6, and l7 with CAD measurements."
    "Confirm whether O4-to-wrist length is exactly l8."
    "Split l9 and l10 into flange and tool lengths from CAD."
    "Confirm exact positive joint directions from CAD."
    "Add masses, centers of mass, inertias, actuator data, and collision geometry."];

end
