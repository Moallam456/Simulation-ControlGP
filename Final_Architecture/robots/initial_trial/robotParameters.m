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

params.geometry.l1 = 0.200;  % O0 -> O1, base height, estimated [m].
params.geometry.l2 = 0.120;  % O1 -> shoulder offset level, estimated [m].
params.geometry.l3 = 0.150;  % shoulder horizontal offset to O2 [m].
params.geometry.l4 = 0.350;  % J2 -> J3 arm segment [m].
params.geometry.l5 = 0.040;  % J3 -> J4 world-Y offset, estimated [m].
params.geometry.l6 = 0.050;  % J3 -> J4 world-Z offset, estimated [m].
params.geometry.l7 = 0.060;  % J3 -> J4 world-X offset, estimated [m].
params.geometry.l8 = 0.075;  % J4 -> J5 segment [m].
params.geometry.l9 = 0.075;  % J5 -> J6 segment [m].
params.geometry.l10 = 0.050; % J6 -> end-effector segment [m].

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
params.tool.mass = 2.0;
params.tool.segmentMass = 0.71; % J6 -> EE structure, separate from tool mass [kg].
params.tool.radius = 0.035; % estimated equivalent payload/tool radius [m].
params.tool.TFlangeTCP = eye(4);
params.tool.TFlangeTCP(3,4) = ...
    params.geometry.l10;
params.tool.centerOfMass = [0 0 0]; % at TCP for current preliminary payload model.
params.tool.inertia = NaN(3,3); % optional inertia about tool COM, TCP axes [kg*m^2].
params.tool.massPropertySource = ...
    "preliminary welding tool/payload mass, lumped at TCP";
params.tool.dynamics.status = "estimated";

%% Links
%
% A body at Ji carries the segment Ji -> J(i+1). The supplied motor/gearbox
% modules are provisionally assigned to the upstream body at the distal
% joint center. J6 -> TCP structure and payload live on the fixed TCP body.
params.base.mass = 12.00; % fixed O0 -> J1 assembly, estimated [kg].
params.base.radius = 0.055; % estimated [m].
params.base.dynamics.status = "estimated";
params.base.massPropertySource = "preliminary fixed base assembly mass";

structuralMass = [10.00 5.00 2.14 1.07 1.07 0]; % kg
jointModuleMass = [0 3.20 1.50 1.50 1.00 0]; % kg
linkMass = structuralMass + jointModuleMass;

linkRadius = [0.050 0.040 0.035 0.030 0.026 0.026]; % m, estimated
jointRadius = [0.065 0.048 0.042 0.036 0.032 0.032]; % m, estimated

linkColor = [
    0.15 0.15 0.15
    0.95 0.78 0.05
    0.95 0.78 0.05
    0.95 0.78 0.05
    0.95 0.78 0.05
    0.20 0.20 0.20];

for i = 1:6
    params.links(i).mass = linkMass(i);
    params.links(i).structuralMass = structuralMass(i);
    params.links(i).jointModuleMass = jointModuleMass(i);
    params.links(i).radius = linkRadius(i);
    params.links(i).jointRadius = jointRadius(i);
    params.links(i).color = linkColor(i,:);
    params.links(i).centerOfMass = [NaN NaN NaN];
    params.links(i).inertia = NaN(3,3); % optional tensor about COM, body axes.
    params.links(i).massPropertySource = ...
        "preliminary segment and joint-module masses, simplified geometry inertia";
    params.links(i).dynamics.status = "estimated";
end

params.dynamics.horizontalReferenceFrame = ...
    "COM distances measured from J2 with the arm completely horizontal.";
params.dynamics.geometryChecks.j3ToJ4OffsetPathLength = ...
    params.geometry.l5 + params.geometry.l6 + params.geometry.l7;
params.dynamics.geometryChecks.j3ToJ4ProvidedLength = 0.150;
params.dynamics.provisionalMasses = [
    struct("body","base_structure","reason","estimated fixed base assembly mass")
    struct("body","link_1","reason","estimated J1-to-J2 shoulder support mass")];

% Source measurements (horizontal J2 reference) are retained as notes. The
% geometry and component masses above are the single numerical source.
params.dynamics.horizontalComFromJ2 = [0.175 0.425 0.5375 0.6125 0.675];

%% Notes

params.notes.missingData = [
    "Replace estimated l1 and l2 with CAD measurements."
    "Confirm the estimated J3-to-J4 offset split l5/l6/l7; the current path length sums to 150 mm."
    "Replace provisional fixed-base and J1-to-J2 masses with measured assembly masses."
    "Confirm which side of each joint carries its motor/gearbox housing."
    "Confirm final radii and physical shape for each simplified cylinder."
    "Confirm exact positive joint directions from CAD."
    "Replace simplified inertias with CAD-derived inertias when available."
    "Add actuator data and final collision geometry."];

end
