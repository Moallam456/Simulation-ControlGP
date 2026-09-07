function station = buildPreliminaryWeldingStation()
%BUILDPRELIMINARYWELDINGSTATION
% Create preliminary world collision objects for the welding station.
%
% Usage:
%
%   station = environment.buildPreliminaryWeldingStation();
%
% CURRENT ASSUMPTIONS
% -------------------------------------------------------------------------
%
% Robot Base frame:
%
%       B = [0, 0, 0]
%
% The robot mounting plane is:
%
%       Z = 0
%
% A large thin collision box is placed slightly BELOW Z = 0 so that the
% robot base itself is not permanently reported as colliding with the
% mounting surface.
%
% A preliminary welding table is positioned in front of the robot.
%
% These dimensions are DESIGN ASSUMPTIONS only.
% They shall later be replaced by the actual client installation geometry.


%% ========================================================================
% 1. ROBOT BASE / WORLD REFERENCE
% =========================================================================

station.robotBasePosition = [0; 0; 0];

station.mountingPlaneZ = 0;


%% ========================================================================
% 2. MOUNTING PLANE
% =========================================================================
%
% MATLAB has no infinite collision plane primitive.
%
% Therefore we approximate the mounting/floor plane using a large,
% thin collisionBox.
%
% Top surface is placed slightly below Z = 0 to avoid the legitimate
% robot-base mounting interface being detected as a collision.

station.mountingPlane.sizeX = 2.50;    % [m]
station.mountingPlane.sizeY = 2.50;    % [m]
station.mountingPlane.thickness = 0.020; % [m]

% Leave 5 mm clearance below robot mounting plane.

station.mountingPlane.topZ = -0.005;

station.mountingPlane.centerZ = ...
    station.mountingPlane.topZ - ...
    station.mountingPlane.thickness/2;


mountingPlane = collisionBox( ...
    station.mountingPlane.sizeX, ...
    station.mountingPlane.sizeY, ...
    station.mountingPlane.thickness);

mountingPlane.Pose = trvec2tform([ ...
    0, ...
    0, ...
    station.mountingPlane.centerZ]);


%% ========================================================================
% 3. PRELIMINARY WELDING TABLE
% =========================================================================
%
% Preliminary arrangement:
%
%                  +X
%
% Robot Base ------------------> Welding Table
%
%
% Table center:
%
%       X = 0.55 m
%       Y = 0
%
% Table top:
%
%       Z = 0.20 m
%
% These values are NOT final client geometry.

station.table.centerX = 0.55;
station.table.centerY = 0.00;

station.table.sizeX = 0.60;       % [m]
station.table.sizeY = 0.70;       % [m]

station.table.thickness = 0.050;  % [m]

station.table.topZ = 0.20;

station.table.centerZ = ...
    station.table.topZ - ...
    station.table.thickness/2;


weldingTable = collisionBox( ...
    station.table.sizeX, ...
    station.table.sizeY, ...
    station.table.thickness);

weldingTable.Pose = trvec2tform([ ...
    station.table.centerX, ...
    station.table.centerY, ...
    station.table.centerZ]);


%% ========================================================================
% 4. STORE WORLD OBJECTS
% =========================================================================
%
% checkCollision expects world collision geometries as a cell array.

station.mountingPlane.object = mountingPlane;

station.table.object = weldingTable;

station.worldObjects = { ...
    mountingPlane, ...
    weldingTable ...
};


%% ========================================================================
% 5. REQUIRED WELDING REGION — PRELIMINARY
% =========================================================================
%
% This is positioned ABOVE the welding table.
%
% Later this will be replaced by actual fixture/workpiece geometry.

station.workspace.centerX = 0.55;
station.workspace.centerY = 0.00;
station.workspace.centerZ = 0.30;

station.workspace.sizeX = 0.40;
station.workspace.sizeY = 0.40;
station.workspace.sizeZ = 0.20;

end