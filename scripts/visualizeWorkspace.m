%% ========================================================================
% WORKSPACE VISUALIZATION
% =========================================================================
%
% This script assumes workspaceResult already exists.
%
% workspaceResult.points:
%
%       column 1 -> X
%       column 2 -> Y
%       column 3 -> Z

points = workspaceResult.points;

x = points(:,1);
y = points(:,2);
z = points(:,3);

% Horizontal distance from base Z-axis
rho = sqrt(x.^2 + y.^2);


%% ========================================================================
% 1. 3D KINEMATIC WORKSPACE
% =========================================================================

figure

scatter3(x,y,z,5,".");

hold on

scatter3(0,0,0,150,"filled");

text(0,0,0,"  Robot Base", ...
    "FontWeight","bold");

xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

title("Sampled Kinematic TCP Workspace")

axis equal
grid on

hold off


%% ========================================================================
% 2. TOP VIEW — XY
% =========================================================================

figure

scatter(x,y,5,".");

hold on

scatter(0,0,150,"filled");

text(0,0,"  Robot Base", ...
    "FontWeight","bold");

xlabel("X [m]")
ylabel("Y [m]")

title("Kinematic Workspace — Top View (XY)")

axis equal
grid on

hold off


%% ========================================================================
% 3. SIDE VIEW — XZ
% =========================================================================

figure

scatter(x,z,5,".");

hold on

scatter(0,0,150,"filled");

text(0,0,"  Robot Base", ...
    "FontWeight","bold");

xlabel("X [m]")
ylabel("Z [m]")

title("Kinematic Workspace — Side View (XZ)")

axis equal
grid on

hold off


%% ========================================================================
% 4. FRONT VIEW — YZ
% =========================================================================

figure

scatter(y,z,5,".");

hold on

scatter(0,0,150,"filled");

text(0,0,"  Robot Base", ...
    "FontWeight","bold");

xlabel("Y [m]")
ylabel("Z [m]")

title("Kinematic Workspace — Front View (YZ)")

axis equal
grid on

hold off


%% ========================================================================
% 5. RADIAL WORKSPACE — DISTANCE FROM BASE AXIS VS HEIGHT
% =========================================================================
%
% rho is horizontal radial distance from the robot's base Z-axis:
%
% rho = sqrt(X^2 + Y^2)

figure

scatter(rho,z,5,".");

xlabel("Horizontal Distance From Base Axis, rho [m]")
ylabel("Z [m]")

title("Kinematic Workspace — Radial Cross Section")

grid on
