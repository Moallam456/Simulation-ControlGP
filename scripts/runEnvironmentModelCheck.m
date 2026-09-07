clear
clc
close all

%% ========================================================================
% 1. ROBOT GEOMETRY
% =========================================================================

geometry.d1 = 0.089159;

geometry.L2 = 0.425;
geometry.L3 = 0.375;

geometry.d4 = 0.10915;
geometry.d5 = 0.09465;
geometry.d6 = 0.0823;


%% ========================================================================
% 2. BUILD ROBOT
% =========================================================================

robot = config.WeldingRobot(geometry);

robotModel = ...
    robotmodel.buildRobot(robot);

collisionRobot = ...
    robotmodel.addPreliminaryCollisionGeometry( ...
        robotModel, ...
        robot);


%% ========================================================================
% 3. BUILD WELDING STATION
% =========================================================================

station = ...
    environment.buildPreliminaryWeldingStation();


%% ========================================================================
% 4. ZERO CONFIGURATION
% =========================================================================

qZero = zeros(robot.dof,1);


%% ========================================================================
% 5. VISUALIZE ROBOT + ENVIRONMENT
% =========================================================================

figure

ax = show( ...
    collisionRobot, ...
    qZero, ...
    "Visuals","off", ...
    "Collisions","on", ...
    "Frames","on");

hold(ax,"on");


%% Show mounting plane

[~, mountingPatch] = show( ...
    station.mountingPlane.object, ...
    "Parent",ax);

mountingPatch.FaceAlpha = 0.25;


%% Show welding table

[~, tablePatch] = show( ...
    station.table.object, ...
    "Parent",ax);

tablePatch.FaceAlpha = 0.45;


xlabel(ax,"X [m]")
ylabel(ax,"Y [m]")
zlabel(ax,"Z [m]")

title(ax, ...
    "Welding Robot + Preliminary Welding Station");

axis(ax,"equal")
grid(ax,"on")

hold(ax,"off")


%% ========================================================================
% 6. PRINT ENVIRONMENT
% =========================================================================

fprintf("\n");
fprintf("============================================\n");
fprintf("PRELIMINARY WELDING STATION\n");
fprintf("============================================\n");

fprintf("\nRobot Base:\n");
fprintf("[0, 0, 0] m\n");

fprintf("\nMounting plane:\n");
fprintf("Z = %.3f m\n", ...
    station.mountingPlaneZ);

fprintf("\nWelding table:\n");

fprintf("Center X = %.0f mm\n", ...
    station.table.centerX*1000);

fprintf("Center Y = %.0f mm\n", ...
    station.table.centerY*1000);

fprintf("Size X = %.0f mm\n", ...
    station.table.sizeX*1000);

fprintf("Size Y = %.0f mm\n", ...
    station.table.sizeY*1000);

fprintf("Table top Z = %.0f mm\n", ...
    station.table.topZ*1000);