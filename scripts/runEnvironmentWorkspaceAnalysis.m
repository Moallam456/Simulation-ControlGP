clear
clc
close all

%% ========================================================================
% ROBOT GEOMETRY
% =========================================================================

geometry.d1 = 0.089159;

geometry.L2 = 0.425;
geometry.L3 = 0.375;

geometry.d4 = 0.10915;
geometry.d5 = 0.09465;
geometry.d6 = 0.0823;


%% ========================================================================
% BUILD ROBOT
% =========================================================================

robot = config.WeldingRobot(geometry);

robotModel = ...
    robotmodel.buildRobot(robot);

collisionRobot = ...
    robotmodel.addPreliminaryCollisionGeometry( ...
        robotModel, ...
        robot);


%% ========================================================================
% BUILD ENVIRONMENT
% =========================================================================

station = ...
    environment.buildPreliminaryWeldingStation();


%% ========================================================================
% SETTINGS
% =========================================================================

numSamples = 50000;

randomSeed = 1;


%% ========================================================================
% SELF-COLLISION-FREE WORKSPACE
% =========================================================================

selfCollisionFreeWorkspace = ...
    analysis.collisionFreeWorkspace( ...
        collisionRobot, ...
        robot, ...
        numSamples, ...
        randomSeed);


%% ========================================================================
% ENVIRONMENT-CONSTRAINED WORKSPACE
% =========================================================================

environmentWorkspace = ...
    analysis.environmentConstrainedWorkspace( ...
        collisionRobot, ...
        robot, ...
        station.worldObjects, ...
        numSamples, ...
        randomSeed);


%% ========================================================================
% RESULTS
% =========================================================================

fprintf("\n====================================================\n");
fprintf("ENVIRONMENT-CONSTRAINED WORKSPACE\n");
fprintf("====================================================\n");

fprintf("\nTotal configurations = %d\n", ...
    environmentWorkspace.numSamples);

fprintf("\nValid after all collision checks = %.2f %%\n", ...
    environmentWorkspace.validPercent);

fprintf("Self-collision configurations = %.2f %%\n", ...
    environmentWorkspace.selfCollisionPercent);

fprintf("World-collision configurations = %.2f %%\n", ...
    environmentWorkspace.worldCollisionPercent);

fprintf("\nSelf-collision-free minimum Z = %.1f mm\n", ...
    min(selfCollisionFreeWorkspace.points(:,3))*1000);

fprintf("Environment-constrained minimum Z = %.1f mm\n", ...
    min(environmentWorkspace.points(:,3))*1000);

fprintf("\nSelf-collision-free maximum reach = %.1f mm\n", ...
    selfCollisionFreeWorkspace.maxReach*1000);

fprintf("Environment-constrained maximum reach = %.1f mm\n", ...
    environmentWorkspace.maxReach*1000);


%% ========================================================================
% RADIAL COMPARISON
% =========================================================================

figure
hold on

scatter( ...
    selfCollisionFreeWorkspace.horizontalRadius, ...
    selfCollisionFreeWorkspace.points(:,3), ...
    3, ...
    ".");

scatter( ...
    environmentWorkspace.horizontalRadius, ...
    environmentWorkspace.points(:,3), ...
    5, ...
    ".");

xlabel("Horizontal Radius \rho [m]")
ylabel("Z [m]")

title("Self-Collision-Free vs Environment-Constrained Workspace")

legend( ...
    "Self-Collision-Free", ...
    "Environment-Constrained", ...
    "Location","best");

grid on
hold off


%% ========================================================================
% 3-D ENVIRONMENT-CONSTRAINED WORKSPACE
% =========================================================================

figure

ax = axes;

scatter3( ...
    ax, ...
    environmentWorkspace.points(:,1), ...
    environmentWorkspace.points(:,2), ...
    environmentWorkspace.points(:,3), ...
    4, ...
    ".");

hold(ax,"on");


%% Robot base

scatter3(ax,0,0,0,150,"filled");

text(ax,0,0,0, ...
    "  Robot Base", ...
    "FontWeight","bold");


%% Mounting plane

[~, mountingPatch] = show( ...
    station.mountingPlane.object, ...
    "Parent",ax);

mountingPatch.FaceAlpha = 0.20;


%% Welding table

[~, tablePatch] = show( ...
    station.table.object, ...
    "Parent",ax);

tablePatch.FaceAlpha = 0.40;


xlabel(ax,"X [m]")
ylabel(ax,"Y [m]")
zlabel(ax,"Z [m]")

title(ax, ...
    "Environment-Constrained TCP Workspace")

axis(ax,"equal")
grid(ax,"on")

hold(ax,"off")