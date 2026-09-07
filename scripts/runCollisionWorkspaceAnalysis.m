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
% 2. CREATE ROBOT CONFIGURATION
% =========================================================================

robot = config.WeldingRobot(geometry);


%% ========================================================================
% 3. BUILD KINEMATIC ROBOT
% =========================================================================

robotModel = ...
    robotmodel.buildRobot(robot);


%% ========================================================================
% 4. BUILD COLLISION ROBOT
% =========================================================================

collisionRobot = ...
    robotmodel.addPreliminaryCollisionGeometry( ...
        robotModel, ...
        robot);


%% ========================================================================
% 5. ANALYSIS SETTINGS
% =========================================================================

numSamples = 50000;

randomSeed = 1;


%% ========================================================================
% 6. GENERATE MATHEMATICAL KINEMATIC WORKSPACE
% =========================================================================

kinematicWorkspace = ...
    analysis.workspace( ...
        robotModel, ...
        robot, ...
        numSamples, ...
        randomSeed);


%% ========================================================================
% 7. GENERATE COLLISION-FREE WORKSPACE
% =========================================================================

collisionFreeWorkspace = ...
    analysis.collisionFreeWorkspace( ...
        collisionRobot, ...
        robot, ...
        numSamples, ...
        randomSeed);


%% ========================================================================
% 8. PRINT RESULTS
% =========================================================================

fprintf("\n");
fprintf("============================================================\n");
fprintf("KINEMATIC VS COLLISION-FREE WORKSPACE\n");
fprintf("============================================================\n");

fprintf("\nROBOT GEOMETRY\n");
fprintf("----------------------------\n");

fprintf("L2 = %.0f mm\n", ...
    geometry.L2*1000);

fprintf("L3 = %.0f mm\n", ...
    geometry.L3*1000);

fprintf("L2 + L3 = %.0f mm\n", ...
    (geometry.L2+geometry.L3)*1000);


%% Kinematic workspace

fprintf("\nKINEMATIC WORKSPACE\n");
fprintf("----------------------------\n");

fprintf("Maximum 3-D reach = %.1f mm\n", ...
    kinematicWorkspace.maxReach*1000);

fprintf("Minimum 3-D radius = %.1f mm\n", ...
    kinematicWorkspace.minReach*1000);

fprintf("Maximum horizontal radius = %.1f mm\n", ...
    kinematicWorkspace.maxHorizontalReach*1000);

fprintf("Minimum horizontal radius = %.1f mm\n", ...
    kinematicWorkspace.minHorizontalReach*1000);


%% Collision-free workspace

fprintf("\nCOLLISION-FREE WORKSPACE\n");
fprintf("----------------------------\n");

fprintf("Maximum 3-D reach = %.1f mm\n", ...
    collisionFreeWorkspace.maxReach*1000);

fprintf("Minimum 3-D radius = %.1f mm\n", ...
    collisionFreeWorkspace.minReach*1000);

fprintf("Maximum horizontal radius = %.1f mm\n", ...
    collisionFreeWorkspace.maxHorizontalReach*1000);

fprintf("Minimum horizontal radius = %.1f mm\n", ...
    collisionFreeWorkspace.minHorizontalReach*1000);


%% Collision statistics

fprintf("\nCOLLISION STATISTICS\n");
fprintf("----------------------------\n");

fprintf("Total sampled configurations = %d\n", ...
    collisionFreeWorkspace.numSamples);

fprintf("Collision-free configurations = %d\n", ...
    collisionFreeWorkspace.numCollisionFree);

fprintf("Colliding configurations = %d\n", ...
    collisionFreeWorkspace.numColliding);

fprintf("Collision-free percentage = %.2f %%\n", ...
    collisionFreeWorkspace.collisionFreePercent);

fprintf("Rejected by collision = %.2f %%\n", ...
    collisionFreeWorkspace.collisionPercent);

fprintf("\n============================================================\n");


%% ========================================================================
% 9. 3-D KINEMATIC WORKSPACE
% =========================================================================

figure
hold on

scatter3( ...
    kinematicWorkspace.points(:,1), ...
    kinematicWorkspace.points(:,2), ...
    kinematicWorkspace.points(:,3), ...
    3, ...
    ".");

scatter3( ...
    0,0,0, ...
    150, ...
    "filled");

text(0,0,0, ...
    "  Robot Base", ...
    "FontWeight","bold");

xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

title("Mathematical Kinematic Workspace")

axis equal
grid on

hold off


%% ========================================================================
% 10. 3-D COLLISION-FREE WORKSPACE
% =========================================================================

figure
hold on

scatter3( ...
    collisionFreeWorkspace.points(:,1), ...
    collisionFreeWorkspace.points(:,2), ...
    collisionFreeWorkspace.points(:,3), ...
    3, ...
    ".");

scatter3( ...
    0,0,0, ...
    150, ...
    "filled");

text(0,0,0, ...
    "  Robot Base", ...
    "FontWeight","bold");

xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

title("Preliminary Collision-Free Workspace")

axis equal
grid on

hold off


%% ========================================================================
% 11. RADIAL WORKSPACE COMPARISON
% =========================================================================

figure
hold on

scatter( ...
    kinematicWorkspace.horizontalRadius, ...
    kinematicWorkspace.points(:,3), ...
    3, ...
    ".");

scatter( ...
    collisionFreeWorkspace.horizontalRadius, ...
    collisionFreeWorkspace.points(:,3), ...
    4, ...
    ".");

xlabel("Horizontal Radius \rho [m]")
ylabel("Z [m]")

title("Kinematic vs Collision-Free Radial Workspace")

legend( ...
    "Kinematic Workspace", ...
    "Collision-Free Workspace", ...
    "Location","best");

grid on

hold off

%% ========================================================================
% FIND COLLISION-FREE CONFIGURATION WITH MINIMUM HORIZONTAL RADIUS
% =========================================================================

[minimumHorizontalRadius, indexMinHorizontal] = ...
    min(collisionFreeWorkspace.horizontalRadius);

qMinHorizontal = ...
    collisionFreeWorkspace.q(:,indexMinHorizontal);

pointMinHorizontal = ...
    collisionFreeWorkspace.points(indexMinHorizontal,:);


fprintf("\nMINIMUM COLLISION-FREE HORIZONTAL RADIUS\n");
fprintf("----------------------------------------\n");

fprintf("Horizontal radius = %.1f mm\n", ...
    minimumHorizontalRadius*1000);

fprintf("TCP position:\n");
fprintf("X = %.1f mm\n",pointMinHorizontal(1)*1000);
fprintf("Y = %.1f mm\n",pointMinHorizontal(2)*1000);
fprintf("Z = %.1f mm\n",pointMinHorizontal(3)*1000);

fprintf("\nJoint configuration [deg]:\n");

disp(rad2deg(qMinHorizontal));


%% Show configuration

figure

show( ...
    collisionRobot, ...
    qMinHorizontal, ...
    "Visuals","off", ...
    "Collisions","on", ...
    "Frames","on");

axis equal
grid on

title(sprintf( ...
    "Minimum Collision-Free Horizontal Radius = %.1f mm, Z = %.1f mm", ...
    minimumHorizontalRadius*1000, ...
    pointMinHorizontal(3)*1000));


%% ========================================================================
% 12. SIDE-SLICE COMPARISON
% =========================================================================
%
% Use only configurations close to Y = 0 so that internal workspace
% structure is not hidden by a full projection.

sliceHalfThickness = 0.025;


%% Kinematic slice

kinematicSliceMask = ...
    abs(kinematicWorkspace.points(:,2)) <= ...
    sliceHalfThickness;


%% Collision-free slice

collisionSliceMask = ...
    abs(collisionFreeWorkspace.points(:,2)) <= ...
    sliceHalfThickness;


figure
hold on

scatter( ...
    kinematicWorkspace.points(kinematicSliceMask,1), ...
    kinematicWorkspace.points(kinematicSliceMask,3), ...
    6, ...
    ".");

scatter( ...
    collisionFreeWorkspace.points(collisionSliceMask,1), ...
    collisionFreeWorkspace.points(collisionSliceMask,3), ...
    10, ...
    ".");


scatter(0,0,150,"filled");

text(0,0, ...
    "  Robot Base", ...
    "FontWeight","bold");


xlabel("X [m]")
ylabel("Z [m]")

title(sprintf( ...
    "XZ Slice Comparison — |Y| <= %.0f mm", ...
    sliceHalfThickness*1000));

legend( ...
    "Kinematic", ...
    "Collision-Free", ...
    "Robot Base", ...
    "Location","best");

axis equal
grid on

hold off