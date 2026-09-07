clear
clc
close all

%% ========================================================================
% 1. WELDING ROBOT CANDIDATE
% =========================================================================

geometry.d1 = 0.089159;

% Main design variables
geometry.L2 = 0.425;
geometry.L3 = 0.375;

% Preliminary wrist values
geometry.d4 = 0.10915;
geometry.d5 = 0.09465;
geometry.d6 = 0.0823;


%% ========================================================================
% 2. BUILD ROBOT
% =========================================================================

robot = config.WeldingRobot(geometry);

robotModel = robotmodel.buildRobot(robot);


%% ========================================================================
% 3. GENERATE KINEMATIC WORKSPACE
% =========================================================================
%
% Use more samples here because this is a diagnostic investigation.

numSamples = 50000;

randomSeed = 1;

workspaceResult = analysis.workspace( ...
    robotModel, ...
    robot, ...
    numSamples, ...
    randomSeed);


%% ========================================================================
% 4. CALCULATE RADIAL ENVELOPE
% =========================================================================

workspaceEnvelope = ...
    analysis.workspaceEnvelope( ...
        workspaceResult, ...
        60);


%% ========================================================================
% 5. EXTRACT DATA
% =========================================================================

points = workspaceResult.points;

x = points(:,1);
y = points(:,2);
z = points(:,3);

rho = workspaceResult.horizontalRadius;


%% ========================================================================
% 6. DISPLAY BASIC RESULTS
% =========================================================================

fprintf("\n====================================================\n");
fprintf("KINEMATIC WORKSPACE QUALITY CHECK\n");
fprintf("====================================================\n");

fprintf("\nL2 = %.0f mm\n",geometry.L2*1000);
fprintf("L3 = %.0f mm\n",geometry.L3*1000);

fprintf("\nMaximum 3-D reach = %.4f m\n", ...
    workspaceResult.maxReach);

fprintf("Minimum 3-D distance to base origin = %.4f m\n", ...
    workspaceResult.minReach);

fprintf("\nMaximum horizontal radius = %.4f m\n", ...
    workspaceResult.maxHorizontalReach);

fprintf("Minimum horizontal radius = %.4f m\n", ...
    workspaceResult.minHorizontalReach);


%% ========================================================================
% 7. PRINT CONFIGURATION CLOSEST TO BASE
% =========================================================================

fprintf("\n----------------------------------------------------\n");
fprintf("CONFIGURATION CLOSEST TO BASE ORIGIN\n");
fprintf("----------------------------------------------------\n");

disp("TCP position [m]:")

disp(workspaceResult.closestToBase.point)

disp("Joint angles [deg]:")

disp(rad2deg( ...
    workspaceResult.closestToBase.q))


%% ========================================================================
% 8. 3-D WORKSPACE
% =========================================================================

figure

scatter3(x,y,z,3,".");

hold on

scatter3(0,0,0,150,"filled");

text(0,0,0, ...
    "  Robot Base", ...
    "FontWeight","bold");

xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

title("Sampled Kinematic TCP Workspace")

axis equal
grid on

hold off


%% ========================================================================
% 9. TOP VIEW — XY PROJECTION
% =========================================================================

figure

scatter(x,y,3,".");

hold on

scatter(0,0,150,"filled");

text(0,0, ...
    "  Robot Base", ...
    "FontWeight","bold");

xlabel("X [m]")
ylabel("Y [m]")

title("Kinematic Workspace — XY Projection")

axis equal
grid on

hold off


%% ========================================================================
% 10. SIDE SLICE — XZ
% =========================================================================
%
% IMPORTANT:
%
% This is a SLICE, not a projection.
%
% Only points close to Y = 0 are shown.
%
% This prevents the full 3-D workspace from being collapsed onto the
% X-Z plane and hiding internal holes.

sliceHalfThickness = 0.025;     % +/-25 mm

sliceMask = ...
    abs(y) <= sliceHalfThickness;


figure

scatter( ...
    x(sliceMask), ...
    z(sliceMask), ...
    8, ...
    ".");

hold on

scatter(0,0,150,"filled");

text(0,0, ...
    "  Robot Base", ...
    "FontWeight","bold");

xlabel("X [m]")
ylabel("Z [m]")

title(sprintf( ...
    "Workspace XZ Slice — |Y| <= %.0f mm", ...
    sliceHalfThickness*1000));

axis equal
grid on

hold off


%% ========================================================================
% 11. RADIAL WORKSPACE
% =========================================================================
%
% This is one of the most important plots.
%
% rho = horizontal distance from the base Z-axis.

figure

scatter(rho,z,3,".");

hold on


%% Inner radial boundary

plot( ...
    workspaceEnvelope.rhoMin, ...
    workspaceEnvelope.z, ...
    "LineWidth",2);


%% Outer radial boundary

plot( ...
    workspaceEnvelope.rhoMax, ...
    workspaceEnvelope.z, ...
    "LineWidth",2);


xlabel("Horizontal Radius \rho [m]")
ylabel("Z [m]")

title("Radial Kinematic Workspace")

legend( ...
    "Sampled TCP Positions", ...
    "Approx. Inner Boundary", ...
    "Approx. Outer Boundary", ...
    "Location","best");

grid on

hold off


%% ========================================================================
% 12. SHOW CONFIGURATION CLOSEST TO BASE ORIGIN
% =========================================================================

qClosest = ...
    workspaceResult.closestToBase.q;

figure

show( ...
    robotModel, ...
    qClosest, ...
    "Frames","on", ...
    "Visuals","off");

axis equal
grid on

title(sprintf( ...
    "Pose Producing Closest TCP to Base — %.1f mm", ...
    workspaceResult.closestToBase.distance*1000));


%% ========================================================================
% 13. SHOW CONFIGURATION CLOSEST TO BASE Z-AXIS
% =========================================================================

qAxis = ...
    workspaceResult.closestToBaseAxis.q;

figure

show( ...
    robotModel, ...
    qAxis, ...
    "Frames","on", ...
    "Visuals","off");

axis equal
grid on

title(sprintf( ...
    "Pose Producing Minimum Horizontal Radius — %.1f mm", ...
    workspaceResult.closestToBaseAxis.horizontalDistance*1000));
