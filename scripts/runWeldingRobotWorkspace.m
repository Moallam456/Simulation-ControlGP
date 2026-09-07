clear
clc
close all

%% ========================================================================
%  1. ROBOT INSTALLATION / BASE LOCATION
% =========================================================================
%
% The robot Base frame is currently used as the reference frame.
%
% Robot Base:
%
%       X = 0 m
%       Y = 0 m
%       Z = 0 m
%
% Coordinate convention:
%
%       +X -> forward from the robot
%       +Y -> lateral direction
%       +Z -> upward
%
% Z = 0 represents the robot mounting plane.
%
% IMPORTANT:
% This is currently a preliminary installation assumption.
% Later, a Station/World frame can be introduced so that the robot base
% position itself can be varied relative to the welding table.

station.robotBasePosition = [0; 0; 0];    % [m]


%% ========================================================================
%  2. REQUIRED WELDING REGION
% =========================================================================
%
% The welding region is defined relative to the Robot Base frame.
%
% Instead of entering arbitrary X/Y/Z limits directly, the workspace is
% defined using:
%
%       Workspace center
%       Workspace dimensions
%
% This makes the physical meaning easier to understand.

% -------------------------------------------------------------------------
% Center of required welding region
% -------------------------------------------------------------------------

workspaceReq.centerX = 0.55;   % 550 mm forward from robot base
workspaceReq.centerY = 0.00;   % centered laterally
workspaceReq.centerZ = 0.30;   % 300 mm above mounting plane


% -------------------------------------------------------------------------
% Size of required welding region
% -------------------------------------------------------------------------

workspaceReq.sizeX = 0.40;     % 400 mm
workspaceReq.sizeY = 0.40;     % 400 mm
workspaceReq.sizeZ = 0.20;     % 200 mm


% -------------------------------------------------------------------------
% Convert center + size into Cartesian limits
% -------------------------------------------------------------------------

workspaceReq.xMin = ...
    workspaceReq.centerX - workspaceReq.sizeX/2;

workspaceReq.xMax = ...
    workspaceReq.centerX + workspaceReq.sizeX/2;

workspaceReq.yMin = ...
    workspaceReq.centerY - workspaceReq.sizeY/2;

workspaceReq.yMax = ...
    workspaceReq.centerY + workspaceReq.sizeY/2;

workspaceReq.zMin = ...
    workspaceReq.centerZ - workspaceReq.sizeZ/2;

workspaceReq.zMax = ...
    workspaceReq.centerZ + workspaceReq.sizeZ/2;


%% ========================================================================
%  3. DEFINE WELDING ROBOT CANDIDATE
% =========================================================================

% -------------------------------------------------------------------------
% Preliminary fixed base offset
% -------------------------------------------------------------------------

geometry.d1 = 0.089159;


% -------------------------------------------------------------------------
% MAIN DESIGN VARIABLES
% -------------------------------------------------------------------------
%
% During the initial geometry study, change ONLY:
%
%       L2 = Shoulder -> Elbow
%       L3 = Elbow -> Wrist
%
% Example candidate:
%
%       L2 = 425 mm
%       L3 = 375 mm
%
%       Total main-link length = 800 mm

geometry.L2 = 0.425;
geometry.L3 = 0.375;


% -------------------------------------------------------------------------
% Preliminary fixed wrist offsets
% -------------------------------------------------------------------------
%
% These currently use UR5 reference values only as preliminary geometry.
%
% They are NOT final welding-robot dimensions.

geometry.d4 = 0.10915;
geometry.d5 = 0.09465;
geometry.d6 = 0.0823;


%% ========================================================================
%  4. CREATE WELDING ROBOT CONFIGURATION
% =========================================================================

robot = config.WeldingRobot(geometry);


%% ========================================================================
%  5. BUILD MATLAB ROBOT MODEL
% =========================================================================

robotModel = robotmodel.buildRobot(robot);


%% ========================================================================
%  6. WORKSPACE SAMPLING SETTINGS
% =========================================================================
%
% Use a fixed random seed so that candidate geometries are compared under
% repeatable random sampling conditions.

rng(1);

numSamples = 10000;


%% ========================================================================
%  7. CALCULATE SAMPLED ROBOT WORKSPACE
% =========================================================================

workspaceResult = analysis.workspace( ...
    robotModel, ...
    robot, ...
    numSamples);


%% ========================================================================
%  8. CALCULATE REQUIRED WORKSPACE REACHABILITY
% =========================================================================
%
% Reachability is evaluated independently using position-only IK.
%
% This is more meaningful than deciding whether a required point happens
% to lie near one of the random workspace samples.

reachabilityResult = analysis.reachability( ...
    robotModel, ...
    robot, ...
    workspaceReq);


%% ========================================================================
%  9. CALCULATE WORKSPACE SPANS
% =========================================================================

xSpan = ...
    workspaceResult.xRange(2) - ...
    workspaceResult.xRange(1);

ySpan = ...
    workspaceResult.yRange(2) - ...
    workspaceResult.yRange(1);

zSpan = ...
    workspaceResult.zRange(2) - ...
    workspaceResult.zRange(1);


%% ========================================================================
%  10. DISPLAY ROBOT INSTALLATION
% =========================================================================

fprintf("\n");
fprintf("============================================================\n");
fprintf("WELDING ROBOT WORKSPACE ANALYSIS\n");
fprintf("============================================================\n");

fprintf("\nROBOT INSTALLATION\n");
fprintf("----------------------------\n");

fprintf("Robot Base X = %.3f m\n", ...
    station.robotBasePosition(1));

fprintf("Robot Base Y = %.3f m\n", ...
    station.robotBasePosition(2));

fprintf("Robot Base Z = %.3f m\n", ...
    station.robotBasePosition(3));


%% ========================================================================
%  11. DISPLAY ROBOT GEOMETRY
% =========================================================================

fprintf("\nROBOT GEOMETRY\n");
fprintf("----------------------------\n");

fprintf("L2 = %.0f mm\n", ...
    robot.geometry.L2 * 1000);

fprintf("L3 = %.0f mm\n", ...
    robot.geometry.L3 * 1000);

fprintf("L2 + L3 = %.0f mm\n", ...
    (robot.geometry.L2 + robot.geometry.L3) * 1000);

fprintf("L2 / L3 = %.3f\n", ...
    robot.geometry.L2 / robot.geometry.L3);


%% ========================================================================
%  12. DISPLAY REQUIRED WELDING REGION
% =========================================================================

fprintf("\nREQUIRED WELDING REGION\n");
fprintf("----------------------------\n");

fprintf("Center = [%.3f, %.3f, %.3f] m\n", ...
    workspaceReq.centerX, ...
    workspaceReq.centerY, ...
    workspaceReq.centerZ);

fprintf("Size   = %.0f x %.0f x %.0f mm\n", ...
    workspaceReq.sizeX * 1000, ...
    workspaceReq.sizeY * 1000, ...
    workspaceReq.sizeZ * 1000);

fprintf("\nX range = %.3f to %.3f m\n", ...
    workspaceReq.xMin, ...
    workspaceReq.xMax);

fprintf("Y range = %.3f to %.3f m\n", ...
    workspaceReq.yMin, ...
    workspaceReq.yMax);

fprintf("Z range = %.3f to %.3f m\n", ...
    workspaceReq.zMin, ...
    workspaceReq.zMax);


%% ========================================================================
%  13. DISPLAY SAMPLED ROBOT WORKSPACE
% =========================================================================

fprintf("\nSAMPLED ROBOT WORKSPACE\n");
fprintf("----------------------------\n");

fprintf("Number of samples = %d\n", ...
    workspaceResult.numSamples);

fprintf("Maximum sampled reach = %.4f m\n", ...
    workspaceResult.maxReach);

fprintf("Minimum sampled reach = %.4f m\n", ...
    workspaceResult.minReach);

fprintf("\nX range = %.4f to %.4f m\n", ...
    workspaceResult.xRange(1), ...
    workspaceResult.xRange(2));

fprintf("Y range = %.4f to %.4f m\n", ...
    workspaceResult.yRange(1), ...
    workspaceResult.yRange(2));

fprintf("Z range = %.4f to %.4f m\n", ...
    workspaceResult.zRange(1), ...
    workspaceResult.zRange(2));


%% ========================================================================
%  14. DISPLAY WORKSPACE SPANS
% =========================================================================

fprintf("\nWORKSPACE SPANS\n");
fprintf("----------------------------\n");

fprintf("X span = %.4f m\n", xSpan);
fprintf("Y span = %.4f m\n", ySpan);
fprintf("Z span = %.4f m\n", zSpan);


%% ========================================================================
%  15. DISPLAY REACHABILITY RESULTS
% =========================================================================

fprintf("\nREQUIRED WORKSPACE REACHABILITY\n");
fprintf("----------------------------\n");

fprintf("Coverage = %.2f %%\n", ...
    reachabilityResult.coveragePercent);

fprintf("Reachable points = %d / %d\n", ...
    reachabilityResult.numReachablePoints, ...
    reachabilityResult.numRequiredPoints);

fprintf("Unreachable points = %d\n", ...
    reachabilityResult.numUnreachablePoints);

fprintf("\n============================================================\n");


%% ========================================================================
%  16. PLOT COMPLETE ROBOT WORKSPACE
% =========================================================================

figure
hold on

% -------------------------------------------------------------------------
% Sampled TCP workspace
% -------------------------------------------------------------------------

scatter3( ...
    workspaceResult.points(:,1), ...
    workspaceResult.points(:,2), ...
    workspaceResult.points(:,3), ...
    3, ...
    ".");


% -------------------------------------------------------------------------
% Robot base position
% -------------------------------------------------------------------------

scatter3( ...
    station.robotBasePosition(1), ...
    station.robotBasePosition(2), ...
    station.robotBasePosition(3), ...
    150, ...
    "filled");

text( ...
    station.robotBasePosition(1), ...
    station.robotBasePosition(2), ...
    station.robotBasePosition(3), ...
    "  Robot Base", ...
    "FontWeight", ...
    "bold");


xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

title(sprintf( ...
    "Sampled Robot Workspace - L2 = %.0f mm, L3 = %.0f mm", ...
    robot.geometry.L2 * 1000, ...
    robot.geometry.L3 * 1000));

legend( ...
    "Robot TCP Workspace", ...
    "Robot Base", ...
    "Location", ...
    "best");

axis equal
grid on

hold off


%% ========================================================================
%  17. SEPARATE REACHABLE AND UNREACHABLE REQUIRED POINTS
% =========================================================================

reachablePoints = ...
    reachabilityResult.requiredPoints( ...
    reachabilityResult.reachableMask, :);

unreachablePoints = ...
    reachabilityResult.requiredPoints( ...
    ~reachabilityResult.reachableMask, :);


%% ========================================================================
%  18. PLOT REQUIRED WELDING WORKSPACE COVERAGE
% =========================================================================

figure
hold on


% -------------------------------------------------------------------------
% Sampled robot workspace
% -------------------------------------------------------------------------

scatter3( ...
    workspaceResult.points(:,1), ...
    workspaceResult.points(:,2), ...
    workspaceResult.points(:,3), ...
    3, ...
    ".");


% -------------------------------------------------------------------------
% Robot base
% -------------------------------------------------------------------------

scatter3( ...
    station.robotBasePosition(1), ...
    station.robotBasePosition(2), ...
    station.robotBasePosition(3), ...
    150, ...
    "filled");

text( ...
    station.robotBasePosition(1), ...
    station.robotBasePosition(2), ...
    station.robotBasePosition(3), ...
    "  Robot Base", ...
    "FontWeight", ...
    "bold");


% -------------------------------------------------------------------------
% Reachable required points
% -------------------------------------------------------------------------

if ~isempty(reachablePoints)

    scatter3( ...
        reachablePoints(:,1), ...
        reachablePoints(:,2), ...
        reachablePoints(:,3), ...
        25, ...
        "o");

end


% -------------------------------------------------------------------------
% Unreachable required points
% -------------------------------------------------------------------------

if ~isempty(unreachablePoints)

    scatter3( ...
        unreachablePoints(:,1), ...
        unreachablePoints(:,2), ...
        unreachablePoints(:,3), ...
        40, ...
        "x");

end


xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

title(sprintf( ...
    "Required Workspace Coverage = %.1f%%", ...
    reachabilityResult.coveragePercent));

legend( ...
    "Robot Sampled Workspace", ...
    "Robot Base", ...
    "Reachable Required Points", ...
    "Unreachable Required Points", ...
    "Location", ...
    "best");

axis equal
grid on

hold off


%% ========================================================================
%  19. SHOW ROBOT IN ZERO CONFIGURATION
% =========================================================================
%
% This third figure makes the actual location of the robot relative to its
% Base frame visually obvious.
%
% The rigidBodyTree Base itself is located at [0,0,0].

qZero = zeros(robot.dof,1);

figure

show( ...
    robotModel, ...
    qZero, ...
    "Frames", ...
    "on", ...
    "Visuals", ...
    "off");

hold on

scatter3( ...
    station.robotBasePosition(1), ...
    station.robotBasePosition(2), ...
    station.robotBasePosition(3), ...
    150, ...
    "filled");

text( ...
    station.robotBasePosition(1), ...
    station.robotBasePosition(2), ...
    station.robotBasePosition(3), ...
    "  Robot Base [0,0,0]", ...
    "FontWeight", ...
    "bold");

xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

title("Welding Robot - Zero Configuration")

axis equal
grid on

hold off