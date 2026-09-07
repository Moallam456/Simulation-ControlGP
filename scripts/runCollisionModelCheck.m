clear
clc
close all

%% ========================================================================
%  1. DEFINE WELDING ROBOT GEOMETRY
% =========================================================================
%
% Preliminary geometry for collision-model checking.
%
% L2 and L3 are the main design variables.
% d1, d4, d5, d6 currently use preliminary UR5-scale values.

geometry.d1 = 0.089159;

geometry.L2 = 0.425;    % Shoulder -> Elbow [m]
geometry.L3 = 0.375;    % Elbow -> Wrist [m]

geometry.d4 = 0.10915;
geometry.d5 = 0.09465;
geometry.d6 = 0.0823;


%% ========================================================================
%  2. CREATE ROBOT CONFIGURATION
% =========================================================================

robot = config.WeldingRobot(geometry);


%% ========================================================================
%  3. BUILD KINEMATIC ROBOT MODEL
% =========================================================================

robotModel = robotmodel.buildRobot(robot);


%% ========================================================================
%  4. ADD PRELIMINARY COLLISION GEOMETRY
% =========================================================================

collisionRobot = ...
    robotmodel.addPreliminaryCollisionGeometry( ...
        robotModel, ...
        robot);


%% ========================================================================
%  5. DISPLAY BASIC MODEL INFORMATION
% =========================================================================

fprintf("\n");
fprintf("============================================================\n");
fprintf("PRELIMINARY COLLISION MODEL CHECK\n");
fprintf("============================================================\n");

fprintf("\nRobot geometry:\n");
fprintf("L2 = %.0f mm\n",geometry.L2*1000);
fprintf("L3 = %.0f mm\n",geometry.L3*1000);

fprintf("\nCollision model:\n");

fprintf("Base radius       = %.1f mm\n", ...
    robot.collision.baseRadius*1000);

fprintf("Base height       = %.1f mm\n", ...
    robot.collision.baseHeight*1000);

fprintf("Upper-arm radius  = %.1f mm\n", ...
    robot.collision.upperArmRadius*1000);

fprintf("Forearm radius    = %.1f mm\n", ...
    robot.collision.forearmRadius*1000);

fprintf("Shoulder radius   = %.1f mm\n", ...
    robot.collision.shoulderRadius*1000);

fprintf("Elbow radius      = %.1f mm\n", ...
    robot.collision.elbowRadius*1000);

fprintf("Wrist 1 radius    = %.1f mm\n", ...
    robot.collision.wrist1Radius*1000);

fprintf("Wrist 2 radius    = %.1f mm\n", ...
    robot.collision.wrist2Radius*1000);

fprintf("Wrist 3 radius    = %.1f mm\n", ...
    robot.collision.wrist3Radius*1000);

fprintf("\n============================================================\n");


%% ========================================================================
%  6. ZERO CONFIGURATION
% =========================================================================

qZero = zeros(robot.dof,1);


%% ========================================================================
%  7. SHOW KINEMATIC MODEL ONLY
% =========================================================================
%
% This figure shows the original mathematical robot without physical
% collision envelopes.

figure

show( ...
    robotModel, ...
    qZero, ...
    "Visuals","off", ...
    "Collisions","off", ...
    "Frames","on");

xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

title("Kinematic Robot Model - Zero Configuration")

axis equal
grid on


%% ========================================================================
%  8. SHOW COLLISION MODEL
% =========================================================================
%
% This is the important figure.
%
% Check visually that:
%
% 1. Base cylinder is located around the robot base.
% 2. Shoulder sphere is around the shoulder joint.
% 3. Upper-arm capsule follows L2.
% 4. Elbow sphere is around the elbow.
% 5. Forearm capsule follows L3.
% 6. Wrist envelopes are located around the wrist.
% 7. Flange envelope is around the final flange.

figure

show( ...
    collisionRobot, ...
    qZero, ...
    "Visuals","off", ...
    "Collisions","on", ...
    "Frames","on");

xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

title("Preliminary Collision Model - Zero Configuration")

axis equal
grid on


%% ========================================================================
%  9. TEST A NON-ZERO CONFIGURATION
% =========================================================================
%
% A non-zero configuration makes it easier to verify whether the collision
% bodies actually move with the correct links.

qTest = deg2rad([
     30
    -45
     60
     20
    -30
     45
]);


figure

show( ...
    collisionRobot, ...
    qTest, ...
    "Visuals","off", ...
    "Collisions","on", ...
    "Frames","on");

xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

title("Preliminary Collision Model - Test Configuration")

axis equal
grid on


%% ========================================================================
%  10. CHECK SELF-COLLISION AT ZERO CONFIGURATION
% =========================================================================
%
% Adjacent bodies are intentionally skipped because mechanically connected
% links naturally touch at their joints.

isCollidingZero = checkCollision( ...
    collisionRobot, ...
    qZero, ...
    SkippedSelfCollisions="parent", ...
    Exhaustive="on");


fprintf("\nZERO-CONFIGURATION COLLISION CHECK\n");
fprintf("---------------------------------\n");

if isCollidingZero

    fprintf("Collision detected.\n");

else

    fprintf("No self-collision detected.\n");

end


%% ========================================================================
%  11. CHECK SELF-COLLISION AT TEST CONFIGURATION
% =========================================================================

isCollidingTest = checkCollision( ...
    collisionRobot, ...
    qTest, ...
    SkippedSelfCollisions="parent", ...
    Exhaustive="on");


fprintf("\nTEST-CONFIGURATION COLLISION CHECK\n");
fprintf("---------------------------------\n");

if isCollidingTest

    fprintf("Collision detected.\n");

else

    fprintf("No self-collision detected.\n");

end


%% ========================================================================
%  12. SHOW TEST CONFIGURATION WITH COLLISION STATUS
% =========================================================================

figure

show( ...
    collisionRobot, ...
    qTest, ...
    "Visuals","off", ...
    "Collisions","on", ...
    "Frames","on");

xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

if isCollidingTest

    title("Test Configuration - COLLISION DETECTED");

else

    title("Test Configuration - Collision Free");

end

axis equal
grid on