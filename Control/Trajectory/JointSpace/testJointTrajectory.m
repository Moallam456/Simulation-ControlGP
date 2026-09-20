%% testJointTrajectory
%
% Tests the joint-space trajectory generator:
%
%   qStart
%      ↓
%   Quintic joint-space trajectory
%      ↓
%   qGoal
%      ↓
%   Validate
%      ↓
%   Plot
%      ↓
%   Animate UR5


clear;
clc;
close all;


%% ========================================================================
% 1. PROJECT PATHS
% =========================================================================

thisFileFolder = fileparts(mfilename("fullpath"));

repoRoot = fileparts( ...
           fileparts( ...
           fileparts(thisFileFolder)));

addpath(repoRoot);

addpath(fullfile( ...
    repoRoot, ...
    "Control", ...
    "Trajectory", ...
    "JointSpace"));

addpath(fullfile( ...
    repoRoot, ...
    "Control", ...
    "Trajectory", ...
    "TimeScaling"));


%% ========================================================================
% 2. ROBOT
% =========================================================================

robot = config.UR5();

visualRobot = ...
    robotmodel.buildUR5geometry(robot);


%% ========================================================================
% 3. TEST CONFIGURATIONS
% =========================================================================

qStart = deg2rad([
     30
    -45
     60
     20
    -30
     45
]);


% Temporary HOME configuration for testing.
% This is NOT yet the final physical robot home position.

qGoal = deg2rad([
      0
    -90
     90
      0
      0
      0
]);


%% ========================================================================
% 4. TRAJECTORY SETTINGS
% =========================================================================

T  = 5.0;       % Total movement duration [s]
dt = 0.001;     % 1 ms control period


%% ========================================================================
% 5. GENERATE JOINT-SPACE TRAJECTORY
% =========================================================================

[t, q, qDot, qDDot, qDDDot, info] = ...
    generateJointTrajectory( ...
        qStart, ...
        qGoal, ...
        T, ...
        dt);


fprintf("\n");
fprintf("============================================================\n");
fprintf(" JOINT-SPACE TRAJECTORY TEST\n");
fprintf("============================================================\n");

fprintf("Profile:        %s\n", info.type);
fprintf("Duration:       %.3f s\n", info.duration);
fprintf("Samples:        %d\n", info.samples);
fprintf("Sampling time:  %.4f s\n", info.dt);


%% ========================================================================
% 6. ENDPOINT VALIDATION
% =========================================================================

startError = ...
    max(abs(q(1,:).' - qStart));

goalError = ...
    max(abs(q(end,:).' - qGoal));


fprintf("\nEndpoint validation\n");
fprintf("-------------------\n");

fprintf( ...
    "Start error: %.3e rad\n", ...
    startError);

fprintf( ...
    "Goal error:  %.3e rad\n", ...
    goalError);


%% ========================================================================
% 7. JOINT POSITION LIMIT CHECK
% =========================================================================

qMin = robot.limits.qMin(:);
qMax = robot.limits.qMax(:);

positionViolation = false(6,1);


for joint = 1:6

    positionViolation(joint) = ...
        any(q(:,joint) < qMin(joint)) || ...
        any(q(:,joint) > qMax(joint));

end


%% ========================================================================
% 8. JOINT VELOCITY LIMIT CHECK
% =========================================================================

peakJointVelocity = ...
    max(abs(qDot), [], 1).';


if ~isempty(robot.limits.qdMax)

    qdMax = ...
        robot.limits.qdMax(:);

    velocityViolation = ...
        peakJointVelocity > qdMax;

else

    qdMax = ...
        nan(6,1);

    velocityViolation = ...
        false(6,1);

end


%% ========================================================================
% 9. JOINT SUMMARY
% =========================================================================

fprintf("\n");
fprintf("Joint validation\n");
fprintf("----------------\n");


for joint = 1:6

    fprintf( ...
    ['J%d | start = %7.2f deg | goal = %7.2f deg | ' ...
     'peak velocity = %.3f rad/s | ' ...
     'position violation = %d | velocity violation = %d\n'], ...
    joint, ...
    rad2deg(qStart(joint)), ...
    rad2deg(qGoal(joint)), ...
    peakJointVelocity(joint), ...
    positionViolation(joint), ...
    velocityViolation(joint));
end


%% ========================================================================
% 10. JOINT POSITION PLOT
% =========================================================================

figure;

plot( ...
    t, ...
    rad2deg(q), ...
    "LineWidth", 1.2);

grid on;

xlabel("Time [s]");
ylabel("Joint Position [deg]");

legend( ...
    "J1", ...
    "J2", ...
    "J3", ...
    "J4", ...
    "J5", ...
    "J6", ...
    "Location", ...
    "best");

title("Joint-Space Trajectory — Position");


%% ========================================================================
% 11. JOINT VELOCITY PLOT
% =========================================================================

figure;

plot( ...
    t, ...
    qDot, ...
    "LineWidth", 1.2);

grid on;

xlabel("Time [s]");
ylabel("Joint Velocity [rad/s]");

legend( ...
    "J1", ...
    "J2", ...
    "J3", ...
    "J4", ...
    "J5", ...
    "J6", ...
    "Location", ...
    "best");

title("Joint-Space Trajectory — Velocity");


%% ========================================================================
% 12. JOINT ACCELERATION PLOT
% =========================================================================

figure;

plot( ...
    t, ...
    qDDot, ...
    "LineWidth", 1.2);

grid on;

xlabel("Time [s]");
ylabel("Joint Acceleration [rad/s^2]");

legend( ...
    "J1", ...
    "J2", ...
    "J3", ...
    "J4", ...
    "J5", ...
    "J6", ...
    "Location", ...
    "best");

title("Joint-Space Trajectory — Acceleration");


%% ========================================================================
% 13. CALCULATE TCP PATH FOR VISUALIZATION
% =========================================================================

numSamples = length(t);

tcpPath = ...
    zeros(3, numSamples);


for k = 1:numSamples

    Ttcp = ...
        controlFK( ...
            robot, ...
            q(k,:).' ...
        );

    tcpPath(:,k) = ...
        Ttcp(1:3,4);

end


%% ========================================================================
% 14. ROBOT ANIMATION
% =========================================================================

figure;

ax = axes;


show( ...
    visualRobot, ...
    q(1,:).', ...
    "Parent", ax, ...
    "FastUpdate", true, ...
    "PreservePlot", false, ...
    "Visuals", "on", ...
    "Collisions", "off");


hold(ax, "on");

grid(ax, "on");
axis(ax, "equal");


plot3( ...
    ax, ...
    tcpPath(1,:), ...
    tcpPath(2,:), ...
    tcpPath(3,:), ...
    "--", ...
    "LineWidth", 1.5);


xlabel(ax, "X [m]");
ylabel(ax, "Y [m]");
zlabel(ax, "Z [m]");


% Do not attempt to redraw all 5001 samples.
% Approximately 50 animation frames per second.

animationStride = ...
    max(1, round(0.02 / dt));


for k = 1:animationStride:numSamples

    show( ...
        visualRobot, ...
        q(k,:).', ...
        "Parent", ax, ...
        "FastUpdate", true, ...
        "PreservePlot", false, ...
        "Visuals", "on", ...
        "Collisions", "off");


    title( ...
        ax, ...
        sprintf( ...
            "Joint-Space Motion | t = %.2f / %.2f s", ...
            t(k), ...
            t(end)));


    drawnow;

end


% Explicitly display final configuration.

show( ...
    visualRobot, ...
    q(end,:).', ...
    "Parent", ax, ...
    "FastUpdate", true, ...
    "PreservePlot", false, ...
    "Visuals", "on", ...
    "Collisions", "off");


%% ========================================================================
% 15. FINAL RESULT
% =========================================================================

fprintf("\n");
fprintf("============================================================\n");
fprintf(" JOINT-SPACE TEST RESULT\n");
fprintf("============================================================\n");

fprintf( ...
    "Start configuration:      %s\n", ...
    passFail(startError < 1e-10));

fprintf( ...
    "Goal configuration:       %s\n", ...
    passFail(goalError < 1e-10));

fprintf( ...
    "Joint position limits:    %s\n", ...
    passFail(~any(positionViolation)));

fprintf( ...
    "Joint velocity limits:    %s\n", ...
    passFail(~any(velocityViolation)));

fprintf("============================================================\n");


%% ========================================================================
% LOCAL FUNCTION
% =========================================================================

function txt = passFail(tf)

if tf
    txt = "PASS";
else
    txt = "FAIL";
end

end