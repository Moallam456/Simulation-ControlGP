clear
clc
close all

%% Load UR5 reference configuration

robot = config.UR5();

%% Build robot

robotModel = robotmodel.buildRobot(robot);

%% Calculate workspace

numSamples = 10000;

workspaceResult = analysis.workspace( ...
    robotModel, ...
    robot, ...
    numSamples);

%% Display basic results

fprintf("Maximum sampled reach: %.3f m\n", ...
    workspaceResult.maxReach);

fprintf("Minimum sampled reach: %.3f m\n", ...
    workspaceResult.minReach);

fprintf("\nX range: %.3f to %.3f m\n", ...
    workspaceResult.xRange(1), ...
    workspaceResult.xRange(2));

fprintf("Y range: %.3f to %.3f m\n", ...
    workspaceResult.yRange(1), ...
    workspaceResult.yRange(2));

fprintf("Z range: %.3f to %.3f m\n", ...
    workspaceResult.zRange(1), ...
    workspaceResult.zRange(2));


%% Plot workspace

figure

scatter3( ...
    workspaceResult.points(:,1), ...
    workspaceResult.points(:,2), ...
    workspaceResult.points(:,3), ...
    3, ...
    ".");

xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

title("UR5 Sampled TCP Workspace")

axis equal
grid on