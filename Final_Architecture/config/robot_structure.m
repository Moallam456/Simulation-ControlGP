function robotStruct = robot_structure()

% ==========================================================
% ROBOT STRUCTURE
% ==========================================================
% Describes robot topology only.
% No dimensions, masses, limits, payloads, CAD properties.
% ==========================================================

%% General

robotStruct.name = "Generic_6DOF_Welding_Robot";

robotStruct.numJoints = 6;

%% Joint Configuration

% R = Revolute
robotStruct.jointTypes = ["R","R","R","R","R","R"];

%% Parent-Child Relationship

robotStruct.parentLinks = [0 1 2 3 4 5];

%% Joint Axes
%
% MODIFY TO MATCH FINAL MECHANICAL DESIGN
%

robotStruct.jointAxes = [
     0 0 1;    % J1
     0 1 0;    % J2
     0 1 0;    % J3
     1 0 0;    % J4
     0 1 0;    % J5
     1 0 0];   % J6

%% Configuration Type

robotStruct.configuration = ...
    "Industrial_Articulated_6DOF";

%% Wrist Information

robotStruct.hasSphericalWrist = true;

%% Home Configuration

robotStruct.homePosition = deg2rad( ...
    [0 -90 90 0 0 0]);

%% Notes

robotStruct.description = ...
    "6-DOF industrial welding robot";

end