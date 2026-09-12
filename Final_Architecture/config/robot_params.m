function robot = robot_params()

% ==========================================================
% ROBOT PARAMETERS
% ==========================================================
% Single source of truth for robot properties.
% ==========================================================

%% Structure

robot.structure = robot_structure();

robot.name = "ARC_Mate_100iD_Test_Model";
robot.version = "v1";

%% =========================================================
%% GEOMETRY
%% =========================================================

% ---------- Physical Meaning ----------

robot.geometry.baseHeight      = 0.525;

robot.geometry.shoulderOffset  = 0.150;

robot.geometry.upperArmLength  = 0.770;

robot.geometry.forearmLength   = 0.100;

robot.geometry.wristOffset     = 0.740;

robot.geometry.flangeLength    = 0.100;

% ---------- Names matching IK ----------

robot.geometry.a1 = robot.geometry.shoulderOffset;
robot.geometry.a2 = robot.geometry.upperArmLength;
robot.geometry.a3 = robot.geometry.forearmLength;

robot.geometry.d1 = robot.geometry.baseHeight;
robot.geometry.d4 = robot.geometry.wristOffset;

%% =========================================================
%% JOINT LIMITS
%% =========================================================

qMinDeg = [ ...
   -170 ...
   -117.5 ...
   -227.5 ...
   -190 ...
   -180 ...
   -450 ];

qMaxDeg = [ ...
    170 ...
    117.5 ...
    227.5 ...
    190 ...
    180 ...
    450 ];

for i = 1:6

    robot.joints(i).hardLimitMin = ...
        deg2rad(qMinDeg(i));

    robot.joints(i).hardLimitMax = ...
        deg2rad(qMaxDeg(i));

    % temporarily equal to hard limits

    robot.joints(i).softLimitMin = ...
        robot.joints(i).hardLimitMin;

    robot.joints(i).softLimitMax = ...
        robot.joints(i).hardLimitMax;

end

%% =========================================================
%% VELOCITY LIMITS
%% =========================================================

for i = 1:6

    robot.joints(i).maxVelocity = NaN;

end

%% =========================================================
%% ACCELERATION LIMITS
%% =========================================================

for i = 1:6

    robot.joints(i).maxAcceleration = NaN;

end

%% =========================================================
%% MASS PROPERTIES
%% =========================================================

for i = 1:6

    robot.links(i).mass = NaN;

    robot.links(i).com = [NaN NaN NaN];

    robot.links(i).inertia = [
        NaN NaN NaN
        NaN NaN NaN
        NaN NaN NaN];

end

%% =========================================================
%% PAYLOAD
%% =========================================================

robot.payload.mass = 2.0;

robot.payload.com = [0 0 0.05];

%% =========================================================
%% TOOL
%% =========================================================

robot.tool.name = "TIG_Torch";

robot.tool.mass = 0.8;

robot.tool.length = 0.20;

%% TCP Transform (Flange -> TCP)

robot.tool.T_F_TCP = eye(4);

robot.tool.T_F_TCP(3,4) = ...
    robot.tool.length;

robot.geometry.dTool = ...
    robot.tool.length;

%% =========================================================
%% COLLISION PLACEHOLDERS
%% =========================================================

for i = 1:6

    robot.links(i).collisionRadius = NaN;

end

%% =========================================================
%% REACH ESTIMATE
%% =========================================================

robot.performance.nominalReach = ...
    robot.geometry.a2 + ...
    robot.geometry.d4 + ...
    robot.tool.length;

%% ========================================