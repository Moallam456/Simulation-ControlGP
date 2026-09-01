function refRobot = buildRobot(robot)
%BUILDROBOT Build MATLAB reference robot from RobotConfig.
%
%   refRobot = reference.buildRobot(robot)
%
%   INPUT:
%       robot       Canonical RobotConfig structure
%
%   OUTPUT:
%       refRobot    MATLAB rigidBodyTree reference model
%
%   PURPOSE:
%       Convert the shared robot configuration into an independent
%       MATLAB Robotics System Toolbox model used by the Simulation team
%       for FK, IK, Jacobian, workspace, and validation.
%
%   IMPORTANT:
%       This is the Simulation reference model.
%       It is NOT the Control team's kinematic implementation.

%% ========================================================================
%  1. BASIC CONFIGURATION CHECKS
% =========================================================================

if robot.dof ~= 6
    error("reference:buildRobot:InvalidDOF", ...
        "This reference builder currently expects a 6-DOF robot.");
end

if robot.kinematics.convention ~= "Standard DH"
    error("reference:buildRobot:UnsupportedConvention", ...
        "This builder currently supports Standard DH only.");
end

% For the UR5 reference robot all theta offsets are zero.
% Non-zero offsets will be handled explicitly later rather than
% silently applying the wrong convention.
if any(abs(robot.kinematics.thetaOffset) > 1e-12)
    error("reference:buildRobot:ThetaOffsetNotSupported", ...
        "Non-zero theta offsets are not supported yet.");
end


%% ========================================================================
%  2. CREATE RIGID BODY TREE
% =========================================================================

% Six moving bodies + one fixed TCP body.
refRobot = rigidBodyTree( ...
    "DataFormat", "column", ...
    "MaxNumBodies", robot.dof + 1);

% Match our project frame naming convention.
refRobot.BaseName = char(robot.frames.base);


%% ========================================================================
%  3. BUILD THE SIX REVOLUTE JOINTS
% =========================================================================

for i = 1:robot.dof

    % --------------------------------------------------------------
    % Body name
    % --------------------------------------------------------------

    % Frame after J6 is treated as the robot flange frame.
    if i == robot.dof
        bodyName = char(robot.frames.flange);
    else
        bodyName = sprintf("Link%d", i);
    end

    % --------------------------------------------------------------
    % Create rigid body
    % --------------------------------------------------------------

    body = rigidBody(bodyName);

    % --------------------------------------------------------------
    % Create revolute joint
    % --------------------------------------------------------------

    jointName = char(robot.joints.names(i));

    joint = rigidBodyJoint(jointName, "revolute");

    % Apply joint limits from canonical RobotConfig.
    joint.PositionLimits = [ ...
        robot.limits.qMin(i), ...
        robot.limits.qMax(i)];

    % --------------------------------------------------------------
    % Standard DH parameters
    % --------------------------------------------------------------
    %
    % MATLAB expects:
    %
    % [a   alpha   d   theta]
    %
    % For a revolute joint, theta is supplied later by q(i), so the
    % theta value below is ignored by setFixedTransform.

    dhParameters = [ ...
        robot.kinematics.a(i), ...
        robot.kinematics.alpha(i), ...
        robot.kinematics.d(i), ...
        0];

    setFixedTransform(joint, dhParameters, "dh");

    % Assign joint to body.
    body.Joint = joint;


    %% ---------------------------------------------------------------
    %  ATTACH BODY TO ROBOT TREE
    % ----------------------------------------------------------------

    if i == 1

        % First body attaches to robot base.
        parentName = refRobot.BaseName;

    else

        % Every following body attaches to previous body.
        if i-1 == robot.dof
            parentName = char(robot.frames.flange);
        else
            parentName = sprintf("Link%d", i-1);
        end

    end

    addBody(refRobot, body, parentName);

end


%% ========================================================================
%  4. ADD TCP FRAME
% =========================================================================
%
% The six DH transformations end at the flange.
%
% We then independently define:
%
%       Flange --> TCP
%
% This is important because later our welding robot will have:
%
%       Flange --> TIG torch --> tungsten TCP

tcpBody = rigidBody(char(robot.frames.tcp));

tcpJoint = rigidBodyJoint("TCP_Fixed", "fixed");

setFixedTransform( ...
    tcpJoint, ...
    robot.tool.T_F_TCP);

tcpBody.Joint = tcpJoint;

addBody( ...
    refRobot, ...
    tcpBody, ...
    char(robot.frames.flange));

end
