function robot = UR5()
%UR5 Canonical configuration of the original Universal Robots UR5.
%
%   robot = config.UR5()
%
%   Purpose:
%   --------
%   This file defines the common robot configuration used by:
%       1. Simulation reference models
%       2. Control-team FK / IK / Jacobian implementations
%       3. Validation scripts
%
%   IMPORTANT:
%   This file contains robot DATA only.
%   It must not perform FK, IK, Jacobian, dynamics, plotting, or simulation.
%
%   Project conventions:
%       Length              : metre [m]
%       Joint angle         : radian [rad]
%       Joint velocity      : rad/s
%       Coordinate system   : right-handed
%       Joint vector        : 6x1 column vector
%       DH convention       : Standard Denavit-Hartenberg
%
%   Reference:
%   Universal Robots official UR5 DH parameter documentation.

%% ========================================================================
%  1. ROBOT IDENTITY
% =========================================================================

robot.name = "UR5";
robot.manufacturer = "Universal Robots";
robot.role = "Reference Robot";

robot.dof = 6;


%% ========================================================================
%  2. PROJECT CONVENTIONS
% =========================================================================

robot.conventions.coordinateSystem = "right-handed";

robot.conventions.lengthUnit = "m";
robot.conventions.angleUnit = "rad";
robot.conventions.angularVelocityUnit = "rad/s";

robot.conventions.jointVectorShape = "6x1";

% Transformation notation used throughout the project:
%
% T_A_B = pose of frame B expressed in frame A
%
% Example:
% T_B_TCP = pose of TCP expressed in the robot Base frame

robot.conventions.transformNotation = ...
    "T_A_B = pose of frame B expressed in frame A";


%% ========================================================================
%  3. JOINT DEFINITIONS
% =========================================================================

robot.joints.names = [
    "J1_Base"
    "J2_Shoulder"
    "J3_Elbow"
    "J4_Wrist1"
    "J5_Wrist2"
    "J6_Wrist3"
];

robot.joints.type = repmat("revolute", robot.dof, 1);

% All joint vectors must follow this order:
%
% q = [q1
%      q2
%      q3
%      q4
%      q5
%      q6]

robot.joints.order = (1:robot.dof).';


%% ========================================================================
%  4. STANDARD DH KINEMATIC PARAMETERS
% =========================================================================
%
% For each revolute joint:
%
% theta_i = q_i + thetaOffset_i
%
% Standard DH transformation:
%
% A_i =
% RotZ(theta_i) *
% TransZ(d_i) *
% TransX(a_i) *
% RotX(alpha_i)
%
% Units:
% a, d     -> metres
% alpha    -> radians

robot.kinematics.convention = "Standard DH";

robot.kinematics.a = [
     0
    -0.425
    -0.39225
     0
     0
     0
];

robot.kinematics.d = [
    0.089159
    0
    0
    0.10915
    0.09465
    0.0823
];

robot.kinematics.alpha = [
     pi/2
     0
     0
     pi/2
    -pi/2
     0
];

robot.kinematics.thetaOffset = zeros(robot.dof,1);


%% ========================================================================
%  5. JOINT LIMITS
% =========================================================================

% Original UR5 published working range:
% +/- 360 degrees for all six joints.

robot.limits.qMin = -2*pi * ones(robot.dof,1);
robot.limits.qMax =  2*pi * ones(robot.dof,1);

% Published maximum joint speed:
% 180 deg/s = pi rad/s

robot.limits.qdMax = pi * ones(robot.dof,1);

% Do not assume acceleration limits yet.
robot.limits.qddMax = [];


%% ========================================================================
%  6. REFERENCE CONFIGURATIONS
% =========================================================================

% Mathematical zero configuration.
robot.configuration.zero = zeros(robot.dof,1);

% We will define a useful "home" pose separately once we begin
% visualization and workspace testing.
robot.configuration.home = [];


%% ========================================================================
%  7. FRAME DEFINITIONS
% =========================================================================

robot.frames.base = "Base";
robot.frames.flange = "Flange";
robot.frames.tcp = "TCP";


%% ========================================================================
%  8. TOOL / TCP DEFINITION
% =========================================================================

% Initially the TCP is assumed to coincide with the UR5 flange.
%
% Later, for our welding robot:
%
% flange -> TIG torch mounting -> tungsten TCP
%
% will be represented here.

robot.tool.T_F_TCP = eye(4);


%% ========================================================================
%  9. COMMERCIAL REFERENCE PERFORMANCE
% =========================================================================

robot.performance.reach = 0.850;          % [m]
robot.performance.payload = 5.0;          % [kg]
robot.performance.repeatability = 0.0001; % +/- 0.1 mm [m]
robot.performance.maxJointSpeed = pi;     % [rad/s]


%% ========================================================================
%  10. SOURCE METADATA
% =========================================================================

robot.source.geometry = ...
    "Universal Robots official DH parameter documentation";

robot.source.performance = ...
    "Universal Robots original UR5 technical specification";

robot.source.parameterStatus = "Commercial reference data";

end