function [robot, L] = buildRobotCAD()
% BUILDROBOTCAD
% Minimal 6-DOF robot model for the CAD/DH structure used by:
%   - controlFK(robot,q)
%   - analyticalIK_CAD(T0TCP,L,...)
%
% The geometric structure follows the CAD joint-axis arrangement:
%   J1 : base rotation
%   J2 : shoulder pitch, perpendicular to J1
%   J3 : elbow pitch, parallel to J2
%   J4 : forearm roll, perpendicular to J3
%   J5 : wrist pitch, perpendicular to J4
%   J6 : tool roll, perpendicular to J5
%
% IMPORTANT:
%   1) Frame 0 is a fixed base frame, NOT a joint.
%   2) The values below are temporary assumed dimensions for testing.
%   3) Change ONLY the L.l1 ... L.l10 block to update dimensions.
%   4) thetaOffset is set to zero because the real encoder/mechanical zero
%      offsets cannot be inferred from the CAD image alone.
%   5) This model uses MODIFIED DH:
%
%      A_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) ...
%            * Rz(theta_i) * Tz(d_i)
%
% Units:
%   lengths : metres
%   angles  : radians

%% ========================================================================
% EDITABLE ROBOT DIMENSIONS
% ========================================================================
% These are reasonable TEST values only. Replace with measured CAD values.

L.l1  = 0.180;   % base height
L.l2  = 0.045;   % J1-to-J2 lateral/axis offset
L.l3  = 0.080;   % base/shoulder horizontal offset
L.l4  = 0.420;   % upper-arm main length
L.l5  = 0.060;   % elbow/J4 axis offset
L.l6  = 0.310;   % forearm main length
L.l7  = 0.055;   % wrist offset part 1
L.l8  = 0.045;   % wrist offset part 2
L.l9  = 0.090;   % tool/TCP offset part 1
L.l10 = 0.110;   % tool/TCP offset part 2

%% ========================================================================
% BASIC ROBOT INFORMATION
% ========================================================================

robot.name = 'CAD_6R_TestRobot';
robot.dof  = 6;

robot.jointNames = { ...
    'J1 Base', ...
    'J2 Shoulder', ...
    'J3 Elbow', ...
    'J4 Forearm Roll', ...
    'J5 Wrist Pitch', ...
    'J6 Tool Roll'};

robot.kinematics.convention = 'modifiedDH';

%% ========================================================================
% MODIFIED-DH PARAMETERS
% ========================================================================
%
% Row i:
%   [a_(i-1), alpha_(i-1), d_i, thetaOffset_i]
%
%   i      a             alpha           d
%   ------------------------------------------------
%   1      0              0              l1
%   2      l3            -pi/2           l2
%   3      l4             0               0
%   4      l6            -pi/2           l5
%   5      0             +pi/2           l7+l8
%   6      0             -pi/2            0
%
% This gives the CAD axis relationships:
%   J1 ⟂ J2
%   J2 || J3
%   J3 ⟂ J4
%   J4 ⟂ J5
%   J5 ⟂ J6

robot.kinematics.a = [ ...
    0;
    L.l3;
    L.l4;
    L.l6;
    0;
    0];

robot.kinematics.alpha = [ ...
     0;
    -pi/2;
     0;
    -pi/2;
     pi/2;
    -pi/2];

robot.kinematics.d = [ ...
    L.l1;
    L.l2;
    0;
    L.l5;
    L.l7 + L.l8;
    0];

%% ========================================================================
% JOINT ZERO OFFSETS
% ========================================================================
% Unknown from the CAD image alone, therefore zero for the initial model.
% Replace these later after mechanical/encoder zero calibration.

robot.kinematics.thetaOffset = zeros(6,1);

%% ========================================================================
% TEMPORARY JOINT LIMITS FOR RANDOM TESTING
% ========================================================================
% These are NOT claimed to be the physical robot limits.
% They merely keep random tests in a sensible range.

robot.jointLimits = deg2rad([ ...
   -170,  170;   % J1
   -120,  120;   % J2
   -150,  150;   % J3
   -180,  180;   % J4
   -120,  120;   % J5
   -180,  180]); % J6

%% ========================================================================
% FIXED TOOL / TCP TRANSFORM
% ========================================================================
% The DH model places the TCP on +z6 by l9+l10.

robot.tool.T_F_TCP = eye(4);
robot.tool.T_F_TCP(3,4) = L.l9 + L.l10;

%% ========================================================================
% OPTIONAL METADATA FOR DEBUGGING
% ========================================================================

robot.lengths = L;

robot.notes = [ ...
    "CAD-axis test model. Dimensions are temporary assumptions. " ...
    + "thetaOffset and physical joint limits require later calibration." ];

end
