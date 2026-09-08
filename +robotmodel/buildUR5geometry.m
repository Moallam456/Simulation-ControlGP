function visualRobot = buildUR5geometry(robot)
%BUILDUR5GEOMETRY Build a visually realistic UR5 rigidBodyTree.
%
%   visualRobot = robotmodel.buildUR5geometry(robot)
%
% PURPOSE
% -------
% Build a MATLAB rigidBodyTree for VISUALIZATION using the official
% Universal Robots UR5 meshes:
%
%       base
%       shoulder
%       upper arm
%       forearm
%       wrist 1
%       wrist 2
%       wrist 3
%
% Both:
%
%       visual DAE meshes
%
% and:
%
%       collision STL meshes
%
% are attached.
%
%
% IMPORTANT
% ---------
% This function is intended primarily as the VISUALIZATION model.
%
% The Control team's:
%
%       controlFK
%       controlJacobian
%       ADLS_IK
%
% remain the authoritative algorithms used to calculate motion.
%
% This robot is used to display:
%
%       q(t)
%
% produced by the trajectory pipeline.
%
%
% SOURCE OF GEOMETRY
% ------------------
% Joint origins and orientations:
%
%       Universal Robots ROS2 Description
%       config/ur5/default_kinematics.yaml
%
% Mesh origins and orientations:
%
%       config/ur5/visual_parameters.yaml
%
%
% ASSET FOLDER EXPECTED
% ---------------------
%
% +robotmodel/
%     buildUR5geometry.m
%
%     UR5Assets/
%         visual/
%             base.dae
%             shoulder.dae
%             upperarm.dae
%             forearm.dae
%             wrist1.dae
%             wrist2.dae
%             wrist3.dae
%
%         collision/
%             base.stl
%             shoulder.stl
%             upperarm.stl
%             forearm.stl
%             wrist1.stl
%             wrist2.stl
%             wrist3.stl
%
%
% The function also accepts the current folder name:
%
%       "UR5 assets"
%
% so you do NOT have to rename it immediately.


%% ========================================================================
% 1. INPUT
% =========================================================================

if nargin < 1 || isempty(robot)

    robot = config.UR5();

end


if robot.dof ~= 6

    error( ...
        "buildUR5geometry currently supports only the 6-DOF UR5.");

end


%% ========================================================================
% 2. LOCATE UR5 MESH ASSETS
% =========================================================================
%
% mfilename("fullpath") returns the path to this function:
%
%       .../+robotmodel/buildUR5geometry.m
%
% Therefore:
%
%       fileparts(...)
%
% gives:
%
%       .../+robotmodel

thisFolder = fileparts(mfilename("fullpath"));


% First try the recommended folder name.

assetRoot = fullfile( ...
    thisFolder, ...
    "UR5Assets");


% Your current folder is called "UR5 assets".
% Support that as a fallback.

if ~isfolder(assetRoot)

    assetRoot = fullfile( ...
        thisFolder, ...
        "UR5 assets");

end


if ~isfolder(assetRoot)

    error( ...
        ['UR5 asset folder not found.\n' ...
         'Expected either:\n\n' ...
         '    +robotmodel/UR5Assets\n\n' ...
         'or:\n\n' ...
         '    +robotmodel/UR5 assets']);

end


visualFolder = fullfile( ...
    assetRoot, ...
    "visual");


collisionFolder = fullfile( ...
    assetRoot, ...
    "collision");


%% ========================================================================
% 3. DEFINE FILE PATHS
% =========================================================================

visualFiles.base = ...
    fullfile(visualFolder,"base.dae");

visualFiles.shoulder = ...
    fullfile(visualFolder,"shoulder.dae");

visualFiles.upperArm = ...
    fullfile(visualFolder,"upperarm.dae");

visualFiles.forearm = ...
    fullfile(visualFolder,"forearm.dae");

visualFiles.wrist1 = ...
    fullfile(visualFolder,"wrist1.dae");

visualFiles.wrist2 = ...
    fullfile(visualFolder,"wrist2.dae");

visualFiles.wrist3 = ...
    fullfile(visualFolder,"wrist3.dae");


collisionFiles.base = ...
    fullfile(collisionFolder,"base.stl");

collisionFiles.shoulder = ...
    fullfile(collisionFolder,"shoulder.stl");

collisionFiles.upperArm = ...
    fullfile(collisionFolder,"upperarm.stl");

collisionFiles.forearm = ...
    fullfile(collisionFolder,"forearm.stl");

collisionFiles.wrist1 = ...
    fullfile(collisionFolder,"wrist1.stl");

collisionFiles.wrist2 = ...
    fullfile(collisionFolder,"wrist2.stl");

collisionFiles.wrist3 = ...
    fullfile(collisionFolder,"wrist3.stl");


%% ========================================================================
% 4. VERIFY FILES EXIST
% =========================================================================

visualFileList = struct2cell(visualFiles);

collisionFileList = struct2cell(collisionFiles);


for i = 1:numel(visualFileList)

    if ~isfile(visualFileList{i})

        error( ...
            "Missing UR5 visual mesh:\n%s", ...
            visualFileList{i});

    end

end


for i = 1:numel(collisionFileList)

    if ~isfile(collisionFileList{i})

        error( ...
            "Missing UR5 collision mesh:\n%s", ...
            collisionFileList{i});

    end

end


%% ========================================================================
% 5. CREATE RIGID BODY TREE
% =========================================================================
%
% Bodies:
%
%       1 BaseVisual
%       2 Shoulder
%       3 UpperArm
%       4 Forearm
%       5 Wrist1
%       6 Wrist2
%       7 Wrist3
%       8 Flange
%       9 TCP
%
%
% Six of these contain revolute joints.

visualRobot = rigidBodyTree( ...
    "DataFormat","column", ...
    "MaxNumBodies",9);


visualRobot.BaseName = char(robot.frames.base);


%% ========================================================================
% 6. ROBOT JOINT LIMITS
% =========================================================================

qMin = robot.limits.qMin(:);

qMax = robot.limits.qMax(:);


%% ========================================================================
% 7. FIXED BASE GEOMETRY
% =========================================================================
%
% The physical base itself must NOT rotate with Joint 1.
%
% Therefore the base visual is placed on a separate FIXED body attached
% directly to the rigidBodyTree base.
%
%
% Official mesh offset:
%
%       translation = [0 0 0]
%
%       roll  =   0 deg
%       pitch =   0 deg
%       yaw   = 180 deg

baseBody = rigidBody("UR5BaseVisual");

baseJoint = rigidBodyJoint( ...
    "UR5BaseVisualFixed", ...
    "fixed");

setFixedTransform( ...
    baseJoint, ...
    eye(4));

baseBody.Joint = baseJoint;


TMeshBase = xyzRPYTransform( ...
    [0 0 0], ...
    [0 0 pi]);


addVisual( ...
    baseBody, ...
    "mesh", ...
    visualFiles.base, ...
    TMeshBase);


addCollision( ...
    baseBody, ...
    "mesh", ...
    collisionFiles.base, ...
    TMeshBase);


addBody( ...
    visualRobot, ...
    baseBody, ...
    visualRobot.BaseName);


%% ========================================================================
% 8. JOINT 1 — SHOULDER PAN
% =========================================================================
%
% Official UR5 joint origin:
%
%       xyz = [0, 0, 0.089159]
%       rpy = [0, 0, 0]
%
%
% Joint axis:
%
%       z
%
%
% The shoulder mesh itself has:
%
%       yaw = 180 deg

shoulderBody = rigidBody("Shoulder");

joint1 = rigidBodyJoint( ...
    getJointName(robot,1,"J1"), ...
    "revolute");


joint1.JointAxis = [0 0 1];

joint1.PositionLimits = [qMin(1) qMax(1)];


TJoint1 = xyzRPYTransform( ...
    [0 0 0.089159], ...
    [0 0 0]);


setFixedTransform( ...
    joint1, ...
    TJoint1);


shoulderBody.Joint = joint1;


TMeshShoulder = xyzRPYTransform( ...
    [0 0 0], ...
    [0 0 pi]);


addVisual( ...
    shoulderBody, ...
    "mesh", ...
    visualFiles.shoulder, ...
    TMeshShoulder);


addCollision( ...
    shoulderBody, ...
    "mesh", ...
    collisionFiles.shoulder, ...
    TMeshShoulder);


addBody( ...
    visualRobot, ...
    shoulderBody, ...
    visualRobot.BaseName);


%% ========================================================================
% 9. JOINT 2 — SHOULDER LIFT / UPPER ARM
% =========================================================================
%
% Official joint origin:
%
%       xyz = [0 0 0]
%
%       roll = 90 deg
%       pitch = 0
%       yaw = 0
%
%
% Official upper-arm mesh offset:
%
%       xyz = [0 0 0.13585]
%
%       roll  =  90 deg
%       pitch =   0 deg
%       yaw   = -90 deg

upperArmBody = rigidBody("UpperArm");

joint2 = rigidBodyJoint( ...
    getJointName(robot,2,"J2"), ...
    "revolute");


joint2.JointAxis = [0 0 1];

joint2.PositionLimits = [qMin(2) qMax(2)];


TJoint2 = xyzRPYTransform( ...
    [0 0 0], ...
    [pi/2 0 0]);


setFixedTransform( ...
    joint2, ...
    TJoint2);


upperArmBody.Joint = joint2;


TMeshUpperArm = xyzRPYTransform( ...
    [0 0 0.13585], ...
    [pi/2 0 -pi/2]);


addVisual( ...
    upperArmBody, ...
    "mesh", ...
    visualFiles.upperArm, ...
    TMeshUpperArm);


addCollision( ...
    upperArmBody, ...
    "mesh", ...
    collisionFiles.upperArm, ...
    TMeshUpperArm);


addBody( ...
    visualRobot, ...
    upperArmBody, ...
    "Shoulder");


%% ========================================================================
% 10. JOINT 3 — ELBOW / FOREARM
% =========================================================================
%
% Official joint origin:
%
%       xyz = [-0.425 0 0]
%       rpy = [0 0 0]
%
%
% Official forearm mesh offset:
%
%       xyz = [0 0 0.0165]
%
%       roll  =  90 deg
%       pitch =   0 deg
%       yaw   = -90 deg

forearmBody = rigidBody("Forearm");

joint3 = rigidBodyJoint( ...
    getJointName(robot,3,"J3"), ...
    "revolute");


joint3.JointAxis = [0 0 1];

joint3.PositionLimits = [qMin(3) qMax(3)];


TJoint3 = xyzRPYTransform( ...
    [-0.425 0 0], ...
    [0 0 0]);


setFixedTransform( ...
    joint3, ...
    TJoint3);


forearmBody.Joint = joint3;


TMeshForearm = xyzRPYTransform( ...
    [0 0 0.0165], ...
    [pi/2 0 -pi/2]);


addVisual( ...
    forearmBody, ...
    "mesh", ...
    visualFiles.forearm, ...
    TMeshForearm);


addCollision( ...
    forearmBody, ...
    "mesh", ...
    collisionFiles.forearm, ...
    TMeshForearm);


addBody( ...
    visualRobot, ...
    forearmBody, ...
    "UpperArm");


%% ========================================================================
% 11. JOINT 4 — WRIST 1
% =========================================================================
%
% Official joint origin:
%
%       xyz =
%
%       [-0.39225, 0, 0.10915]
%
%
% Mesh offset:
%
%       xyz = [0 0 -0.093]
%
%       roll = 90 deg

wrist1Body = rigidBody("Wrist1");

joint4 = rigidBodyJoint( ...
    getJointName(robot,4,"J4"), ...
    "revolute");


joint4.JointAxis = [0 0 1];

joint4.PositionLimits = [qMin(4) qMax(4)];


TJoint4 = xyzRPYTransform( ...
    [-0.39225 0 0.10915], ...
    [0 0 0]);


setFixedTransform( ...
    joint4, ...
    TJoint4);


wrist1Body.Joint = joint4;


TMeshWrist1 = xyzRPYTransform( ...
    [0 0 -0.093], ...
    [pi/2 0 0]);


addVisual( ...
    wrist1Body, ...
    "mesh", ...
    visualFiles.wrist1, ...
    TMeshWrist1);


addCollision( ...
    wrist1Body, ...
    "mesh", ...
    collisionFiles.wrist1, ...
    TMeshWrist1);


addBody( ...
    visualRobot, ...
    wrist1Body, ...
    "Forearm");


%% ========================================================================
% 12. JOINT 5 — WRIST 2
% =========================================================================
%
% Official joint origin:
%
%       xyz = [0, -0.09465, 0]
%
%       roll = 90 deg
%
%
% Mesh offset:
%
%       xyz = [0 0 -0.095]
%
%       rpy = [0 0 0]

wrist2Body = rigidBody("Wrist2");

joint5 = rigidBodyJoint( ...
    getJointName(robot,5,"J5"), ...
    "revolute");


joint5.JointAxis = [0 0 1];

joint5.PositionLimits = [qMin(5) qMax(5)];


TJoint5 = xyzRPYTransform( ...
    [0 -0.09465 0], ...
    [pi/2 0 0]);


setFixedTransform( ...
    joint5, ...
    TJoint5);


wrist2Body.Joint = joint5;


TMeshWrist2 = xyzRPYTransform( ...
    [0 0 -0.095], ...
    [0 0 0]);


addVisual( ...
    wrist2Body, ...
    "mesh", ...
    visualFiles.wrist2, ...
    TMeshWrist2);


addCollision( ...
    wrist2Body, ...
    "mesh", ...
    collisionFiles.wrist2, ...
    TMeshWrist2);


addBody( ...
    visualRobot, ...
    wrist2Body, ...
    "Wrist1");


%% ========================================================================
% 13. JOINT 6 — WRIST 3
% =========================================================================
%
% Official joint origin:
%
%       xyz = [0, 0.0823, 0]
%
%       roll  =  90 deg
%       pitch = 180 deg
%       yaw   = 180 deg
%
%
% Mesh offset:
%
%       xyz = [0 0 -0.0818]
%
%       roll = 90 deg

wrist3Body = rigidBody("Wrist3");

joint6 = rigidBodyJoint( ...
    getJointName(robot,6,"J6"), ...
    "revolute");


joint6.JointAxis = [0 0 1];

joint6.PositionLimits = [qMin(6) qMax(6)];


TJoint6 = xyzRPYTransform( ...
    [0 0.0823 0], ...
    [pi/2 pi pi]);


setFixedTransform( ...
    joint6, ...
    TJoint6);


wrist3Body.Joint = joint6;


TMeshWrist3 = xyzRPYTransform( ...
    [0 0 -0.0818], ...
    [pi/2 0 0]);


addVisual( ...
    wrist3Body, ...
    "mesh", ...
    visualFiles.wrist3, ...
    TMeshWrist3);


addCollision( ...
    wrist3Body, ...
    "mesh", ...
    collisionFiles.wrist3, ...
    TMeshWrist3);


addBody( ...
    visualRobot, ...
    wrist3Body, ...
    "Wrist2");


%% ========================================================================
% 14. ADD UR FLANGE FRAME
% =========================================================================
%
% Universal Robots defines a fixed flange transform after wrist 3:
%
%       roll  = 0
%       pitch = -90 deg
%       yaw   = -90 deg
%
%
% This body has no geometry.
%
% It simply provides the proper tool attachment frame.

flangeName = char(robot.frames.flange);

flangeBody = rigidBody(flangeName);

flangeJoint = rigidBodyJoint( ...
    "FlangeFixed", ...
    "fixed");


TFlange = xyzRPYTransform( ...
    [0 0 0], ...
    [0 -pi/2 -pi/2]);


setFixedTransform( ...
    flangeJoint, ...
    TFlange);


flangeBody.Joint = flangeJoint;


addBody( ...
    visualRobot, ...
    flangeBody, ...
    "Wrist3");


%% ========================================================================
% 15. ADD PROJECT TCP FRAME
% =========================================================================
%
% The robot's physical UR5 geometry ends at the flange.
%
% Our project may later attach:
%
%       welding torch
%       custom tool mount
%       tungsten TCP
%
% Therefore we preserve the same:
%
%       robot.tool.T_F_TCP
%
% used by the Control model.

tcpName = char(robot.frames.tcp);

tcpBody = rigidBody(tcpName);

tcpJoint = rigidBodyJoint( ...
    "TCP_Fixed", ...
    "fixed");


setFixedTransform( ...
    tcpJoint, ...
    robot.tool.T_F_TCP);


tcpBody.Joint = tcpJoint;


addBody( ...
    visualRobot, ...
    tcpBody, ...
    flangeName);


%% ========================================================================
% 16. FINAL INFORMATION
% =========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' UR5 VISUAL GEOMETRY MODEL BUILT\n');
fprintf('============================================================\n');

fprintf('Visual mesh folder:\n%s\n\n',visualFolder);

fprintf('Collision mesh folder:\n%s\n\n',collisionFolder);

fprintf('Bodies: %d\n',visualRobot.NumBodies);
fprintf('Moving joints: 6\n');

fprintf('Official UR5 visual meshes attached: PASS\n');
fprintf('Official UR5 collision meshes attached: PASS\n');

fprintf('============================================================\n');

end


%% ========================================================================
% LOCAL HELPER: URDF XYZ + RPY -> HOMOGENEOUS TRANSFORM
% =========================================================================
%
% URDF roll-pitch-yaw means:
%
%       R = Rz(yaw) * Ry(pitch) * Rx(roll)
%
% We implement this explicitly instead of relying on Euler-angle defaults.

function T = xyzRPYTransform(xyz,rpy)

x = xyz(1);
y = xyz(2);
z = xyz(3);

roll  = rpy(1);
pitch = rpy(2);
yaw   = rpy(3);


Rx = [
    1           0            0
    0   cos(roll)   -sin(roll)
    0   sin(roll)    cos(roll)
];


Ry = [
     cos(pitch)   0   sin(pitch)
              0   1            0
    -sin(pitch)   0   cos(pitch)
];


Rz = [
    cos(yaw)   -sin(yaw)   0
    sin(yaw)    cos(yaw)   0
           0           0   1
];


R = Rz * Ry * Rx;


T = [
    R       [x;y;z]
    0 0 0        1
];

end


%% ========================================================================
% LOCAL HELPER: GET PROJECT JOINT NAME
% =========================================================================
%
% Prefer the names already defined in config.UR5().
%
% If they are unavailable, use the supplied fallback name.

function name = getJointName(robot,index,fallback)

if isfield(robot,"joints") && ...
   isfield(robot.joints,"names") && ...
   numel(robot.joints.names) >= index

    name = char(robot.joints.names(index));

else

    name = fallback;

end

end