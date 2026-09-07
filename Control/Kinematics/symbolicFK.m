% Symbolic joint angles for the 6 revolute joints
syms q1 q2 q3 q4 q5 q6 real

% Store symbolic joint angles in the same 6x1 format used by the robot model
q = [
    q1
    q2
    q3
    q4
    q5
    q6
];

% Load the UR5 reference robot configuration
robot = config.UR5();

% Read the fixed DH parameters of the UR5
a = robot.kinematics.a;
d = robot.kinematics.d;
alpha = robot.kinematics.alpha;
thetaOffset = robot.kinematics.thetaOffset;

% Start at the Base frame with an identity transformation
T = sym(eye(4));
%sym(eye) instead of eye, to create symbolic identity matrix

% Multiply the DH transformation of each joint
for i = 1:robot.dof

    theta = q(i) + thetaOffset(i);

    A = dhTransform(a(i), d(i), alpha(i), theta);

    T = T * A;

end
% Raw symbolic Base-to-Flange FK
T_06_raw = T;

% Raw symbolic Base-to-TCP FK
T_raw = T_06_raw * robot.tool.T_F_TCP;

%% Clean symbolic version for readable equations

% Convert the UR5 dimensions to clean exact symbolic values
a_clean = sym(a, 'r');
d_clean = sym(d, 'r');
thetaOffset_clean = sym(thetaOffset, 'r');

% Define the UR5 twist angles exactly
alpha_clean = [
     sym(pi)/2
     0
     0
     sym(pi)/2
    -sym(pi)/2
     0
];

% Start a separate clean symbolic FK chain
T_clean = sym(eye(4));

for i = 1:robot.dof

    theta_clean = q(i) + thetaOffset_clean(i);

    A_clean = dhTransform( ...
        a_clean(i), ...
        d_clean(i), ...
        alpha_clean(i), ...
        theta_clean);

    T_clean = T_clean * A_clean;

end

% Apply flange-to-TCP transform
T_clean = simplify(T_clean * sym(robot.tool.T_F_TCP, 'r'));

% Extract clean TCP position and orientation
TCP_position_clean = simplify(T_clean(1:3,4));
TCP_orientation_clean = simplify(T_clean(1:3,1:3));

%% Validate clean symbolic FK against raw symbolic FK

q_test = [
     0.2
    -0.3
     0.5
     0.1
    -0.4
     0.2
];

T_raw_test = double(subs( ...
    T_raw, ...
    [q1 q2 q3 q4 q5 q6], ...
    q_test.' ));

T_clean_test = double(subs( ...
    T_clean, ...
    [q1 q2 q3 q4 q5 q6], ...
    q_test.' ));

clean_vs_raw_difference = T_clean_test - T_raw_test;

max_clean_error = max(abs(clean_vs_raw_difference), [], 'all');