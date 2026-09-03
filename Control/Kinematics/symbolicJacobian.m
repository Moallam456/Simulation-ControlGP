%% Symbolic Jacobian derivation

% Symbolic joint angles
syms q1 q2 q3 q4 q5 q6 real

q = [
    q1
    q2
    q3
    q4
    q5
    q6
];

% Load UR5 reference configuration
robot = config.UR5();

% Read DH parameters
a = robot.kinematics.a;
d = robot.kinematics.d;
thetaOffset = robot.kinematics.thetaOffset;

% Convert dimensions to clean symbolic values
a_sym = sym(a, 'r');
d_sym = sym(d, 'r');
thetaOffset_sym = sym(thetaOffset, 'r');

% Define twist angles exactly
alpha_sym = [
     sym(pi)/2
     0
     0
     sym(pi)/2
    -sym(pi)/2
     0
];

% Start at the Base frame
T = sym(eye(4));

% Storage for joint origins and axes
jointOrigins = sym(zeros(3, robot.dof));
jointAxes = sym(zeros(3, robot.dof));

% Move through each joint and store its origin and axis
for i = 1:robot.dof

    % Store frame i-1 origin in Base coordinates
    jointOrigins(:,i) = T(1:3,4);

    % Store frame i-1 Z-axis in Base coordinates
    jointAxes(:,i) = T(1:3,3);

    % Current symbolic joint angle
    theta = q(i) + thetaOffset_sym(i);

    % Symbolic DH transformation for this joint
    A = dhTransform( ...
        a_sym(i), ...
        d_sym(i), ...
        alpha_sym(i), ...
        theta);

    % Move to the next frame
    T = T * A;
end


% Apply the flange-to-TCP transformation
T_B_TCP = simplify(T * sym(robot.tool.T_F_TCP, 'r'));

% Extract symbolic TCP position in Base coordinates
p_TCP = simplify(T_B_TCP(1:3,4));

% Preallocate symbolic Jacobian matrix
J_sym = sym(zeros(6, robot.dof));

% Build one symbolic Jacobian column for each revolute joint
for i = 1:robot.dof

    % Joint origin and axis in Base coordinates
    p_joint = jointOrigins(:,i);
    z_joint = jointAxes(:,i);

    % Symbolic linear velocity contribution
    Jv = cross(z_joint, p_TCP - p_joint);

    % Symbolic angular velocity contribution
    Jw = z_joint;

    % Store the full Jacobian column
    J_sym(:,i) = [
        Jv
        Jw
    ];

end

%% Simplify symbolic Jacobian

% Create a cleaner symbolic form
J_clean = simplify(J_sym, 'Steps', 100);

%% Validate symbolic Jacobian against numerical Jacobian

q_test = [
     0.2
    -0.3
     0.5
     0.1
    -0.4
     0.2
];

% Substitute the test joint angles into the symbolic Jacobian
J_symbolic_test = double(subs( ...
    J_clean, ...
    [q1 q2 q3 q4 q5 q6], ...
    q_test.' ));

% Calculate Jacobian using our numerical implementation
J_numeric_test = controlJacobian(robot, q_test);

% Compare them
symbolic_vs_numeric_difference = ...
    J_symbolic_test - J_numeric_test;

% Maximum absolute error
max_symbolic_error = max( ...
    abs(symbolic_vs_numeric_difference), ...
    [], 'all');
