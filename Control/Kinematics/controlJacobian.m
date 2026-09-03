function J = controlJacobian(robot,q)

% Geometric Jacobian expressed in the Base frame
% [v_B; omega_B] = J * q_dot

% Read the robot DH parameters
a = robot.kinematics.a;
d = robot.kinematics.d;
alpha = robot.kinematics.alpha;
thetaOffset = robot.kinematics.thetaOffset;

% Start from the Base frame
T = eye(4);

% Store joint origins (q) and joint Z-axes expressed in the Base frame
jointOrigins = zeros(3, robot.dof);
jointAxes = zeros(3, robot.dof);

for i = 1:robot.dof

    % Store the origin of frame i-1 in Base coordinates
    jointOrigins(:,i) = T(1:3,4);

    % Store the Z-axis of frame i-1 in Base coordinates
    jointAxes(:,i) = T(1:3,3);

    % Calculate the current joint angle
    theta = q(i) + thetaOffset(i);

    % Calculate the DH transformation for this joint
    A = dhTransform(a(i), d(i), alpha(i), theta);

    % Move to the next frame
    T = T * A;

end

% Apply the flange-to-TCP transformation
T_B_TCP = T * robot.tool.T_F_TCP;

% Extract TCP position expressed in the Base frame
p_TCP = T_B_TCP(1:3,4);

% Preallocate the 6 x DOF Jacobian matrix
J = zeros(6, robot.dof);

% Build one Jacobian column for each revolute joint
for i = 1:robot.dof

    % Joint origin and axis in Base coordinates
    p_joint = jointOrigins(:,i);
    z_joint = jointAxes(:,i);

    % Linear velocity contribution of joint i
    Jv = cross(z_joint, p_TCP - p_joint);

    % Angular velocity contribution of joint i
    Jw = z_joint;

    % Store both parts in column i
    J(:,i) = [
        Jv
        Jw
    ];

end

end