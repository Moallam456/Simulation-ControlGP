function T_B_TCP = controlFK(robot,q)

a = robot.kinematics.a;
d = robot.kinematics.d;
alpha = robot.kinematics.alpha;
thetaOffset = robot.kinematics.thetaOffset;

% This creates an Identity matrix 4x4
% This represets no rotation and no translation as we are starting from the
% base frame
T = eye(4);

%Loops through each joint
for i = 1:robot.dof
    % Calculate  the actual joint angle
    theta = q(i) + thetaOffset(i);
    A = dhTransform(a(i), d(i), alpha(i), theta);
    %To accumulate the transformations
    T = T * A;

end

%Now we got the last frame relative to base frame
%Next we multiply by the TCP to F transformation to get teh final T

T_B_TCP = T * robot.tool.T_F_TCP;

end