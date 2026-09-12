function T_B_TCP = forward_kinematics(robot,q)

dh = dh_parameters(robot);

T = eye(4);

for i = 1:robot.structure.numJoints

    theta = q(i) + dh.thetaOffset(i);

    A = dh_transform( ...
        dh.a(i), ...
        dh.d(i), ...
        dh.alpha(i), ...
        theta);

    T = T*A;

end

T_B_TCP = T * robot.tool.T_F_TCP;

end