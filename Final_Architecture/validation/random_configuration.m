function q = random_configuration(robot)

% RANDOM_CONFIGURATION Draw one valid random row-vector configuration.

n = robot.structure.dof;

q = zeros(1,n);

for i = 1:n

    qmin = robot.params.joints.positionLimits(i,1);
    qmax = robot.params.joints.positionLimits(i,2);

    q(i) = qmin + rand*(qmax-qmin);

end

end
