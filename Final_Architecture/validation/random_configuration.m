function q = random_configuration(robot)

n = size(robot.jointLimits,1);

q = zeros(n,1);

for i = 1:n

    qmin = robot.jointLimits(i,1);
    qmax = robot.jointLimits(i,2);

    q(i) = qmin + rand*(qmax-qmin);

end

end