function [velocity, acceleration, jerk] = pathProgressChainRule(qs, qss, qsss, sd, sdd, sddd)
%PATHPROGRESSCHAINRULE Apply timing derivatives to the same spatial cubic.
% Spatial arrays are joints-by-sample; timing arrays are 1-by-sample.
% A NaN sddd marks an ideal trapezoid's acceleration switch, not zero jerk.

[dof,n] = size(qs);
velocity = zeros(dof,n);
acceleration = velocity;
jerk = velocity;
for i = 1:n
    speed = sd(i);
    accel = sdd(i);
    for joint = 1:dof
        velocity(joint,i) = qs(joint,i)*speed;
        acceleration(joint,i) = qss(joint,i)*speed*speed + qs(joint,i)*accel;
        jerk(joint,i) = qsss(joint,i)*speed*speed*speed + ...
            3*qss(joint,i)*speed*accel + qs(joint,i)*sddd(i);
    end
end
end
