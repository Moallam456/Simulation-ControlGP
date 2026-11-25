function [p,ps,pss,psss] = pathProgressCircleGeometry(circle,progress)
%PATHPROGRESSCIRCLEGEOMETRY Evaluate a signed sweep using numeric arrays.
% e1 points toward the start, so the starting angle in this basis is zero.
% Outputs are 3-by-N in metres, with derivatives per unit progress.
progress = progress(:).';
sweep = circle.thetaTotal;
p = zeros(3,numel(progress));
ps = p;
pss = p;
psss = p;
for k = 1:numel(progress)
    theta = sweep*progress(k);
    radial = circle.radius*(circle.e1*cos(theta)+circle.e2*sin(theta));
    angular = circle.radius*(-circle.e1*sin(theta)+circle.e2*cos(theta));
    p(:,k) = circle.center+radial;
    ps(:,k) = sweep*angular;
    pss(:,k) = -sweep^2*radial;
    psss(:,k) = -sweep^3*angular;
end
end
