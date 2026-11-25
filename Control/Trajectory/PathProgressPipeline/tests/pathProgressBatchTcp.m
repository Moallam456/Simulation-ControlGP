function [p,R,speed,leftSpeed] = pathProgressBatchTcp(frames,q,v,vLeft)
%PATHPROGRESSBATCHTCP Batch the URDF revolute chain and its linear Jacobian.
% Test-only MATLAB implementation. frames stores zero-angle relative
% transforms and local joint axes derived from the Robotics System Toolbox.
n = size(q,2);
dof = size(q,1);
R = repmat(eye(3),1,1,n);
p = zeros(3,n);
origins = zeros(3,n,dof);
axes = zeros(3,n,dof);
for j = 1:dof
    fixed = frames.fixed(:,:,j);
    p = p+reshape(pagemtimes(R,fixed(1:3,4)),3,n);
    R = pagemtimes(R,fixed(1:3,1:3));
    origins(:,:,j) = p;
    axes(:,:,j) = reshape(pagemtimes(R,frames.axis(:,j)),3,n);
    skew = frames.skew(:,:,j);
    angle = reshape(q(j,:),1,1,n);
    R = pagemtimes(R,eye(3)+skew.*sin(angle)+(skew*skew).*(1-cos(angle)));
end
p = p+reshape(pagemtimes(R,frames.tool(1:3,4)),3,n);
R = pagemtimes(R,frames.tool(1:3,1:3));
linear = zeros(3,n);
left = linear;
for j = 1:dof
    J = cross(axes(:,:,j),p-origins(:,:,j),1);
    linear = linear+J.*v(j,:);
    left = left+J.*vLeft(j,:);
end
speed = vecnorm(linear,2,1);
leftSpeed = vecnorm(left,2,1);
end
