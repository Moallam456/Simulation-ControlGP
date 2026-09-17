function torque = estimateGravityMomentFromCom(robot,q,gravity)
% ESTIMATEGRAVITYMOMENTFROMCOM Sum static moments of downstream body weights.
% Independent of gravityTorque; useful to audit mass/COM placement at a pose.
validateConfigurations(q,robot);
if size(q,1) ~= 1 || numel(gravity) ~= 3 || any(~isfinite(gravity))
    error('dynamics:InvalidGravityMomentInput','Supply one pose and a finite gravity vector.');
end
model = robot.model;
dof = robot.structure.dof;
torque = zeros(1,dof);
for j = 1:dof
    jointBody = model.Bodies{j};
    TJoint = getTransform(model,q,jointBody.Name);
    origin = TJoint(1:3,4);
    axisWorld = TJoint(1:3,1:3)*jointBody.Joint.JointAxis(:);
    for k = j:numel(model.Bodies)
        body = model.Bodies{k};
        T = getTransform(model,q,body.Name);
        comWorld = T(1:3,1:3)*body.CenterOfMass(:)+T(1:3,4);
        force = body.Mass*gravity(:);
        torque(j) = torque(j)-dot(axisWorld,cross(comWorld-origin,force));
    end
end
end
