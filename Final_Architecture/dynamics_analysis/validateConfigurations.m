function validateConfigurations(q,robot)
% VALIDATECONFIGURATIONS Shared SI joint-position contract.
dof = robot.structure.dof;
if ~isnumeric(q) || size(q,2) ~= dof || isempty(q) || any(~isfinite(q(:)))
    error('dynamics:InvalidConfiguration','q must be a finite N-by-DOF matrix in radians.');
end
limits = robot.params.joints.positionLimits;
if any(any(q < limits(:,1).'-1e-10 | q > limits(:,2).'+1e-10))
    error('dynamics:JointLimit','Configuration exceeds robot joint limits.');
end
end
