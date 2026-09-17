function [model,validation] = prepareDynamicsModel(robot,gravity)
% PREPAREDYNAMICSMODEL Validate data and apply scenario gravity to a copy.
validation = validateDynamicModel(robot);
if ~validation.pass
    error('dynamics:InvalidModel','%s',strjoin(validation.errors,newline));
end
if ~isnumeric(gravity) || numel(gravity) ~= 3 || any(~isfinite(gravity))
    error('dynamics:InvalidGravity','scenario.gravity must be a finite 1-by-3 vector in m/s^2.');
end
model = copy(robot.model);
model.Gravity = reshape(gravity,1,3);
end
