function [T_B_TCP, frames] = forward_kinematics(robot,q)

% FORWARD_KINEMATICS Generic FK for the common robot interface.
%
% INPUTS:
%   robot : struct returned by loadRobot()
%   q     : 1-by-n row vector [rad]
%
% OUTPUTS:
%   T_B_TCP : base-to-end-effector transform
%   frames  : base, body, and end-effector transforms

validateRobot(robot);
q = validateConfiguration(robot,q);

if isfield(robot.dh,'fixedTransforms')
    [T_B_TCP, frames] = fixedTransformFK(robot,q);
else
    [T_B_TCP, frames] = rigidBodyTreeFK(robot,q);
end

end

function validateRobot(robot)

if ~isstruct(robot) || ~isfield(robot,'structure') || ...
   ~isfield(robot,'dh') || ~isfield(robot,'params')
    error('forward_kinematics:InvalidRobot', ...
        'Expected a robot struct returned by loadRobot().');
end

end

function q = validateConfiguration(robot,q)

q = q(:).';

if numel(q) ~= robot.structure.dof
    error('forward_kinematics:InvalidJointVector', ...
        'Expected %d joint values for robot %s; received %d.', ...
        robot.structure.dof, robot.id, numel(q));
end

if any(~isfinite(q))
    error('forward_kinematics:InvalidJointVector', ...
        'Joint configuration must contain finite values.');
end

end

function [T_B_TCP, frames] = fixedTransformFK(robot,q)

T = eye(4);
frames = cell(1,robot.structure.dof + 2);
frames{1} = T;

for i = 1:robot.structure.dof
    T = T * robot.dh.fixedTransforms{i};
    T = T * jointAxisTransform(robot.structure.jointAxes(i,:),q(i));
    frames{i+1} = T;
end

T_B_TCP = T * robot.params.tool.TFlangeTCP;
frames{end} = T_B_TCP;

end

function T = jointAxisTransform(axis,q)

axis = axis(:);
axis = axis/norm(axis);

x = axis(1);
y = axis(2);
z = axis(3);

c = cos(q);
s = sin(q);
C = 1 - c;

R = [
    x*x*C + c,   x*y*C - z*s, x*z*C + y*s;
    y*x*C + z*s, y*y*C + c,   y*z*C - x*s;
    z*x*C - y*s, z*y*C + x*s, z*z*C + c];

T = eye(4);
T(1:3,1:3) = R;

end

function [T_B_TCP, frames] = rigidBodyTreeFK(robot,q)

frames = cell(1,robot.structure.dof + 2);
frames{1} = eye(4);

for i = 1:robot.structure.dof
    frames{i+1} = getTransform( ...
        robot.model, ...
        q, ...
        char(robot.structure.bodyNames(i)));
end

T_B_TCP = getTransform( ...
    robot.model, ...
    q, ...
    char(robot.structure.frames.endEffector));

frames{end} = T_B_TCP;

end
