function structure = robotStructure()

% ROBOTSTRUCTURE Topology and frame definitions for initial_trial.
%
% q is ordered as:
%   q = [J1 J2 J3 J4 J5 J6]

structure.name = "initial_trial";
structure.manufacturer = "project_team";
structure.model = "initial_trial";
structure.dof = 6;

structure.bodyNames = [
    "link_1"
    "link_2"
    "link_3"
    "link_4"
    "link_5"
    "link_6"];

structure.parentBodies = [
    "base"
    "link_1"
    "link_2"
    "link_3"
    "link_4"
    "link_5"];

structure.jointNames = [
    "joint_1"
    "joint_2"
    "joint_3"
    "joint_4"
    "joint_5"
    "joint_6"];

structure.jointTypes = repmat("revolute",1,structure.dof);
structure.jointOrder = 1:structure.dof;

% Axes are expressed in each joint frame. They are chosen so the zero
% configuration matches the provided sketch:
% J1 about vertical base Z, J2/J3 pitch, J4 roll along horizontal wrist,
% J5 pitch, J6 roll along the tool/wrist direction.
structure.jointAxes = [
    0  0  1;
    0 -1  0;
    0 -1  0;
    0  0  1;
    0  1  0;
    0  0  1];

structure.frames.base = "base";
structure.frames.flange = "link_6";
structure.frames.endEffector = "tcp";

structure.hasSphericalWrist = false;
structure.description = "6-DOF initial trial welding robot geometry.";

end
