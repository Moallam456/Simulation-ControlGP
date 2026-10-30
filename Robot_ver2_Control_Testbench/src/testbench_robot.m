function robot = testbench_robot(cfg)
% TESTBENCH_ROBOT Loads only this package's URDF; never projectConfig/path lookup.
if nargin==0, cfg=testbench_config(); end
root=fileparts(fileparts(mfilename('fullpath')));
file=fullfile(root,'mechanics','URDF_Assembly_Robot_ver2','urdf', ...
    'URDF_Assembly_Robot_ver2.urdf');
robot=importrobot(file,'DataFormat','row'); robot.Gravity=cfg.gravity_m_s2;
assert(isequal(robot.BodyNames,{'Link_1','Link_2','Link_3','Link_4','Link_5','End_Effector','tcp'}));
for i=1:6
    robot.Bodies{i}.Joint.PositionLimits=cfg.provisionalLimits_rad(i,:);
    robot.Bodies{i}.Joint.HomePosition=cfg.home_rad(i);
end
end
