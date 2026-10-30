function preview_testbench()
% Headless visual QA; actual main dashboard and generated block diagram.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root,fullfile(root,'src'),fullfile(root,'control'));
old=get(groot,'DefaultFigureVisible');cleanup=onCleanup(@() set(groot,'DefaultFigureVisible',old));
set(groot,'DefaultFigureVisible','off');
session=open_robot_testbench();set(session.figure,'Visible','on');drawnow;
frame=getframe(session.figure);imwrite(frame.cdata,fullfile(tempdir,'robot_testbench_dashboard.png'));
close(session.figure);
load_system(fullfile(root,'robot_ver2_testbench.slx'));
set_param('robot_ver2_testbench','PaperPositionMode','auto');
print('-srobot_ver2_testbench','-dpng','-r110',fullfile(tempdir,'robot_testbench_diagram.png'));
close_system('robot_ver2_testbench',0);
end
