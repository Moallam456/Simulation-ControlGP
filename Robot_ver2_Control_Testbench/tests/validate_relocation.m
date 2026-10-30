function report=validate_relocation()
% Relocate only runtime deliverables; discard all developer/Final_Architecture paths.
source=fileparts(fileparts(mfilename('fullpath')));
destination=[tempname '_robot_testbench_relocated'];mkdir(destination);
for folder={'src','control','mechanics'}
    copyfile(fullfile(source,folder{1}),fullfile(destination,folder{1}));
end
for file={'testbench_config.m','open_robot_testbench.m','robot_ver2_testbench.slx'}
    copyfile(fullfile(source,file{1}),fullfile(destination,file{1}));
end
oldPath=path;clean=onCleanup(@() path(oldPath));
if bdIsLoaded('robot_ver2_testbench'),close_system('robot_ver2_testbench',0);end
clear testbench_config testbench_robot testbench_runtime rb2_native rb2_controller_native rb2_admittance_native
restoredefaultpath;addpath(destination,fullfile(destination,'src'),fullfile(destination,'control'));
assert(startsWith(which('testbench_robot'),destination));assert(startsWith(which('rb2_controller_native'),destination));
cfg=testbench_config();robot=testbench_robot(cfg);cache=testbench_collision_cache(robot);
assert(isfinite(cache.leverBound_m));report.leverBound_m=cache.leverBound_m;
before=cache.meshHashes;
file=fullfile(destination,'mechanics','URDF_Assembly_Robot_ver2','meshes','robot_ver2_Link_1.STL');
f=fopen(file,'r+');assert(f>=0);fseek(f,96,'bof');v=fread(f,1,'single');
fseek(f,96,'bof');fwrite(f,v+1e-6,'single');fclose(f);
changed=testbench_collision_cache(robot);
assert(~isequal(before,changed.meshHashes) && isempty(changed.q));
report.meshChangeInvalidation=true;
setappdata(0,'RobotTestbenchConfig',cfg);
load_system(fullfile(destination,'robot_ver2_testbench.slx'));
in=Simulink.SimulationInput('robot_ver2_testbench');
in=in.setModelParameter('StopTime','.05','SimMechanicsOpenEditorOnUpdate','off');
out=sim(in);assert(all(isfinite(out.tb_joint_position.Data),'all'));
close_system('robot_ver2_testbench',0);
report.runtimeFolder=destination;report.pass=true;disp(report);
end
