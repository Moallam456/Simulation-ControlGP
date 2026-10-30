function export_robot_geometry()
% Mechanical export: regenerate exact URDF transforms, not analytical guesses.
cfg=testbench_config(); robot=testbench_robot(cfg); folder=fileparts(mfilename('fullpath'));
f=fopen(fullfile(folder,'robot_ver2_geometry.h'),'w'); assert(f>=0); clean=onCleanup(@() fclose(f));
fprintf(f,'/* Generated from packaged URDF by export_robot_geometry. SI, row-major. */\n');
fprintf(f,'#ifndef ROBOT_VER2_GEOMETRY_H\n#define ROBOT_VER2_GEOMETRY_H\n');
fprintf(f,'static const double RB2_ORIGINS[6][16] = {\n');
for i=1:6
    b=robot.Bodies{i}; T=getTransform(robot,zeros(1,6),b.Name,b.Parent.Name);
    fprintf(f,'{');fprintf(f,'%.17g,',reshape(T.',1,16));fprintf(f,'},\n');
end
fprintf(f,'};\nstatic const double RB2_AXES[6][3] = {\n');
for i=1:6, fprintf(f,'{%.17g,%.17g,%.17g},\n',robot.Bodies{i}.Joint.JointAxis); end
fprintf(f,'};\nstatic const double RB2_TOOL[16] = {');
T=getTransform(robot,zeros(1,6),'tcp','End_Effector');fprintf(f,'%.17g,',reshape(T.',1,16));
fprintf(f,'};\n#endif\n');
f2=fopen(fullfile(folder,'robot_ver2_profile.h'),'w');assert(f2>=0);c2=onCleanup(@() fclose(f2));
fprintf(f2,'/* Generated SYNTHETIC profile. Not physical commissioning. */\n#ifndef RB2_PROFILE_H\n#define RB2_PROFILE_H\n');
for name={'home_rad','provisionalLimits_rad'}
    v=cfg.(name{1});fprintf(f2,'static const double RB2_%s[%d] = {',upper(name{1}),numel(v));
    fprintf(f2,'%.17g,',reshape(v.',1,[]));fprintf(f2,'};\n');
end
fprintf(f2,'#define RB2_DEMO_REDUCTION 50.0\n#define RB2_DEMO_SPEED 2.0\n#endif\n');
end
