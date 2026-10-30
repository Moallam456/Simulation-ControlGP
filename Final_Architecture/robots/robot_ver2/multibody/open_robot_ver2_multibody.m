function model=open_robot_ver2_multibody()
% OPEN_ROBOT_VER2_MULTIBODY Open the saved control bench without rebuilding it.
folder=fileparts(mfilename('fullpath'));
root=fileparts(fileparts(fileparts(folder))); addpath(genpath(root));
model='robot_ver2_multibody'; file=fullfile(folder,[model '.slx']);
if ~isfile(file), build_robot_ver2_multibody(); end
open_system(file);
set_param(model,'ZoomFactor','FitSystem');
workspace=get_param(model,'ModelWorkspace'); info=workspace.evalin('multibodyInfo');
source=fullfile(folder,'..','CAD','URDF_Assembly_Robot_ver2','urdf','URDF_Assembly_Robot_ver2.urdf');
if isstruct(info) && isfield(info,'urdfSHA256')
    fid=fopen(source,'rb');
    if fid<0
        warning('multibody:MissingSource','Saved model opened, but the URDF is missing. Keep the robot package for source verification and rebuilding.');
        return;
    end
    cleanup=onCleanup(@() fclose(fid)); data=fread(fid,Inf,'*uint8');
    digest=java.security.MessageDigest.getInstance('SHA-256'); digest.update(data);
    signature=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
    if ~strcmp(signature,info.urdfSHA256)
        warning('multibody:SourceChanged', ...
            'The URDF changed after this model was built. Rebuild explicitly; saved control edits were not overwritten.');
    end
end
fprintf('Opened robot_ver2_multibody. Press Ctrl+D, then Run.\n');
fprintf('RobotPlant: joint torque input; q, qd, qdd and measured TCP outputs.\n');
fprintf('DemoController is an ideal PD demonstration, not a motor controller or final tuning.\n');
end
