function collision = collisionGeometry(~,structure,~)
% COLLISIONGEOMETRY Metadata for the STL geometry imported with robot_ver2.
robotFolder = fileparts(fileparts(mfilename('fullpath')));
meshFolder = fullfile(robotFolder,'CAD','URDF_Assembly_Robot_ver2','meshes');
names = ["Base_Link" structure.bodyNames];
meshFiles = strings(size(names));
for i = 1:numel(names)
    meshFiles(i) = string(fullfile(meshFolder,"robot_ver2_"+names(i)+".STL"));
end
collision.available = all(isfile(meshFiles));
collision.schemaVersion = 1;
collision.source = "URDF STL meshes used as visual and collision geometry";
collision.status = "PRELIMINARY";
collision.minimumClearance = 0.010;
collision.bodyNames = names;
collision.meshFiles = meshFiles;
collision.approximation = "MATLAB represents mesh collision geometry using convex meshes.";
collision.knownHomeCollision = strings(0,2);
collision.knownHomeCollisionStatus = "NONE_DETECTED";
collision.notes = "No pair is skipped; sampled configurations can still collide.";
end
