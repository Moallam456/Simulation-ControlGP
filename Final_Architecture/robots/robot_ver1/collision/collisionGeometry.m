function collision = collisionGeometry(params,structure,chain) %#ok<INUSD>
% COLLISIONGEOMETRY Metadata for the collision meshes imported from URDF.

robotFolder = fileparts(fileparts(mfilename('fullpath')));
meshFolder = fullfile(robotFolder,'CAD','robot_ver1','meshes');
meshNames = ["Base_Link" structure.bodyNames] + ".STL";
meshFiles = strings(size(meshNames));
for i = 1:numel(meshNames)
    meshFiles(i) = string(fullfile(meshFolder,meshNames(i)));
end

collision.available = all(isfile(meshFiles));
collision.source = "URDF STL meshes used as visual and collision geometry";
collision.bodyNames = ["Base_Link" structure.bodyNames];
collision.meshFiles = meshFiles;
collision.approximation = "MATLAB represents mesh collision geometry using convex meshes.";
collision.knownHomeCollision = ["Link_3" "Wrist_2"];
collision.knownHomeCollisionStatus = "UNRESOLVED";
collision.notes = "Do not interpret the known pair as validated clearance or suppress it.";

end
