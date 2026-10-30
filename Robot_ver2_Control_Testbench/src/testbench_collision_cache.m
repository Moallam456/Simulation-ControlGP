function cache=testbench_collision_cache(robot)
% Per-body radii and a fallback global displacement bound, valid at every angle.
% For revolute joints, point displacement <= L*sum(abs(delta q)). Any pair's
% separation can shrink by at most twice this value. No point cloud lookup.
translation=0;radius=0;skipped=cell(numel(robot.Bodies),2);
bodyOrder=[robot.BodyNames {robot.BaseName}];bodyRadii=zeros(numel(bodyOrder),1);
for i=1:numel(robot.Bodies)
    b=robot.Bodies{i};T=getTransform(robot,zeros(1,6),b.Name,b.Parent.Name);
    translation=translation+norm(T(1:3,4));
    skipped(i,:)={b.Parent.Name,b.Name};
end
% R2024b exposes imported collision descriptions as strings, not shape objects.
% Read the same authoritative URDF's collision mesh, scale and origin instead.
root=fileparts(fileparts(mfilename('fullpath')));
package=fullfile(root,'mechanics','URDF_Assembly_Robot_ver2');
doc=xmlread(fullfile(package,'urdf','URDF_Assembly_Robot_ver2.urdf'));
collisions=doc.getElementsByTagName('collision');hashes=cell(collisions.getLength,1);
for k=0:collisions.getLength-1
    node=collisions.item(k);meshes=node.getElementsByTagName('mesh');
    if meshes.getLength~=1,radius=Inf;break;end
    mesh=meshes.item(0);uri=char(mesh.getAttribute('filename'));
    prefix='package://URDF_Assembly_Robot_ver2/';
    assert(startsWith(uri,prefix),'testbench:CollisionURI','Unexpected collision mesh package.');
    path=fullfile(package,uri(numel(prefix)+1:end));
    shape=stlread(path);scale=ones(1,3);offset=zeros(1,3);
    if mesh.hasAttribute('scale'),scale=sscanf(char(mesh.getAttribute('scale')),'%f').';end
    origins=node.getElementsByTagName('origin');
    if origins.getLength>0,offset=sscanf(char(origins.item(0).getAttribute('xyz')),'%f').';end
    assert(numel(scale)==3 && numel(offset)==3);
    localRadius=norm(offset)+max(vecnorm(shape.Points.*scale,2,2));
    radius=max(radius,localRadius);
    body=char(node.getParentNode().getAttribute('name'));
    index=find(strcmp(bodyOrder,body),1);assert(~isempty(index));bodyRadii(index)=max(bodyRadii(index),localRadius);
    hashes{k+1}=testbench_hash(path);
end
cache=struct('leverBound_m',translation+radius,'q',[],'report',[], ...
    'exactChecks',0,'boundChecks',0,'meshHashes',{hashes}, ...
    'skippedPairs',{skipped},'useNativeJacobian',exist('rb2_native','file')==3, ...
    'bodyOrder',{bodyOrder},'bodyRadii_m',bodyRadii,'transforms',[], ...
    'selfDistances',[],'worldDistances',[]);
end
