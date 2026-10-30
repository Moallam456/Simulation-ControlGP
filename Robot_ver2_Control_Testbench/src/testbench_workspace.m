function data=testbench_workspace(robot,cfg,ax)
% Configuration-based usable workspace; not an envelope-based safety gate.
if nargin<3,figure;ax=axes;end
stream=RandStream('mt19937ar','Seed',17);n=2000;
q=cfg.provisionalLimits_rad(:,1).'+rand(stream,n,6).*diff(cfg.provisionalLimits_rad,1,2).';
points=NaN(n,3);accepted=false(n,1);index=zeros(n,1);
for k=1:n
    J=geometricJacobian(robot,q(k,:),'tcp');s=svd(J(4:6,:));index(k)=s(end)/s(1);
    if index(k)<cfg.minimumLinearInverseCondition,continue;end
    evidence=testbench_safety(robot,q(k,:),cfg);
    if evidence.accepted,T=getTransform(robot,q(k,:),'tcp');points(k,:)=T(1:3,4);accepted(k)=true;end
end
hold(ax,'on');scatter3(ax,points(accepted,1),points(accepted,2),points(accepted,3),6,index(accepted),'filled');
data=struct('q',q,'tcp_m',points,'accepted',accepted,'linearIndex',index);
setappdata(ax,'UsableWorkspace',data);
end
