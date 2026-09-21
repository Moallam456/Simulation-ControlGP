function [q,qd,qdd] = computeJointDerivatives(sNodes,qNodes,s,sd,sdd)
% COMPUTEJOINTDERIVATIVES Differentiate a joint spline by the chain rule.
% The IK path is solved versus geometric progress, not numerically twice in t.
sNodes=sNodes(:); s=s(:); sd=sd(:); sdd=sdd(:);
if numel(sNodes)<4 || any(diff(sNodes)<=0) || ...
        size(qNodes,1)~=numel(sNodes) || ...
        ~isequal(size(s),size(sd),size(sdd))
    error('trajectory:InvalidJointPath','Invalid spatial IK path or timing arrays.');
end
n=numel(s); dof=size(qNodes,2);
q=zeros(n,dof); qd=q; qdd=q;
for j=1:dof
    pp=spline(sNodes,qNodes(:,j).');
    [breaks,coefs]=unmkpp(pp);
    first=mkpp(breaks,[3*coefs(:,1) 2*coefs(:,2) coefs(:,3)]);
    second=mkpp(breaks,[6*coefs(:,1) 2*coefs(:,2)]);
    q(:,j)=ppval(pp,s);
    qs=ppval(first,s);
    qss=ppval(second,s);
    qd(:,j)=qs.*sd;
    qdd(:,j)=qss.*sd.^2+qs.*sdd;
end
end
