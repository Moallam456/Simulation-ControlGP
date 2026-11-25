function [qn,vn,lo,hi] = pathProgressDriveStep(q,v,u,tau,h)
%PATHPROGRESSDRIVESTEP Exact constant-input update. q: rad, v/u: rad/s.
% tau is a per-joint column [s]. h is a scalar or one interval per column.
% Optional lo/hi include internal position extrema for a single interval.
qn = q;
vn = v;
for k = 1:size(q,1)
    if tau(k)==0
        qn(k,:) = q(k,:)+u(k,:).*h;
        vn(k,:) = u(k,:);
    else
        x = h/tau(k);
        response = -expm1(-x);
        area = h-tau(k)*response;
        small = x<1e-3;
        z = x(small);
        area(small) = h(small).*z.*(.5+z.*(-1/6+z.*(1/24+z.*(-1/120+z/720))));
        delta = u(k,:)-v(k,:);
        qn(k,:) = q(k,:)+v(k,:).*h+delta.*area;
        vn(k,:) = v(k,:)+delta.*response;
    end
end
if nargout<3, return; end
lo = min(q,qn);
hi = max(q,qn);
for k = 1:numel(tau)
    if tau(k)>0 && v(k)*vn(k)<0
        turningTime = tau(k)*log1p(-v(k)/u(k));
        turning = pathProgressDriveStep(q(k),v(k),u(k),tau(k),turningTime);
        lo(k) = min(lo(k),turning);
        hi(k) = max(hi(k),turning);
    end
end
end
