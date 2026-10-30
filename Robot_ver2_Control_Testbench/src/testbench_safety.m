function [report,cache] = testbench_safety(robot,q,cfg,previous,startAlreadyChecked,cache)
% TESTBENCH_SAFETY Exact configuration/mesh checks along interpolated joint motion.
% A bounded sampling step is NOT continuous collision detection certification.
if nargin<4, previous=q; end
if nargin<5,startAlreadyChecked=false;end
if nargin<6,cache=[];end
report=struct('accepted',false,'reason','INVALID_INPUT','linearIndex',NaN, ...
    'fullSigma',NaN,'selfClearance_m',NaN,'environmentClearance_m',NaN, ...
    'pair',{{}},'checks',0);
if numel(q)~=6 || numel(previous)~=6 || any(~isfinite([q(:);previous(:)])), return; end
q=reshape(q,1,6); previous=reshape(previous,1,6);
limits=cfg.provisionalLimits_rad;
if any(q<limits(:,1).' | q>limits(:,2).') || ...
        any(previous<limits(:,1).' | previous>limits(:,2).')
    report.reason='JOINT_LIMIT'; return;
end
n=max(1,ceil(max(abs(q-previous))/cfg.motionCheckStep_rad));
world=[];
% Match established analysis policy: connected parent/child CAD interfaces
% are excluded. Every nonadjacent body pair remains enabled.
if ~isempty(cache)
    skipped=cache.skippedPairs;
else
    skipped=cell(numel(robot.Bodies),2);
    for k=1:numel(robot.Bodies),skipped(k,:)={robot.Bodies{k}.Parent.Name,robot.Bodies{k}.Name};end
end
fractions=linspace(0,1,n+1);
if startAlreadyChecked || isequal(q,previous),fractions=fractions(2:end);end
for a=fractions
    x=previous+a*(q-previous); report.checks=report.checks+1;
    if ~isempty(cache) && cache.useNativeJacobian
        J=rb2_native('jacobian',x);
    else
        J=geometricJacobian(robot,x,'tcp');J=J([4:6 1:3],:);
    end
    s=svd(J(1:3,:));report.linearIndex=s(end)/max(s(1),eps);fs=svd(J);
    report.fullSigma=fs(end);
    if report.linearIndex<cfg.minimumLinearInverseCondition
        report.reason='LINEAR_INVERSE_CONDITION'; return;
    end
    if report.fullSigma<cfg.demo.fullJacobianMinimumSigma
        report.reason='FULL_TASK_SINGULARITY'; return;
    end
    if ~isempty(cache) && ~isempty(cache.q)
        if cache.useNativeJacobian && ~isempty(cache.transforms)
            current=rb2_native('frames',x);current=current(:,:,[2:8 1]);
            movement=zeros(numel(cache.bodyOrder),1);
            for body=1:numel(movement)
                translation=norm(current(1:3,4,body)-cache.transforms(1:3,4,body));
                rotation=norm(current(1:3,1:3,body)-cache.transforms(1:3,1:3,body),'fro')/sqrt(2);
                movement(body)=translation+cache.bodyRadii_m(body)*rotation+1e-12;
            end
            ds=cache.selfDistances-movement-movement.';
            upper=triu(true(size(ds)),1);values=ds(upper & isfinite(ds));selfBound=min([Inf;values(:)]);
            dw=cache.worldDistances-movement;values=dw(isfinite(dw));worldBound=min([Inf;values(:)]);
        else
            displacement=2*cache.leverBound_m*sum(abs(x-cache.q));
            selfBound=cache.report.selfClearance_m-displacement;
            worldBound=cache.report.environmentClearance_m-displacement;
        end
        if selfBound>=cfg.minimumSelfClearance_m && worldBound>=cfg.minimumEnvironmentClearance_m
            report.selfClearance_m=selfBound;report.environmentClearance_m=worldBound;
            cache.boundChecks=cache.boundChecks+1;
            continue;
        end
    end
    [hit,d]=checkCollision(robot,x,'Exhaustive','on','SkippedSelfCollisions',skipped);
    selfDistances=d;
    upper=triu(true(size(d)),1); finite=d(upper & isfinite(d));
    report.selfClearance_m=min([Inf;finite(:)]);
    if any(hit(:))
        report.reason='SELF_COLLISION';
        [r,c]=find(upper & isnan(d),1); names=[robot.BodyNames {robot.BaseName}];
        if ~isempty(r), report.pair=names([r c]); end
        return;
    end
    if report.selfClearance_m<cfg.minimumSelfClearance_m
        report.reason='SELF_CLEARANCE'; return;
    end
    if isempty(world)
        world=cell(1,numel(cfg.environment));
        for k=1:numel(world)
            e=cfg.environment(k);world{k}=collisionBox(e.size_m(1),e.size_m(2),e.size_m(3));
            world{k}.Pose=trvec2tform(e.center_m)*eul2tform(e.rpy_rad([3 2 1]));
        end
    end
    [hit,d]=checkCollision(robot,x,world,'IgnoreSelfCollision','on','Exhaustive','on', ...
        'SkippedSelfCollisions',skipped);
    finite=d(isfinite(d)); report.environmentClearance_m=min([Inf;finite(:)]);
    if any(hit(:)), report.reason='ENVIRONMENT_COLLISION'; return; end
    if report.environmentClearance_m<cfg.minimumEnvironmentClearance_m
        report.reason='ENVIRONMENT_CLEARANCE'; return;
    end
    if ~isempty(cache)
        cache.selfDistances=selfDistances;cache.worldDistances=d;
        cache.q=x;cache.report=report;cache.exactChecks=cache.exactChecks+1;
        if cache.useNativeJacobian
            cache.transforms=rb2_native('frames',x);cache.transforms=cache.transforms(:,:,[2:8 1]);
        end
    end
end
report.accepted=true; report.reason='ACCEPTED_SAMPLED_MOTION';
end
