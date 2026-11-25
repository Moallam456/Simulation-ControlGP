function [worst,linearErrors,angularErrors,metrics] = pathProgressDiagnoseTwist(data,robot,solveSample)
%PATHPROGRESSDIAGNOSETWIST Separate target geometry from Jacobian implementation.
% All vectors are in the base frame. No divisions by speed are used.

tcp = char(robot.structure.frames.endEffector);
metrics = table();
n = numel(data.tGlobal);
exact = isfield(data.segments,'exactCircle');
linearErrors = zeros(1,n);
angularErrors = zeros(1,n);
for segmentIndex = 1:numel(data.segments)
    segment = data.segments(segmentIndex);
    fit = data.fit(segmentIndex);
    i = fit.globalIndices;
    timing = segment.timing(:,end-numel(i)+1:end);
    s = timing(2,:);
    sd = timing(3,:);
    distance = segment.length*s;
    if exact
        [~,ps] = pathProgressCircleGeometry(segment.exactCircle,s);
        tangent = ps./vecnorm(ps,2,1);
        desired = ps.*sd;
        cornerDistance = nan(size(s));
        cornerJump = nan(size(s));
    else
        chord = discretize(distance,segment.arcLength);
        slopes = diff(segment.arcGeometry,1,2)./diff(segment.arcLength);
        tangent = slopes(:,chord)./vecnorm(slopes(:,chord),2,1);
        desired = slopes(:,chord).*(segment.length*sd);
        [cornerDistance,corner] = min(abs(distance-segment.arcLength(2:end-1).'),[],1);
        cornerJump = vecnorm(slopes(:,corner+1)-slopes(:,corner),2,1).*segment.length.*sd;
    end
    actual = zeros(6,numel(s));
    fd = actual;
    requestedAngular = zeros(3,numel(s));

    moving = find(sd~=0);
    h = min([1e-6*ones(size(moving));s(moving)/2;(1-s(moving))/2],[],1);
    delta = 2*h;
    qPlus = pathProgressEvaluateCubic(fit.nodes,fit.coefficients,s(moving)+h);
    qMinus = pathProgressEvaluateCubic(fit.nodes,fit.coefficients,s(moving)-h);
    targetPlus = pathProgressPose(segment,s(moving)+h);
    targetMinus = pathProgressPose(segment,s(moving)-h);
    for k = 1:numel(i)
        J = geometricJacobian(robot.model,data.qPath(:,i(k)).',tcp);
        actual(:,k) = J*data.qDot(:,i(k));
    end
    for j = 1:numel(moving)
        k = moving(j);
        plus = getTransform(robot.model,qPlus(:,j).',tcp);
        minus = getTransform(robot.model,qMinus(:,j).',tcp);
        scale = sd(k)/delta(j);
        fd(:,k) = [rotationVector(plus(1:3,1:3)*minus(1:3,1:3).'); ...
            plus(1:3,4)-minus(1:3,4)]*scale;
        requestedAngular(:,k) = rotationVector( ...
            targetPlus(1:3,1:3,j)*targetMinus(1:3,1:3,j).')*scale;
    end
    residual = actual(4:6,:)-desired;
    tangential = sum(residual.*tangent,1);
    crossPath = vecnorm(residual-tangent.*tangential,2,1);
    linearErrors(i) = vecnorm(residual,2,1);
    angularErrors(i) = vecnorm(actual(1:3,:)-requestedAngular,2,1);
    jacobianLinear = vecnorm(actual(4:6,:)-fd(4:6,:),2,1);
    jacobianAngular = vecnorm(actual(1:3,:)-fd(1:3,:),2,1);
    count = numel(i);
    rows = table(repmat(string(data.demo),count,1),i(:),data.tGlobal(i).', ...
        repmat(segmentIndex,count,1),s(:),data.tcpSpeedPath(i).', ...
        vecnorm(desired,2,1).',vecnorm(actual(4:6,:),2,1).', ...
        linearErrors(i).',tangential(:),crossPath(:),angularErrors(i).', ...
        cornerDistance(:),cornerJump(:),jacobianLinear(:),jacobianAngular(:), ...
        'VariableNames',{'Demo','Sample','Time_s','Segment','Progress','PlannedSpeed_m_s', ...
        'DesiredChordSpeed_m_s','ReconstructedSpeed_m_s','LinearError_m_s', ...
        'TangentialError_m_s','CrossPathError_m_s','AngularError_rad_s', ...
        'NearestCornerDistance_m','NearestChordVelocityJump_m_s', ...
        'JacobianVsFkLinear_m_s','JacobianVsFkAngular_rad_s'});
    metrics = [metrics;rows]; %#ok<AGROW>
end
if exact
    metrics = renamevars(metrics,'DesiredChordSpeed_m_s','DesiredExactSpeed_m_s');
end
[~,linear] = max(metrics.LinearError_m_s);
[~,angular] = max(metrics.AngularError_rad_s);
worst = addvars(metrics([linear angular],:),["linear";"angular"], ...
    'After','Demo','NewVariableNames','Peak');
worst.MaxJacobianVsFkLinear_m_s = repmat(max(metrics.JacobianVsFkLinear_m_s),2,1);
worst.MaxJacobianVsFkAngular_rad_s = repmat(max(metrics.JacobianVsFkAngular_rad_s),2,1);
direct = zeros(2,2);
for k = 1:height(worst)
    direct(k,:) = directIkProbe(data,worst.Sample(k),robot,tcp,solveSample);
end
worst.DirectIkLinearError_m_s = direct(:,1);
worst.DirectIkAngularError_rad_s = direct(:,2);
end

function residual = directIkProbe(data,sample,robot,tcp,solveSample)
% Differentiate direct IK + URDF FK inside the selected Cartesian chord.
% This separates nominal IK/FK residual from the fitted joint derivative.
k = data.segmentIndexPath(sample);
segment = data.segments(k);
fit = data.fit(k);
local = sample-fit.globalIndices(end)+size(segment.timing,2);
s = segment.timing(2,local);
sd = segment.timing(3,local);
residual = [0 0];
if sd == 0, return; end
if isfield(segment,'exactCircle')
    bounds = [0,1];
else
    chord = discretize(segment.length*s,segment.arcLength);
    bounds = segment.arcLength(chord:chord+1)/segment.length;
end
queries = [max(bounds(1),s-1e-6),min(bounds(2),s+1e-6)];
target = pathProgressPose(segment,queries);
actual = zeros(4,4,2);
for i = 1:2
    [q,info] = solveSample(target(:,:,i),data.qPath(:,sample));
    assert(info.success,'Direct IK derivative probe failed at sample %d.',sample);
    actual(:,:,i) = getTransform(robot.model,q.',tcp);
end
scale = sd/(queries(2)-queries(1));
actualAngular = rotationVector(actual(1:3,1:3,2)*actual(1:3,1:3,1).');
targetAngular = rotationVector(target(1:3,1:3,2)*target(1:3,1:3,1).');
if isfield(segment,'exactCircle')
    [~,ps] = pathProgressCircleGeometry(segment.exactCircle,s);
    linearError = scale*reshape(diff(actual(1:3,4,:),1,3),3,1)-ps*sd;
else
    linearError = scale*(diff(actual(1:3,4,:),1,3)-diff(target(1:3,4,:),1,3));
end
residual = [norm(linearError(:)),scale*norm(actualAngular-targetAngular)];
end

function v = rotationVector(R)
skew = [R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
scale = norm(skew);
v = skew;
if scale > 0
    v = skew*(atan2(scale,trace(R)-1)/scale);
end
end
