function [peaks,knots,convergence,phaseJumps] = pathProgressDiagnoseCircle(circle,robot)
%PATHPROGRESSDIAGNOSECIRCLE Audit the saved cubic without changing its fit.
segment = circle.segments(1);
fit = circle.fit(1);
phases = timingPhases(segment);
nodes = fit.nodes;
knotTimes = arrayfun(@(s) progressTime(phases,s),nodes(2:end-1));
dof = size(fit.q,1);
left = zeros(dof,numel(knotTimes));
right = left;
for k = 1:numel(knotTimes)
    leftState = stateAtTime(phases,knotTimes(k),false);
    rightState = stateAtTime(phases,knotTimes(k),true);
    left(:,k) = intervalJerk(fit.coefficients(:,k,:),nodes(k+1)-nodes(k),leftState);
    right(:,k) = intervalJerk(fit.coefficients(:,k+1,:),0,rightState);
end
knots = table(repelem((2:numel(nodes)-1).',dof),repelem(knotTimes(:),dof), ...
    repelem(nodes(2:end-1).',dof),repmat((1:dof).',numel(knotTimes),1), ...
    left(:),right(:),right(:)-left(:), ...
    'VariableNames',{'Node','Time_s','Progress','Joint','Left_rad_s3','Right_rad_s3','Jump_rad_s3'});

[~,sample] = max(abs(circle.qJerk(:)));
[joint,sample] = ind2sub(size(circle.qJerk),sample);
sampleTime = circle.tGlobal(sample);
sampleProgress = segment.timing(2,sample);
[~,~,~,~,sampleInterval] = pathProgressEvaluateCubic(nodes,fit.coefficients,sampleProgress);
sampled = [joint,sampleTime,sampleProgress,sample,circle.qJerk(joint,sample),sampleInterval];
[continuous,side] = continuousPeak(phases,fit,knotTimes,circle.tGlobal);
values = [sampled;continuous];
peaks = addvars(array2table(values,'VariableNames',{'Joint','Time_s','Progress', ...
    'NearestSample','SignedJerk_rad_s3','PolynomialInterval'}), ...
    ["sampled";"continuous one-sided"],'Before','Joint','NewVariableNames','Search');
peaks.ArcLength_m = values(:,3)*segment.length;
peaks.AbsJerk_rad_s3 = abs(values(:,5));
peaks.Side = ["execution sample";side];
assert(peaks.AbsJerk_rad_s3(2)+1e-6 >= peaks.AbsJerk_rad_s3(1), ...
    'Continuous search missed the sampled peak.');
convergence = denseFk(segment,fit,robot);
if nargout > 3
    phaseJumps = phaseBoundaryJerk(phases,fit);
end
end

function jumps = phaseBoundaryJerk(phases,fit)
times = phases(1:end-1,2);
dof = size(fit.q,1);
left = zeros(dof,numel(times));
right = left;
progress = zeros(size(times));
for k = 1:numel(times)
    a = stateAtTime(phases,times(k),false);
    b = stateAtTime(phases,times(k),true);
    s = a(1);
    progress(k) = s;
    li = find(fit.nodes<s,1,'last');
    ri = find(fit.nodes<=s,1,'last');
    left(:,k) = intervalJerk(fit.coefficients(:,li,:),s-fit.nodes(li),a);
    right(:,k) = intervalJerk(fit.coefficients(:,ri,:),s-fit.nodes(ri),b);
end
jumps = table(repelem(times,dof),repelem(progress,dof), ...
    repmat((1:dof).',numel(times),1),left(:),right(:),right(:)-left(:), ...
    'VariableNames',{'Time_s','Progress','Joint','Left_rad_s3','Right_rad_s3','Jump_rad_s3'});
end

function phases = timingPhases(segment)
% Same constant-jerk phases as sCurveTimeScaling. Integrate them continuously
% for knot times and check the reconstruction against the stored samples.
duration = segment.profile.segmentDurations(:).';
jerks = segment.profile.jPeak*[1 0 -1 0 -1 0 1];
state = [0 0 0];
t = 0;
phases = zeros(0,7);
for k = 1:numel(duration)
    h = duration(k);
    if h == 0, continue; end
    j = jerks(k);
    finish = integrate([state,j],h);
    phases(end+1,:) = [t,t+h,state,j,finish(1)]; %#ok<AGROW>
    t = t+h;
    state = finish(1:3);
end
for k = 1:size(segment.timing,2)
    time = segment.timing(1,k);
    state = stateAtTime(phases,time,true);
    assert(max(abs(state(1:3).'-segment.timing(2:4,k)))<=1e-12, ...
        'Continuous timing reconstruction disagrees with the saved S-curve.');
end
end

function state = integrate(initial,h)
s = initial(1);
sd = initial(2);
sdd = initial(3);
j = initial(4);
state = [s+sd*h+sdd*h^2/2+j*h^3/6,sd+sdd*h+j*h^2/2,sdd+j*h,j];
end

function index = phaseAtTime(phases,t,rightSide)
if rightSide
    index = find(phases(:,2)>t,1);
else
    index = find(phases(:,2)>=t,1);
end
if isempty(index), index = size(phases,1); end
end

function state = stateAtTime(phases,t,rightSide)
i = phaseAtTime(phases,t,rightSide);
state = integrate(phases(i,3:6),t-phases(i,1));
end

function time = progressTime(phases,s)
k = find(phases(:,7)>=s,1);
a = phases(k,1);
b = phases(k,2);
for i = 1:52
    mid = (a+b)/2;
    state = integrate(phases(k,3:6),mid-phases(k,1));
    if state(1) < s
        a = mid;
    else
        b = mid;
    end
end
time = (a+b)/2;
end

function jerk = intervalJerk(coef,u,state)
qs = coef(:,:,2)+2*coef(:,:,3)*u+3*coef(:,:,4)*u^2;
qss = 2*coef(:,:,3)+6*coef(:,:,4)*u;
jerk = 6*coef(:,:,4)*state(2)^3+3*qss*state(2)*state(3)+qs*state(4);
end

function [peak,side] = continuousPeak(phases,fit,knotTimes,sampleTimes)
% Within each timing-phase/spatial-interval intersection, q(t) is at most
% degree nine. Check real roots of d^4q/dt^4 and both one-sided boundaries.
cuts = unique([0,knotTimes,phases(:,2).']);
peak = zeros(1,6);
side = "interior";
for i = 1:numel(cuts)-1
    start = cuts(i);
    width = cuts(i+1)-start;
    middle = stateAtTime(phases,start+width/2,true);
    interval = find(fit.nodes<=middle(1),1,'last');
    phase = phaseAtTime(phases,start+width/2,true);
    initial = integrate(phases(phase,3:6),start-phases(phase,1));
    u = [initial(1)-fit.nodes(interval),initial(2)*width, ...
        initial(3)*width^2/2,initial(4)*width^3/6];
    u2 = conv(u,u);
    u3 = conv(u2,u);
    for j = 1:size(fit.q,1)
        c = reshape(fit.coefficients(j,interval,:),1,[]);
        q = [c(1),zeros(1,9)] + c(2)*[u,zeros(1,6)] + ...
            c(3)*[u2,zeros(1,3)] + c(4)*u3;
        jerkPoly = q(4:end).*(3:9).*(2:8).*(1:7);
        stationary = roots(fliplr(jerkPoly(2:end).*(1:6)));
        inside = real(stationary(abs(imag(stationary))<1e-9 & ...
            real(stationary)>0 & real(stationary)<1));
        candidates = [0;inside;1];
        for f = candidates.'
            time = start+f*width;
            state = integrate(initial,f*width);
            jerk = intervalJerk(fit.coefficients(j,interval,:),state(1)-fit.nodes(interval),state);
            if abs(jerk)>abs(peak(5))
                [~,sample] = min(abs(sampleTimes-time));
                peak = [j,time,state(1),sample,jerk,interval];
                if f == 0
                    side = "right";
                elseif f == 1
                    side = "left";
                else
                    side = "interior";
                end
            end
        end
    end
end
end

function convergence = denseFk(segment,fit,robot)
% Uniform nested grids use no production collocation data. Also evaluate
% exact Cartesian corners and joint polynomial knots as separate candidates.
tcp = char(robot.structure.frames.endEffector);
fixed = unique([fit.nodes,segment.arcLength/segment.length]);
fixedMax = fkMax(segment,fit,robot,tcp,fixed);
orders = [8192 16384 32768 65536];
rows = zeros(numel(orders),11);
grid = zeros(1,5);
for k = 1:numel(orders)
    n = orders(k);
    if k == 1
        s = (0:n)/n;
    else
        s = (1:2:n-1)/n;
    end
    grid = mergeMax(grid,fkMax(segment,fit,robot,tcp,s));
    combined = mergeMax(grid,fixedMax);
    rows(k,:) = [n+1,grid(1:4),combined(1:4),grid(5),combined(5)];
end
convergence = array2table(rows,'VariableNames',{'GridPoints','GridPosition_m', ...
    'GridPositionProgress','GridRotation_rad','GridRotationProgress', ...
    'WithKnotsPosition_m','WithKnotsPositionProgress','WithKnotsRotation_rad', ...
    'WithKnotsRotationProgress','GridLimitViolation','WithKnotsLimitViolation'});
convergence.WithinOriginalFkTolerance = convergence.WithKnotsPosition_m<=1e-5 & ...
    convergence.WithKnotsRotation_rad<=1e-4;
end

function worst = fkMax(segment,fit,robot,tcp,s)
desired = pathProgressPose(segment,s);
q = pathProgressEvaluateCubic(fit.nodes,fit.coefficients,s);
limits = robot.params.joints.positionLimits;
worst = [0,0,0,0,any(~isfinite(q) | q<limits(:,1) | q>limits(:,2),'all')];
for i = 1:numel(s)
    actual = getTransform(robot.model,q(:,i).',tcp);
    position = norm(actual(1:3,4)-desired(1:3,4,i));
    R = desired(1:3,1:3,i).'*actual(1:3,1:3);
    rotation = atan2(norm([R(3,2)-R(2,3),R(1,3)-R(3,1),R(2,1)-R(1,2)]),trace(R)-1);
    if position > worst(1), worst(1:2) = [position,s(i)]; end
    if rotation > worst(3), worst(3:4) = [rotation,s(i)]; end
end
end

function result = mergeMax(a,b)
result = a;
if b(1)>a(1), result(1:2) = b(1:2); end
if b(3)>a(3), result(3:4) = b(3:4); end
result(5) = a(5) || b(5);
end
