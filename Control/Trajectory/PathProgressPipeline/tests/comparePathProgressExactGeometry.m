function summary = comparePathProgressExactGeometry()
%COMPAREPATHPROGRESSEXACTGEOMETRY Compare preserved chord runs with exact circles.
root = fileparts(fileparts(mfilename('fullpath')));
trajectory = fileparts(root);
results = fullfile(root,'results');
output = fullfile(results,'exact_geometry_comparison');
if ~isfolder(output), mkdir(output); end
previousPath = path;
restore = onCleanup(@() path(previousPath));
addpath(fullfile(root,'tests'),fullfile(root,'helpers'),fullfile(trajectory,'Pipeline'), ...
    fullfile(fileparts(trajectory),'Kinematics'));
saved = load(fullfile(results,'path_progress_after.mat'),'after','traces');
summary = table();
denseResults = table();
boundaries = table();
for demo = ["arc","fullcircle"]
    chord = saved.after.(demo);
    expectedPath = path;
    file = fullfile(root,"PathProgressDemo_"+demo+"_exact.m");
    exact = runExact(file);
    assert(strcmp(path,expectedPath),'The exact entry point leaked MATLAB path changes.');
    [robot,~,solveSample] = loadTrajectoryRobot();
    geometry = checkGeometry(demo,chord,exact);
    [a,aDense,aBoundaries,chordAudit] = assess(demo,chord,"Chord",robot,solveSample);
    assert(max(abs(chordAudit.twist.LinearError_m_s.'-saved.traces.(demo).linearError))<=1e-12, ...
        'The chord diagnostic differs from the preserved run.');
    [b,bDense,bBoundaries,exactAudit] = assess(demo,exact,"Exact circle",robot,solveSample);
    summary = [summary;a;b]; %#ok<AGROW>
    denseResults = [denseResults;aDense;bDense]; %#ok<AGROW>
    boundaries = [boundaries;aBoundaries;bBoundaries]; %#ok<AGROW>
    runs.(demo) = struct('exact',exact,'geometry',geometry,'chordAudit',chordAudit, ...
        'exactAudit',exactAudit);
    pathProgressPlotExactComparison(demo,chord,exact,chordAudit,exactAudit,geometry,output);
    fprintf('%s: completed chord and exact comparisons (%d samples each).\n',demo,numel(exact.tGlobal));
end
writetable(summary,fullfile(output,'comparison.csv'));
writetable(denseResults,fullfile(output,'dense_fk.csv'));
writetable(boundaries,fullfile(output,'jerk_boundaries.csv'));
save(fullfile(output,'exact_geometry_runs.mat'),'runs');
disp(summary);
assert(all(summary.WithinExistingFkLimits), 'A dense FK check exceeded the original tolerances.');
end

function data = runExact(entry)
pathProgressRender = false;
run(entry);
assert(isequal(qPath,pathProgressData.qPath) && isequal(qDot,pathProgressData.qDot) && ...
    isequal(qDDot,pathProgressData.qDDot),'The exact entry point did not expose the joint references.');
data = pathProgressData;
end

function geometry = checkGeometry(demo,chord,exact)
segment = exact.segments(1);
old = chord.segments(1);
circle = segment.exactCircle;
assert(isequal(exact.qStart,chord.qStart) && isequal(exact.tGlobal,chord.tGlobal) && ...
    isequal(segment.rawGeometry,old.rawGeometry) && isequal(segment.timing,old.timing) && ...
    isequal(segment.length,old.length) && ...
    isequal(exact.TPath(1:3,1:3,:),chord.TPath(1:3,1:3,:)), ...
    'The controlled comparison changed the teaching geometry, orientation or timing.');
assert(max(abs(exact.qPath(:,end)-chord.qPath(:,end)))<=1e-4, ...
    'The exact variant ended on a different IK branch.');
origin = chord.TStart(1:3,4);
if demo == "arc"
    middle = [0.20;0.14;0];
    finish = [0.40;0;0];
else
    middle = [0.15;0.15;0];
    finish = [0.30;0;0];
end
% Independent circumcenter for these planar teaching points. Equal distances
% from the origin and finish fix the X coordinate before solving for Y.
cx = finish(1)/2;
cy = (dot(middle,middle)-2*cx*middle(1))/(2*middle(2));
center = origin+[cx;cy;0];
radius = hypot(cx,cy);
normal = cross(middle,finish);
normal = normal/norm(normal);
u = (origin-center)/radius;
v = cross(normal,u);
middleRadial = origin+middle-center;
endRadial = origin+finish-center;
middleAngle = mod(atan2(dot(v,middleRadial),dot(u,middleRadial)),2*pi);
endAngle = mod(atan2(dot(v,endRadial),dot(u,endRadial)),2*pi);
if demo == "arc"
    sweep = endAngle;
    endpoint = origin+finish;
else
    sweep = 2*pi;
    endpoint = origin;
end
tol = 1e-12;
assert(norm(circle.center-center)<=tol && abs(circle.radius-radius)<=tol && ...
    norm(circle.axis-normal)<=tol && abs(circle.thetaTotal-sweep)<=tol && ...
    middleAngle>0 && middleAngle<sweep, 'Circle metadata or intended sweep changed.');

progress = (0:4096)/4096;
[p,ps,pss,psss] = pathProgressCircleGeometry(circle,progress);
radial = p-center;
angles = unwrap(atan2(v.'*radial,u.'*radial));
geometry = struct('teachingPoints',[origin,origin+middle,origin+finish], ...
    'radiusError_m',max(abs(vecnorm(radial,2,1)-radius)), ...
    'planeError_m',max(abs(normal.'*radial)), ...
    'sweepError_rad',max(abs(angles-progress*sweep)), ...
    'startError_m',norm(p(:,1)-origin),'endError_m',norm(p(:,end)-endpoint), ...
    'middleError_m',norm(pathProgressCircleGeometry(circle,middleAngle/sweep)-(origin+middle)));
if demo == "fullcircle"
    geometry.throughEndTeachingPointError_m = ...
        norm(pathProgressCircleGeometry(circle,endAngle/sweep)-(origin+finish));
end
errors = rmfield(geometry,'teachingPoints');
assert(all(cellfun(@(x) x<=tol,struct2cell(errors))), ...
    'Radius, plane membership, endpoints, through-point, or signed sweep failed.');

h = 1e-6;
interior = 2:numel(progress)-1;
[pp,sp,ap] = pathProgressCircleGeometry(circle,progress(interior)+h);
[pm,sm,am] = pathProgressCircleGeometry(circle,progress(interior)-h);
geometry.derivativeErrors = [max(vecnorm((pp-pm)/(2*h)-ps(:,interior),2,1)), ...
    max(vecnorm((sp-sm)/(2*h)-pss(:,interior),2,1)), ...
    max(vecnorm((ap-am)/(2*h)-psss(:,interior),2,1))];
assert(all(geometry.derivativeErrors<=5e-8),'Analytic circle derivatives failed the centered checks.');
end

function [summary,dense,boundaries,audit] = assess(demo,data,label,robot,solveSample)
fit = data.fit(1);
segment = data.segments(1);
isExact = isfield(segment,'exactCircle');
length = sum(vecnorm(diff(segment.arcGeometry,1,2),2,1));
speedField = "DesiredChordSpeed_m_s";
if isExact
    length = segment.exactLength;
    speedField = "DesiredExactSpeed_m_s";
end
[worst,~,~,twist] = pathProgressDiagnoseTwist(data,robot,solveSample);
[peaks,knots,dense,phases] = pathProgressDiagnoseCircle(data,robot);
assert(max(worst.MaxJacobianVsFkLinear_m_s)<=1e-8 && ...
    max(worst.MaxJacobianVsFkAngular_rad_s)<=1e-8, ...
    'Jacobian and independent FK velocity estimates differ.');
limit = robot.params.joints.positionLimits;
nodeStep = max(abs(diff(fit.qNodes,1,2)),[],'all');
executionStep = max(abs(diff(data.qPath,1,2)),[],'all');
assert(all(isfinite([data.qPath,data.qDot,data.qDDot,data.qJerk]),'all') && ...
    all(data.qPath>=limit(:,1) & data.qPath<=limit(:,2),'all') && ...
    all(dense.WithKnotsLimitViolation==0) && max(nodeStep,executionStep)<=0.10, ...
    'Joint limits or branch continuity failed.');
assert(all(data.qDot(:,[1 end])==0,'all') && all(data.qDDot(:,[1 end])==0,'all') && ...
    max(abs(data.qPath(:,1)-data.qStart))<=1e-4,'Endpoint stop or initial branch failed.');
if demo == "fullcircle"
    assert(max(abs(data.qPath(:,end)-data.qPath(:,1)))<=1e-4, ...
        'The full circle finished on a different joint branch.');
end
desiredSpeed = twist.(speedField);
if isExact
    assert(max(abs(desiredSpeed-data.tcpSpeedPath.'))<=1e-12, ...
        'Reported exact path speed differs from the analytic tangent speed.');
end

direct = directFkProbe(data,[dense.WithKnotsPositionProgress(end), ...
    dense.WithKnotsRotationProgress(end)],robot,solveSample);
final = dense(end,:);
summary = struct2table(struct('Demo',demo,'Reference',label,'Samples',numel(data.tGlobal), ...
    'Duration_s',data.tGlobal(end),'Length_m',length,'TimingChordLength_m',segment.length, ...
    'LengthDelta_m',length-segment.length,'Nodes',numel(fit.nodes), ...
    'Refinements',fit.refinementPasses,'TimedFkPosition_m',max(data.positionError), ...
    'TimedFkRotation_rad',max(data.orientationError),'DenseFkPosition_m',final.WithKnotsPosition_m, ...
    'DenseFkRotation_rad',final.WithKnotsRotation_rad,'DirectIkAtPositionPeak_m',direct(1,1), ...
    'DirectIkAtRotationPeak_rad',direct(2,2),'PeakDesiredSpeed_m_s',max(desiredSpeed), ...
    'PeakReconstructedSpeed_m_s',max(twist.ReconstructedSpeed_m_s), ...
    'PeakSpeedError_m_s',max(abs(twist.ReconstructedSpeed_m_s-desiredSpeed)), ...
    'PeakAbsTangentialError_m_s',max(abs(twist.TangentialError_m_s)), ...
    'PeakPerpendicularError_m_s',max(twist.CrossPathError_m_s), ...
    'PeakLinearError_m_s',max(twist.LinearError_m_s),'PeakAngularError_rad_s',max(twist.AngularError_rad_s), ...
    'DirectIkLinearAtTwistPeak_m_s',worst.DirectIkLinearError_m_s(1), ...
    'DirectIkAngularAtTwistPeak_rad_s',worst.DirectIkAngularError_rad_s(2), ...
    'PeakJointVelocity_rad_s',max(abs(data.qDot),[],'all'), ...
    'PeakJointAcceleration_rad_s2',max(abs(data.qDDot),[],'all'), ...
    'SampledPeakJerk_rad_s3',peaks.AbsJerk_rad_s3(1), ...
    'ContinuousPeakJerk_rad_s3',peaks.AbsJerk_rad_s3(2), ...
    'PeakKnotJerkJump_rad_s3',max(abs(knots.Jump_rad_s3)), ...
    'PeakPhaseJerkJump_rad_s3',max(abs(phases.Jump_rad_s3)), ...
    'MaxNodeStep_rad',nodeStep,'MaxExecutionStep_rad',executionStep, ...
    'WithinExistingFkLimits',all(dense.WithinOriginalFkTolerance)));
dense = labelRows(dense,demo,label);
knots = addvars(knots,repmat("Polynomial knot",height(knots),1),'Before','Node','NewVariableNames','Event');
phases = addvars(phases,nan(height(phases),1),'Before','Time_s','NewVariableNames','Node');
phases = addvars(phases,repmat("Timing phase",height(phases),1),'Before','Node','NewVariableNames','Event');
boundaries = labelRows([knots;phases],demo,label);
audit = struct('twist',twist,'desiredSpeed',desiredSpeed,'peaks',peaks,'worstTwist',worst, ...
    'knots',knots,'phases',phases,'directFkAtDensePeaks',direct);
end

function rows = labelRows(rows,demo,label)
rows = addvars(rows,repmat(demo,height(rows),1),repmat(label,height(rows),1), ...
    'Before',1,'NewVariableNames',{'Demo','Reference'});
end

function errors = directFkProbe(data,progress,robot,solveSample)
segment = data.segments(1);
fit = data.fit(1);
target = pathProgressPose(segment,progress);
seeds = pathProgressEvaluateCubic(fit.nodes,fit.coefficients,progress);
errors = zeros(numel(progress),2);
for i = 1:numel(progress)
    [q,info] = solveSample(target(:,:,i),seeds(:,i));
    assert(info.success,'Direct IK residual probe failed at progress %.10g.',progress(i));
    achieved = getTransform(robot.model,q.',char(robot.structure.frames.endEffector));
    R = target(1:3,1:3,i).'*achieved(1:3,1:3);
    skew = [R(3,2)-R(2,3),R(1,3)-R(3,1),R(2,1)-R(1,2)];
    errors(i,:) = [norm(achieved(1:3,4)-target(1:3,4,i)),atan2(norm(skew),trace(R)-1)];
end
end
