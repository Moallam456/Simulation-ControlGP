function results = verifyPathProgressPipeline()
%VERIFYPATHPROGRESSPIPELINE Check the isolated demos in MATLAB R2024b.
root = fileparts(fileparts(mfilename('fullpath')));
resultsFolder = fullfile(root,'results');
entries = ["line","arc","fullcircle","multisegment"];
baselineNames = ["line","arc","fullcircle","multi"];
oldPath = path;
oldVisible = get(groot,'DefaultFigureVisible');
oldPause = pause('query');
oldFigures = findall(groot,'Type','figure');
cleanup = onCleanup(@() restoreSession(oldPath,oldVisible,oldPause,oldFigures));
addpath(fullfile(root,'helpers'));
trajectory = fileparts(root);
addpath(fullfile(fileparts(trajectory),'Kinematics'));
set(groot,'DefaultFigureVisible','off');
pause('off');

reference = load(fullfile(resultsFolder,'robot_ver2_baseline.mat'));
checkKnownCubic();
for i = 1:numel(entries)
    entry = fullfile(root,"PathProgressDemo_"+entries(i)+".m");
    expectedPath = path;
    [data,robot,fig] = runEntry(entry);
    assert(strcmp(path,expectedPath),'Entry point leaked MATLAB path changes.');
    assert(getappdata(fig,'PathProgressLastFrame') == numel(data.tGlobal) && ...
        ~isempty(findobj(fig,'Style','pushbutton')), 'Animation did not reach the final frame.');
    baseline = reference.baseline.(baselineNames(i));
    [metrics,diagnostics] = checkDemo(data,robot,baseline);
    allMetrics(i) = metrics; %#ok<AGROW>
    after.(baselineNames(i)) = data;
    traces.(baselineNames(i)) = diagnostics;
    savePlots(data,diagnostics,resultsFolder);
    fprintf('%s passed (%d samples, %d IK nodes).\n',data.demo,metrics.Samples,metrics.Nodes);
end
csv = readtable(fullfile(resultsFolder,'path_progress_line.csv'));
assert(isequal(csv.Properties.VariableNames,reference.lineExportColumns) && ...
    height(csv)==numel(after.line.tGlobal) && all(csv.ikIterations==0) && ...
    max(abs(csv{:,16:21}.'-after.line.qPath),[],'all') < 1e-12 && ...
    max(abs(csv{:,22:27}.'-after.line.qDot),[],'all') < 1e-12,'Line CSV schema or data changed.');
results = struct2table(allMetrics);
writetable(results,fullfile(resultsFolder,'path_progress_summary.csv'));
save(fullfile(resultsFolder,'path_progress_after.mat'),'after','traces');
disp(results);
end

function [data,model,fig] = runEntry(entry)
run(entry);
assert(isequal(qPath,pathProgressData.qPath) && isequal(qDot,pathProgressData.qDot) && ...
    isequal(qDDot,pathProgressData.qDDot),'The entry point did not expose joint references.');
data = pathProgressData;
model = robot;
fig = animationFigure;
end

function checkKnownCubic()
nodes = [0 0.11 0.37 0.79 1];
query = [0 0.025 0.11 0.5 0.79 0.82 1];
polynomial = @(s) 1-2*s+3*s.^2-4*s.^3;
coeff = pathProgressBuildCubic(nodes,[polynomial(nodes);2*nodes]);
[q,qs,qss,qsss,interval] = pathProgressEvaluateCubic(nodes,coeff,query);
assert(isequal(interval,[1 1 2 3 4 4 4]) && ...
    max(abs(q-[polynomial(query);2*query]),[],'all') < 1e-12 && ...
    max(abs(qs-[-2+6*query-12*query.^2;2*ones(size(query))]),[],'all') < 1e-11 && ...
    max(abs(qss-[6-24*query;zeros(size(query))]),[],'all') < 1e-10 && ...
    max(abs(qsss-[-24*ones(size(query));zeros(size(query))]),[],'all') < 1e-9, ...
    'The not-a-knot cubic failed to reproduce a known polynomial.');
[qd,qdd,jerk] = pathProgressChainRule(qs,qss,qsss, ...
    2*ones(size(query)),3*ones(size(query)),4*ones(size(query)));
assert(max(abs(qd-2*qs),[],'all') < 1e-12 && ...
    max(abs(qdd-(4*qss+3*qs)),[],'all') < 1e-12 && ...
    max(abs(jerk-(8*qsss+18*qss+4*qs)),[],'all') < 1e-12,'Chain-rule check failed.');
expectError(@() pathProgressEvaluateCubic(nodes,coeff,[0 1+eps]),'PathProgress:OutsideDomain');
expectError(@() pathProgressBuildCubic([0 0.4 0.4 1],ones(1,4)),'PathProgress:InvalidNodes');
end

function expectError(callback,id)
try
    callback();
catch problem
    assert(strcmp(problem.identifier,id),'Unexpected error: %s',problem.message);
    return;
end
error('Expected error %s.',id);
end

function [summary,trace] = checkDemo(data,robot,baseline)
positionTolerance = 1e-5;     % m, existing FK check
orientationTolerance = 1e-4;  % rad, existing FK check
stepTolerance = 0.10;         % rad, existing maximum actual step
endpointTolerance = 1e-4;     % rad, existing start and branch-matching check
planningTolerance = 1e-12;    % m/rad/s, planning roundoff
n = numel(data.tGlobal);
assert(isequal(data.qStart,baseline.qStart) && ...
    isequal(size(data.qPath),size(baseline.qPath),size(data.qDot),size(data.qDDot)) && ...
    isequal(size(data.TPath),size(baseline.TPath)) && ...
    isequal(data.segmentIndexPath,baseline.segmentIndexPath),'Baseline layouts or inputs changed.');
deltaTime = max(abs(data.tGlobal-baseline.tGlobal));
deltaPosition = max(vecnorm(data.pPath-baseline.pPath,2,1));
deltaRotation = 0;
assert(all(diff(data.tGlobal)>0) && ...
    max(abs(data.tcpSpeedPath-baseline.tcpSpeedPath)) <= planningTolerance && ...
    max(abs(data.arcPositionPath-baseline.arcPositionPath)) <= planningTolerance, ...
    'Planned speed, arc length or timestamp ordering changed.');

limits = robot.params.joints.positionLimits;
tcpFrame = char(robot.structure.frames.endEffector);
execPosition = zeros(1,n);
execRotation = zeros(1,n);
for i = 1:n
    target = data.TPath(:,:,i);
    actual = getTransform(robot.model,data.qPath(:,i).',tcpFrame);
    execPosition(i) = norm(actual(1:3,4)-target(1:3,4));
    execRotation(i) = norm(rotationVector(target(1:3,1:3).'*actual(1:3,1:3)));
    deltaRotation = max(deltaRotation, ...
        norm(rotationVector(baseline.TPath(1:3,1:3,i).'*target(1:3,1:3))));
end
assert(max([deltaTime,deltaPosition,deltaRotation]) <= planningTolerance, ...
    'Timed target poses or timestamps changed.');
step = max(abs(diff(data.qPath,1,2)),[],'all');
assert(all(isfinite([data.qPath data.qDot data.qDDot]),'all') && ...
    all(data.qPath >= limits(:,1) & data.qPath <= limits(:,2),'all') && ...
    step <= stepTolerance,'Execution joint limits, continuity or finiteness failed.');
assert(max(execPosition)<=positionTolerance && max(execRotation)<=orientationTolerance, ...
    'Execution FK exceeds the existing tolerance.');
assert(max(abs(data.qPath(:,1)-data.qStart))<=endpointTolerance && ...
    max(abs(data.qPath(:,[1 end])-baseline.qPath(:,[1 end])),[],'all')<=endpointTolerance, ...
    'Initial or final reference left the baseline IK branch.');

trace = struct('positionError',execPosition,'orientationError',execRotation, ...
    'spatialDistance',[],'spatialPosition',[],'spatialRotation',[], ...
    'tcpSpeed',zeros(1,n),'linearError',zeros(1,n),'angularError',zeros(1,n), ...
    'derivativeTime',[],'velocityDifference',[],'accelerationDifference',[], ...
    'knotProgress',[],'knotPositionJump',[],'knotSlopeJump',[],'knotCurvatureJump',[]);
distance = 0;
elapsed = 0;
linearTolerance = 0;
stopAcceleration = 0;
for k = 1:numel(data.segments)
    segment = data.segments(k);
    fit = data.fit(k);
    assert(isequaln(segment.timing,baseline.timing{k}),'Baseline timing arrays differ for segment %d.',k);
    assert(max(abs(fit.q(:,end)-baseline.qPath(:,fit.globalIndices(end)))) <= endpointTolerance, ...
        'Segment %d ended on a different IK configuration.',k);
    assert(all(fit.velocity(:,[1 end])==0,'all'),'Missing zero-velocity waypoint stop.');
    [~,slope] = pathProgressEvaluateCubic(fit.nodes,fit.coefficients,[0 1]);
    expectedAcceleration = slope.*segment.timing(4,[1 end]);
    stop = fit.acceleration(:,[1 end]);
    assert(max(abs(stop-expectedAcceleration),[],'all') <= 1e-12,'Endpoint acceleration is inconsistent.');
    stopAcceleration = max(stopAcceleration,max(abs(stop),[],'all'));
    if segment.profile.type == "S-Curve"
        assert(all(stop==0,'all'),'An S-curve endpoint did not stop at zero acceleration.');
    end
    if k > 1
        assert(max(abs(fit.q(:,1)-data.fit(k-1).q(:,end))) <= 1e-12 && ...
            all(data.qDot(:,fit.globalIndices(1)-1)==0), ...
            'A waypoint handoff changed configuration or missed its stop.');
    end
    assert(max(fit.spatialPositionError)<=positionTolerance && ...
        max(fit.spatialOrientationError)<=orientationTolerance && ...
        all(isfinite(fit.spatialQ) & fit.spatialQ>=limits(:,1) & fit.spatialQ<=limits(:,2),'all') && ...
        all(abs(diff(fit.spatialQ,1,2))<=stepTolerance,'all'), ...
        'Intermediate spatial FK, joint limits or continuity failed.');
    trace.spatialDistance = [trace.spatialDistance,distance+segment.length*fit.spatial];
    trace.spatialPosition = [trace.spatialPosition,fit.spatialPositionError];
    trace.spatialRotation = [trace.spatialRotation,fit.spatialOrientationError];
    distance = distance+segment.length;

    [jumps,knot] = checkKnots(fit);
    assert(all(max(jumps,[],2) <= [1e-11;1e-10;1e-8]),'A cubic knot is not C2.');
    trace.knotProgress = [trace.knotProgress,k-1+knot];
    trace.knotPositionJump = [trace.knotPositionJump,jumps(1,:)];
    trace.knotSlopeJump = [trace.knotSlopeJump,jumps(2,:)];
    trace.knotCurvatureJump = [trace.knotCurvatureJump,jumps(3,:)];
    [derivativeTime,dv,da] = numericalDerivatives(segment,fit);
    trace.derivativeTime = [trace.derivativeTime,derivativeTime+elapsed];
    elapsed = elapsed+segment.profile.T;
    trace.velocityDifference = [trace.velocityDifference,dv];
    trace.accelerationDifference = [trace.accelerationDifference,da];
    [twist,linearLimit] = checkTwist(segment,fit,robot,tcpFrame);
    trace.tcpSpeed(fit.globalIndices) = twist(1,:);
    trace.linearError(fit.globalIndices) = twist(2,:);
    trace.angularError(fit.globalIndices) = twist(3,:);
    linearTolerance = max(linearTolerance,linearLimit);
end
assert(~isempty(trace.derivativeTime) && max(trace.velocityDifference)<=2e-6 && ...
    max(trace.accelerationDifference)<=2e-4,'Finite-difference derivative check failed.');
summary = struct('Demo',data.demo,'Samples',n,'Nodes',sum(arrayfun(@(f) numel(f.nodes),data.fit)), ...
    'Refinements',sum([data.fit.refinementPasses]),'TimestampDelta_s',deltaTime, ...
    'RequestedPositionDelta_m',deltaPosition,'RequestedRotationDelta_rad',deltaRotation, ...
    'ExecPosition_m',max(execPosition),'ExecRotation_rad',max(execRotation), ...
    'SpatialPosition_m',max(trace.spatialPosition),'SpatialRotation_rad',max(trace.spatialRotation), ...
    'JointStep_rad',step,'NumericalVelocityError_rad_s',max(trace.velocityDifference), ...
    'NumericalAccelerationError_rad_s2',max(trace.accelerationDifference), ...
    'JacobianLinearError_m_s',max(trace.linearError),'JacobianLinearTolerance_m_s',linearTolerance, ...
    'JacobianAngularError_rad_s',max(trace.angularError), ...
    'KnotPositionJump_rad',max(trace.knotPositionJump),'KnotSlopeJump_rad',max(trace.knotSlopeJump), ...
    'KnotCurvatureJump_rad',max(trace.knotCurvatureJump), ...
    'StopAcceleration_rad_s2',stopAcceleration,'PeakVelocity_rad_s',max(data.rateLimits.peak(:,1)), ...
    'PeakAcceleration_rad_s2',max(data.rateLimits.peak(:,2)), ...
    'PeakFiniteJerk_rad_s3',max(data.rateLimits.peak(:,3)), ...
    'UndefinedJerkSamples',data.rateLimits.undefinedJerkSamples,'VelocityLimit',data.rateLimits.status(1), ...
    'AccelerationLimit',data.rateLimits.status(2),'JerkLimit',data.rateLimits.status(3),'Status',"PASS");
end

function [jumps,knot] = checkKnots(fit)
h = diff(fit.nodes);
c = fit.coefficients;
a = c(:,:,1); b = c(:,:,2); d2 = c(:,:,3); d3 = c(:,:,4);
leftQ = ((d3.*h+d2).*h+b).*h+a;
leftSlope = (3*d3.*h+2*d2).*h+b;
leftCurvature = 6*d3.*h+2*d2;
jumps = [max(abs(leftQ(:,1:end-1)-a(:,2:end)),[],1); ...
    max(abs(leftSlope(:,1:end-1)-b(:,2:end)),[],1); ...
    max(abs(leftCurvature(:,1:end-1)-2*d2(:,2:end)),[],1)];
knot = fit.nodes(2:end-1);
end

function [t,dv,da] = numericalDerivatives(segment,fit)
% Perturb time inside a single timing phase. The existing profiles are
% cubic (S-curve) or quadratic (trapezoid), so this local Taylor step is exact.
profile = segment.profile;
if profile.type == "S-Curve"
    switches = [0,cumsum(profile.segmentDurations(:).')];
else
    switches = [0,profile.tAccel,profile.tAccel+profile.tCruise,profile.T];
end
h = 1e-5; % s, balances cancellation and truncation for the derivative check
timing = segment.timing;
keep = all(abs(timing(1,:)-switches(:)) > h,1) & isfinite(timing(5,:));
t = timing(1,keep);
s = timing(2,keep);
sd = timing(3,keep);
sdd = timing(4,keep);
sddd = timing(5,keep);
plus = s+h*sd+h^2/2*sdd+h^3/6*sddd;
minus = s-h*sd+h^2/2*sdd-h^3/6*sddd;
[qPlus,slopePlus] = pathProgressEvaluateCubic(fit.nodes,fit.coefficients,plus);
[qMinus,slopeMinus] = pathProgressEvaluateCubic(fit.nodes,fit.coefficients,minus);
qdPlus = slopePlus.*(sd+h*sdd+h^2/2*sddd);
qdMinus = slopeMinus.*(sd-h*sdd+h^2/2*sddd);
dv = max(abs((qPlus-qMinus)/(2*h)-fit.velocity(:,keep)),[],1);
da = max(abs((qdPlus-qdMinus)/(2*h)-fit.acceleration(:,keep)),[],1);
end

function [trace,linearTolerance] = checkTwist(segment,fit,robot,tcpFrame)
% geometricJacobian returns [angular; linear], both in the robot base frame.
keep = (size(fit.q,2)-numel(fit.globalIndices)+1):size(fit.q,2);
slopes = diff(segment.arcGeometry,1,2)./diff(segment.arcLength);
% The requested polyline has a jump in tangent at each geometry corner.
% Permit the corner's full adjacent-chord speed jump plus a 0.1 mm/s
% smooth-segment allowance for comparing the C2 reconstruction to it.
linearTolerance = 1e-4+max(segment.length*segment.timing(3,:))* ...
    max(vecnorm(diff(slopes,1,2),2,1));
trace = zeros(3,numel(keep));
for i = 1:numel(keep)
    k = keep(i);
    s = segment.timing(2,k);
    sd = segment.timing(3,k);
    index = discretize(segment.length*s,segment.arcLength);
    expectedLinear = slopes(:,index)*segment.length*sd;
    expectedAngular = zeros(3,1);
    if sd ~= 0
        h = min([1e-5,s/2,(1-s)/2]);
        rotations = pathProgressPose(segment,[s-h,s+h]);
        expectedAngular = rotationVector(rotations(1:3,1:3,2)*rotations(1:3,1:3,1).')*sd/(2*h);
    end
    J = geometricJacobian(robot.model,fit.q(:,k).',tcpFrame);
    twist = J*fit.velocity(:,k);
    trace(:,i) = [norm(twist(4:6));norm(twist(4:6)-expectedLinear);norm(twist(1:3)-expectedAngular)];
end
assert(max(trace(2,:))<=linearTolerance && max(trace(3,:))<=2e-4, ...
    'Jacobian twist mismatch: linear %.3g / %.3g m/s; angular %.3g / 0.0002 rad/s.', ...
    max(trace(2,:)),linearTolerance,max(trace(3,:)));
end

function v = rotationVector(R)
skew = [R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
amount = norm(skew);
if amount < eps && trace(R) < 1
    axisAngle = rotm2axang(R);
    v = axisAngle(1:3).'*axisAngle(4);
elseif amount == 0
    v = zeros(3,1);
else
    v = skew*(atan2(amount,trace(R)-1)/amount);
end
end

function savePlots(data,trace,folder)
fig = figure('Position',[100 100 1350 950],'Color','w');
layout = tiledlayout(fig,3,3,'TileSpacing','compact','Padding','compact');
title(layout,"robot_ver2 path-progress verification | "+data.demo,'Interpreter','none');
jointPlot(layout,data.tGlobal,data.qPath,'Joint position','Position [rad]');
jointPlot(layout,data.tGlobal,data.qDot,'Joint velocity','Velocity [rad/s]');
jointPlot(layout,data.tGlobal,data.qDDot,'Joint acceleration','Acceleration [rad/s^2]');
accuracyPlot(layout,data.arcPositionPath,trace.positionError, ...
    trace.spatialDistance,trace.spatialPosition,1e-5,'TCP position error','Error [m]');
accuracyPlot(layout,data.arcPositionPath,trace.orientationError, ...
    trace.spatialDistance,trace.spatialRotation,1e-4,'TCP rotation error','Angle [rad]');
ax = nexttile(layout);
plot(ax,data.tGlobal,data.tcpSpeedPath,'-','LineWidth',1.6);
hold(ax,'on');
plot(ax,data.tGlobal,trace.tcpSpeed,'--','LineWidth',1.2);
label(ax,'TCP speed','Time [s]','Speed [m/s]');
legend(ax,{'Planned','Jacobian reconstruction'},'Location','best');
ax = nexttile(layout);
semilogy(ax,trace.derivativeTime,max(trace.velocityDifference,eps));
hold(ax,'on');
semilogy(ax,trace.derivativeTime,max(trace.accelerationDifference,eps));
label(ax,'Time derivative differences','Time [s]','Absolute difference');
legend(ax,{'Velocity [rad/s]','Acceleration [rad/s^2]'},'Location','best');
ax = nexttile(layout);
semilogy(ax,data.tGlobal,max(trace.linearError,eps));
hold(ax,'on');
semilogy(ax,data.tGlobal,max(trace.angularError,eps));
label(ax,'Jacobian twist differences','Time [s]','Vector error norm');
legend(ax,{'Linear [m/s]','Angular [rad/s]'},'Location','best');
ax = nexttile(layout);
semilogy(ax,trace.knotProgress,max(trace.knotPositionJump,eps),'.');
hold(ax,'on');
semilogy(ax,trace.knotProgress,max(trace.knotSlopeJump,eps),'.');
semilogy(ax,trace.knotProgress,max(trace.knotCurvatureJump,eps),'.');
label(ax,'Internal knot continuity','Segment offset + progress','Absolute jump');
legend(ax,{'Position','Spatial slope','Spatial curvature'},'Location','best');
exportgraphics(fig,fullfile(folder,"path_progress_"+data.demo+".png"),'Resolution',150);
close(fig);
end

function jointPlot(layout,time,values,heading,y)
ax = nexttile(layout);
plot(ax,time,values.');
label(ax,heading,'Time [s]',y);
legend(ax,{'J1','J2','J3','J4','J5','J6'},'Location','best','NumColumns',2);
end

function accuracyPlot(layout,execution,errorTimed,spatial,errorSpatial,tolerance,heading,y)
ax = nexttile(layout);
plot(ax,spatial,errorSpatial,'Color',[.25 .45 .65]);
hold(ax,'on');
plot(ax,execution,errorTimed,'.','Color',[.84 .28 .12]);
yline(ax,tolerance,'--','Tolerance');
ylim(ax,[0,1.14*tolerance]);
label(ax,heading,'Arc length [m]',y);
legend(ax,{'Spatial checks','Execution samples'},'Location','best');
end

function label(ax,heading,x,y)
grid(ax,'on');
title(ax,heading);
xlabel(ax,x);
ylabel(ax,y);
end

function restoreSession(oldPath,oldVisible,oldPause,oldFigures)
path(oldPath);
set(groot,'DefaultFigureVisible',oldVisible);
pause(oldPause);
newFigures = setdiff(findall(groot,'Type','figure'),oldFigures);
delete(newFigures);
end
