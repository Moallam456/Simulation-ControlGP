function results = verifyRobotVer2TrajectoryIntegration()
%VERIFYROBOTVER2TRAJECTORYINTEGRATION Run the four fixed demonstrations.
% From the repository root, add Control/Trajectory/Testing to the MATLAB
% path and call verifyRobotVer2TrajectoryIntegration. Run in a fresh MATLAB
% session because the demonstrations close figures. Playback skips pauses.
%
% The UR5 baseline was captured with R2024b at 95a655ef0122a7a3a5fcdf5fab71a88010b19a5b
% from a clean main branch. It records the original poses, timing and ADLS
% outputs of the same entry scripts. Joint angles are model-dependent.
%
% FK tolerances match robot_ver2's solver defaults: 1e-5 m and 1e-4 rad.
% The first sample must return to within 1e-4 rad of the starting joints.
% At the fixed demo sampling, every unwrapped joint step must be <= 0.10 rad.

folder = fileparts(mfilename('fullpath'));
trajectory = fileparts(folder);
pipeline = fullfile(trajectory,'Pipeline');
repo = fileparts(fileparts(trajectory));
previousPath = path;
previousVisibility = get(groot,'DefaultFigureVisible');
previousPause = pause('query');
cleanup = onCleanup(@() restoreSession(previousPath,previousVisibility,previousPause));
addpath(repo,trajectory,fullfile(trajectory,'TimeScaling'),pipeline);
set(groot,'DefaultFigureVisible','off');
pause('off');

reference = load(fullfile(folder,'robot_ver2_trajectory_ur5_baseline.mat'),'baseline');
entries = ["SingleSegmentTest_line", "SingleSegmentTest_arc", ...
    "SingleSegmentTest_fullcircle", "trajectoryPipelineArcLengthADLS"];
labels = ["Line","Arc","Full circle","Multi-segment"];
positionTolerance = 1e-5;       % m
orientationTolerance = 1e-4;    % rad
jointStepTolerance = 0.10;       % rad, per sample
startingJointTolerance = 1e-4;  % rad
planningTolerance = 1e-12;       % m, rad and s, for baseline roundoff

n = numel(entries);
results = table(labels.',zeros(n,1),strings(n,1),nan(n,1),nan(n,1),nan(n,1), ...
    nan(n,1),nan(n,1),nan(n,1),nan(n,1),strings(n,1),nan(n,1), ...
    'VariableNames',{'Demo','Samples','Status','FailedSample', ...
    'MaxPositionError_m','MaxOrientationError_rad','MaxJointStep_rad', ...
    'StartJointError_rad','MaxOffsetDelta_m','MaxTimestampDelta_s', ...
    'OrientationComparison','MaxRelativeRotationDelta_rad'});
demos = cell(1,n);
for i = 1:n
    try
        data = runDemo(entries(i));
        baseline = reference.baseline.(entries(i));
        demos{i} = data;
        results.Samples(i) = numel(data.time);
        limits = data.limits;
        assert(isequal(size(data.q),[size(limits,1),numel(data.time)]) && ...
            isequal(size(data.pose),[4,4,numel(data.time)]) && ...
            isequal(size(data.achieved),[3,numel(data.time)]) && ...
            isequal(size(data.velocity),size(data.q)), 'Unexpected trajectory output dimensions.');
        assert(all(diff(data.time) > 0) && all(isfinite(data.velocity),'all'), ...
            'Timestamps or joint velocities are invalid.');
        startPositionError = norm(data.pose(1:3,4,1)-data.startPose(1:3,4));
        startOrientationError = relativeAngle(data.startPose(1:3,1:3).'*data.pose(1:3,1:3,1));
        startJointError = max(abs(data.q(:,1)-data.startQ));
        step = max(abs(diff(data.q,1,2)),[],1);
        withinLimits = all(isfinite(data.q) & ...
            data.q >= limits(:,1) & data.q <= limits(:,2),1);
        valid = data.accepted & withinLimits & data.iterations == 0 & ...
            data.positionError <= positionTolerance & ...
            data.orientationError <= orientationTolerance & ...
            [true,step <= jointStepTolerance];
        valid(1) = valid(1) && startPositionError <= positionTolerance && ...
            startOrientationError <= orientationTolerance && ...
            startJointError <= startingJointTolerance;
        results.MaxPositionError_m(i) = max(data.positionError);
        results.MaxOrientationError_rad(i) = max(data.orientationError);
        results.MaxJointStep_rad(i) = max(step);
        results.StartJointError_rad(i) = startJointError;
        failed = find(~valid,1);
        if ~isempty(failed)
            results.FailedSample(i) = failed;
            error('TrajectoryIntegration:InvalidSample','Validation failed at sample %d.',failed);
        end

        % Absolute TCP poses shift with the model-specific initial FK. Compare
        % retained planning inputs in their original relative frames.
        assert(isequal(size(data.time),size(baseline.tGlobal)), 'Baseline sample count changed.');
        offsets = data.position - data.position(:,1);
        baselineOffsets = baseline.pPath - baseline.pPath(:,1);
        results.MaxOffsetDelta_m(i) = max(vecnorm(offsets-baselineOffsets,2,1));
        results.MaxTimestampDelta_s(i) = max(abs(data.time-baseline.tGlobal));
        assert(results.MaxOffsetDelta_m(i) <= planningTolerance && ...
            results.MaxTimestampDelta_s(i) <= planningTolerance, ...
            'Retained Cartesian offsets or timestamps differ from the UR5 baseline.');

        % Line orientation is taught relative to the initial tool frame.
        % Arc/circle orientation follows a rotation in the base frame.
        relativeDifference = compareOrientations(data.pose,baseline.TPath,i==1);
        results.MaxRelativeRotationDelta_rad(i) = relativeDifference;
        if i == 3
            results.OrientationComparison(i) = "Adjusted sweep";
        else
            results.OrientationComparison(i) = "Retained";
            assert(relativeDifference <= planningTolerance, 'Relative orientation profile changed.');
        end
        if i == 1
            csv = readtable(fullfile(pipeline,'single_segment_line_robot_ver2_matlab.csv'));
            assert(height(csv) == numel(data.time) && all(csv.ikIterations == 0), ...
                'Line trajectory export is incomplete.');
        end
        results.Status(i) = "PASS";
    catch failure
        results.Status(i) = "FAIL";
        sample = regexp(failure.message,'failed at sample (\d+)','tokens','once');
        if ~isempty(sample)
            results.FailedSample(i) = str2double(sample{1});
        end
        fprintf(2,'%s: %s\n',labels(i),getReport(failure,'extended','hyperlinks','off'));
    end
end

[robot,tcpFK,solveSample] = loadTrajectoryRobot();
home = robot.params.joints.homePosition;
target = tcpFK(home);
target(1:3,4) = [10;0;10];
[q,info] = solveSample(target,home);
unreachablePassed = isempty(q) && ~info.success && ...
    info.method == "analytical_closed_form" && ...
    info.analyticalStatus == "NO_VALIDATED_BRANCH" && strlength(info.message) > 0;
fprintf('Unreachable target [10, 0, 10] m: %s\n',info.message);
results.UnreachableRejected = repmat(unreachablePassed,n,1);
disp(results);
writetable(results,fullfile(folder,'robot_ver2_trajectory_integration.csv'));
plotResults(demos,labels,positionTolerance,orientationTolerance,jointStepTolerance,folder);
assert(all(results.Status == "PASS") && unreachablePassed, ...
    'One or more robot_ver2 integration checks failed.');
end

function data = runDemo(name)
% Each original entry point clears this function workspace before running.
eval(char(name));
assert(isgraphics(animationFigure) && isgraphics(ax) && ...
    ~isempty(findobj(animationFigure,'Style','pushbutton')), ...
    'The animation and Replay Motion control were not created.');
data = struct('q',qPath,'startQ',qStart,'limits',robot.params.joints.positionLimits, ...
    'time',tGlobal,'pose',TPath,'startPose',TStart,'position',pPath, ...
    'achieved',pAchieved,'velocity',qDot,'accepted',ikConverged,'iterations',ikIterations, ...
    'positionError',positionError,'orientationError',orientationError);
end

function maximum = compareOrientations(actual,baseline,toolRelative)
maximum = 0;
actualStart = actual(1:3,1:3,1);
baselineStart = baseline(1:3,1:3,1);
for k = 1:size(actual,3)
    if toolRelative
        actualRelative = actualStart.'*actual(1:3,1:3,k);
        baselineRelative = baselineStart.'*baseline(1:3,1:3,k);
    else
        actualRelative = actual(1:3,1:3,k)*actualStart.';
        baselineRelative = baseline(1:3,1:3,k)*baselineStart.';
    end
    maximum = max(maximum,relativeAngle(baselineRelative.'*actualRelative));
end
end

function angle = relativeAngle(R)
% atan2 avoids the precision loss of acos(trace) for baseline roundoff.
skew = R - R.';
angle = atan2(norm([skew(3,2),skew(1,3),skew(2,1)]),trace(R)-1);
end

function plotResults(demos,labels,positionTolerance,orientationTolerance,stepTolerance,folder)
fig = figure('Position',[100 100 1150 760],'Color','w');
layout = tiledlayout(fig,2,2);
title(layout,'robot\_ver2 analytical IK trajectory verification');
pathAxes = nexttile(layout);
positionAxes = nexttile(layout);
orientationAxes = nexttile(layout);
stepAxes = nexttile(layout);
allAxes = [pathAxes,positionAxes,orientationAxes,stepAxes];
for ax = allAxes
    hold(ax,'on');
    grid(ax,'on');
end
for i = 1:numel(demos)
    data = demos{i};
    if isempty(data)
        continue;
    end
    p = data.position;
    plot3(pathAxes,p(1,:),p(2,:),p(3,:),'LineWidth',1.4,'DisplayName',labels(i));
    plot(positionAxes,data.time,data.positionError,'LineWidth',1.4,'DisplayName',labels(i));
    plot(orientationAxes,data.time,data.orientationError,'LineWidth',1.4,'DisplayName',labels(i));
    step = max(abs(diff(data.q,1,2)),[],1);
    plot(stepAxes,data.time(2:end),step,'LineWidth',1.4,'DisplayName',labels(i));
end
title(pathAxes,'Requested TCP paths');
xlabel(pathAxes,'X [m]');
ylabel(pathAxes,'Y [m]');
zlabel(pathAxes,'Z [m]');
axis(pathAxes,'equal');
view(pathAxes,3);
legend(pathAxes,'Location','best');
title(positionAxes,'FK position error');
ylabel(positionAxes,'Position error [m]');
yline(positionAxes,positionTolerance,'--','Tolerance');
title(orientationAxes,'FK relative rotation angle');
ylabel(orientationAxes,'Orientation error [rad]');
yline(orientationAxes,orientationTolerance,'--','Tolerance');
title(stepAxes,'Largest consecutive joint change');
ylabel(stepAxes,'Joint change [rad]');
yline(stepAxes,stepTolerance,'--','Tolerance');
for ax = allAxes(2:end)
    xlabel(ax,'Time [s]');
end
exportgraphics(fig,fullfile(folder,'robot_ver2_trajectory_integration.png'),'Resolution',150);
close(fig);
end

function restoreSession(previousPath,previousVisibility,previousPause)
path(previousPath);
set(groot,'DefaultFigureVisible',previousVisibility);
pause(previousPause);
end
