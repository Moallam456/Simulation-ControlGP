function analyzePathProgressFollowup()
%ANALYZEPATHPROGRESSFOLLOWUP Audit saved references and verify the S-curve variant.
root = fileparts(fileparts(mfilename('fullpath')));
trajectory = fileparts(root);
folder = fullfile(root,'results');
oldPath = path;
oldVisible = get(groot,'DefaultFigureVisible');
oldPause = pause('query');
oldFigures = findall(groot,'Type','figure');
restore = onCleanup(@() restoreSession(oldPath,oldVisible,oldPause,oldFigures));
addpath(fullfile(root,'tests'),fullfile(root,'helpers'),fullfile(trajectory,'Pipeline'), ...
    fullfile(fileparts(trajectory),'Kinematics'));
set(groot,'DefaultFigureVisible','off');
pause('off');

saved = load(fullfile(folder,'path_progress_after.mat'),'after','traces');
[robot,~,solveSample] = loadTrajectoryRobot();
twist = table();
for name = ["line","arc","fullcircle","multi"]
    [rows,linear,angular] = pathProgressDiagnoseTwist(saved.after.(name),robot,solveSample);
    previous = saved.traces.(name);
    assert(max(abs(linear-previous.linearError)) <= 1e-12, ...
        'Recomputed linear discrepancy differs for %s.',name);
    assert(max(abs(angular-previous.angularError)) <= 2e-9, ...
        'Angular discrepancy changed beyond finite-difference roundoff for %s.',name);
    twist = [twist;rows]; %#ok<AGROW>
end
fprintf('Reproduced the saved twist discrepancies for all four original demos.\n');

expectedPath = path;
[scurve,model] = runVariant(fullfile(root,'PathProgressDemo_multisegment_scurve.m'));
assert(strcmp(path,expectedPath),'The S-curve entry point leaked MATLAB path changes.');
[stops,summary] = verifyVariant(scurve,saved.after.multi);
[rows,linear,angular] = pathProgressDiagnoseTwist(scurve,model,solveSample);
assert(max(linear) <= 1e-4 && max(angular) <= 2e-4, ...
    'S-curve twist discrepancies exceed the existing straight-path tolerances.');
twist = [twist;rows];
writetable(twist,fullfile(folder,'path_progress_twist_diagnostics.csv'));
writetable(stops,fullfile(folder,'path_progress_multi_scurve_stops.csv'));
writetable(summary,fullfile(folder,'path_progress_multi_scurve_summary.csv'));
save(fullfile(folder,'path_progress_multi_scurve.mat'),'scurve');
disp(twist);
disp(summary);

[peaks,knots,convergence] = pathProgressDiagnoseCircle(saved.after.fullcircle,robot);
writetable(peaks,fullfile(folder,'path_progress_fullcircle_jerk_peaks.csv'));
writetable(knots,fullfile(folder,'path_progress_fullcircle_jerk_knots.csv'));
writetable(convergence,fullfile(folder,'path_progress_fullcircle_fk_convergence.csv'));
disp(peaks);
disp(convergence);
end

function [data,model] = runVariant(entry)
run(entry);
data = pathProgressData;
model = robot;
assert(getappdata(animationFigure,'PathProgressLastFrame') == numel(data.tGlobal), ...
    'The S-curve animation did not render its final frame.');
end

function [stops,summary] = verifyVariant(data,old)
assert(isequal(data.qStart,old.qStart) && numel(data.segments)==numel(old.segments) && ...
    all(diff(data.tGlobal)>0) && all(abs(diff(data.qPath,1,2))<=0.10,'all'), ...
    'The S-curve changed the start, segment count, or execution continuity.');
assert(all(isfinite([data.qPath,data.qDot,data.qDDot,data.qJerk]),'all'), ...
    'The S-curve produced a nonfinite reference.');
n = numel(data.segments);
samples = [1,arrayfun(@(fit) fit.globalIndices(end),data.fit)];
positions = data.pPath(:,samples);
for k = 1:n
    segment = data.segments(k);
    reference = old.segments(k);
    fit = data.fit(k);
    assert(isequal(segment.rawGeometry,reference.rawGeometry) && ...
        isequal(segment.arcGeometry,reference.arcGeometry) && ...
        isequal(segment.startRotation,reference.startRotation) && ...
        isequal(segment.endRotation,reference.endRotation), 'S-curve changed the waypoint path.');
    assert(segment.profile.type=="S-Curve" && segment.profile.dt==reference.profile.dt && ...
        isequal(segment.timing(2:4,[1 end]),[0 1;0 0;0 0]), ...
        'S-curve progress does not start and end at rest.');
    assert(all(fit.velocity(:,[1 end])==0,'all') && ...
        all(fit.acceleration(:,[1 end])==0,'all') && ...
        max(abs(fit.q(:,end)-old.fit(k).q(:,end)))<=1e-4, ...
        'S-curve joint stops or waypoint configurations differ.');
    if k>1
        assert(max(abs(fit.q(:,1)-data.fit(k-1).q(:,end)))<=1e-12, ...
            'The S-curve changed branch at a waypoint.');
    end
end
assert(all(data.qDot(:,samples)==0,'all') && all(data.qDDot(:,samples)==0,'all'), ...
    'A global S-curve waypoint is not at rest.');
stops = table(string(('A':char('A'+n)).'),samples(:),data.tGlobal(samples).', ...
    positions(1,:).',positions(2,:).',positions(3,:).', ...
    max(abs(data.qDot(:,samples)),[],1).',max(abs(data.qDDot(:,samples)),[],1).', ...
    'VariableNames',{'Waypoint','Sample','Time_s','X_m','Y_m','Z_m', ...
    'MaxAbsJointVelocity_rad_s','MaxAbsJointAcceleration_rad_s2'});
spatialPosition = max([data.fit.spatialPositionError]);
spatialRotation = max([data.fit.spatialOrientationError]);
assert(max([spatialPosition,data.positionError])<=1e-5 && ...
    max([spatialRotation,data.orientationError])<=1e-4, 'S-curve FK exceeded the existing limits.');
summary = table(numel(data.tGlobal),n,numel(samples),data.tGlobal(end),old.tGlobal(end), ...
    max(data.positionError),max(data.orientationError),spatialPosition,spatialRotation, ...
    max(abs(diff(data.qPath,1,2)),[],'all'),max(data.rateLimits.peak(:,1)), ...
    max(data.rateLimits.peak(:,2)),max(data.rateLimits.peak(:,3)), ...
    data.rateLimits.status(1),data.rateLimits.status(2),data.rateLimits.status(3), ...
    'VariableNames',{'Samples','Segments','Stops','Duration_s','TrapezoidDuration_s', ...
    'ExecPosition_m','ExecRotation_rad','SpatialPosition_m','SpatialRotation_rad', ...
    'JointStep_rad','PeakVelocity_rad_s','PeakAcceleration_rad_s2','PeakJerk_rad_s3', ...
    'VelocityLimit','AccelerationLimit','JerkLimit'});
end

function restoreSession(oldPath,oldVisible,oldPause,oldFigures)
path(oldPath);
set(groot,'DefaultFigureVisible',oldVisible);
pause(oldPause);
delete(setdiff(findall(groot,'Type','figure'),oldFigures));
end
