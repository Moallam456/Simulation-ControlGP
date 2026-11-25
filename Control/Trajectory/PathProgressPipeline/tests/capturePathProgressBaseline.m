function baseline = capturePathProgressBaseline()
% Run the existing robot_ver2 scripts in a disposable folder under results.
% Use a fresh MATLAB session: the original scripts close figures.

root = fileparts(fileparts(mfilename('fullpath')));
trajectory = fileparts(root);
repo = fileparts(fileparts(trajectory));
pipeline = fullfile(trajectory,'Pipeline');
results = fullfile(root,'results');
if ~isfolder(results), mkdir(results); end
scratch = tempname(results);
mkdir(scratch);
names = ["SingleSegmentTest_line","SingleSegmentTest_arc", ...
    "SingleSegmentTest_fullcircle","trajectoryPipelineArcLengthADLS"];
labels = ["line","arc","fullcircle","multi"];
oldPath = path;
oldVisibility = get(groot,'DefaultFigureVisible');
oldPause = pause('query');
cleanup = onCleanup(@() finish(scratch,names,oldPath,oldVisibility,oldPause));
addpath(repo,trajectory,fullfile(trajectory,'TimeScaling'),pipeline);
addpath(scratch,'-begin');
set(groot,'DefaultFigureVisible','off');
pause('off');

for i = 1:numel(names)
    source = fileread(fullfile(pipeline,names(i)+".m"));
    if labels(i) == "multi"
        % Capture each original profile before the next iteration overwrites it.
        % The original timing helper also returns its jerk array.
        marker = "sDDotSegment = sDDotSegment(:).';";
        discarded = "        ~, ...";
        assert(count(source,marker)==1 && count(source,discarded)==1, ...
            'Baseline timing capture anchors changed.');
        source = replace(source,discarded,"        sDDDotSegment, ...");
        source = replace(source,marker,marker + newline + ...
            "    capturedTiming{segment} = [tSegment;sSegment;sDotSegment;sDDotSegment;sDDDotSegment(:).'];");
    end
    file = fopen(fullfile(scratch,names(i)+".m"),'w','n','UTF-8');
    assert(file >= 0,'Could not stage the baseline script.');
    fprintf(file,'%s',source);
    fclose(file);
    baseline.(labels(i)) = runOriginal(fullfile(scratch,names(i)+".m"));
end
lineExport = readtable(fullfile(scratch,'single_segment_line_robot_ver2_matlab.csv'));
lineExportColumns = lineExport.Properties.VariableNames;
save(fullfile(results,'robot_ver2_baseline.mat'),'baseline','lineExportColumns');
end

function data = runOriginal(entry)
% The called script clears its local workspace, including entry.
run(entry);
if ~exist('capturedTiming','var')
    capturedTiming = {[tSegment;sSegment;sDotSegment;sDDotSegment;sDDDotSegment]};
end
data = struct('qStart',qStart,'TStart',TStart,'TPath',TPath,'tGlobal',tGlobal, ...
    'pPath',pPath,'pAchieved',pAchieved,'arcPositionPath',arcPositionPath, ...
    'tcpSpeedPath',tcpSpeedPath,'segmentIndexPath',segmentIndexPath, ...
    'qPath',qPath,'qDot',qDot,'qDDot',qDDot,'positionError',positionError, ...
    'orientationError',orientationError,'timing',{capturedTiming});
end

function finish(scratch,names,oldPath,oldVisibility,oldPause)
path(oldPath);
set(groot,'DefaultFigureVisible',oldVisibility);
pause(oldPause);
files = [names+".m","single_segment_line_robot_ver2_matlab.csv"];
for name = files
    file = fullfile(scratch,name);
    if isfile(file), delete(file); end
end
rmdir(scratch);
end
