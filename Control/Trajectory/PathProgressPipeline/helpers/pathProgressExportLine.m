function pathProgressExportLine(folder,data)
%PATHPROGRESSEXPORTLINE Preserve the legacy line CSV schema in isolated results.
timing = data.segments(1).timing;
n = numel(data.tGlobal);
values = [(1:n).',timing.',data.arcPositionPath.',data.tcpSpeedPath.', ...
    data.pPath.',data.quatPath.',data.qPath.',data.qDot.', ...
    data.positionError.',data.orientationError.',data.ikIterations.'];
columns = ["sample","time","s","sDot","sDDot","sDDDot","arcLength","tcpSpeed", ...
    "px","py","pz","qw","qx","qy","qz","q"+(1:6),"qd"+(1:6), ...
    "positionError","orientationError","ikIterations"];
results = fullfile(folder,'results');
if ~isfolder(results), mkdir(results); end
writetable(array2table(values,'VariableNames',columns),fullfile(results,'path_progress_line.csv'));
end
