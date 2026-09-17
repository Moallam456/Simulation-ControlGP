function workspace = reachable_workspace(robot,options)

% REACHABLE_WORKSPACE Monte-Carlo TCP workspace for any loaded robot.
%
% Preferred:
%   robot = loadRobot();
%   options.numSamples = 50000;
%   options.seed = 1;
%   workspace = reachable_workspace(robot,options);
%
% Convenience:
%   workspace = reachable_workspace();
%   workspace = reachable_workspace(robot);
%   workspace = reachable_workspace(robot,50000);

rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(rootDir,fullfile(rootDir,'kinematics'),fullfile(rootDir,'workspace_analysis'));

if nargin < 1 || isempty(robot)
    robot = loadRobot();
    options = struct();
elseif isnumeric(robot)
    options.numSamples = robot;
    robot = loadRobot();
elseif nargin < 2
    options = struct();
elseif isnumeric(options)
    options = struct('numSamples',options);
end

options = normalizeWorkspaceOptions(options,rootDir);

fprintf('Generating reachable workspace for %s...\n',robot.id);

qSamples = sample_workspace_configurations(robot,options);
points = zeros(options.numSamples,3);

for i = 1:options.numSamples
    T = forward_kinematics(robot,qSamples(i,:));
    points(i,:) = T(1:3,4).';
end

metrics = compute_workspace_metrics(points,options);
figures = plot_workspace(robot,points,metrics,options);

workspace.robotID = robot.id;
workspace.options = options;
workspace.qSamples = qSamples;
workspace.points = points;
workspace.bounds = metrics.bounds;
workspace.radius = metrics.radius;
workspace.volume = metrics.volume;
workspace.density = metrics.density;
workspace.figures = figures;
workspace.summary = createSummary(robot,metrics,options);

fprintf('Workspace generation complete.\n');
fprintf('Estimated maximum reach = %.3f m\n',workspace.radius.max);
fprintf('Estimated minimum reach = %.3f m\n',workspace.radius.min);
fprintf('Estimated occupied volume = %.3f m^3\n',workspace.volume.estimate);

end

function options = normalizeWorkspaceOptions(options,rootDir)

if ~isstruct(options)
    error('reachable_workspace:InvalidOptions', ...
        'options must be a structure or a numeric sample count.');
end

options = setDefault(options,'numSamples',50000);
options = setDefault(options,'seed',1);
options = setDefault(options,'showPlot',true);
options = setDefault(options,'saveFigure',false);
options = setDefault(options,'outputFolder', ...
    fullfile(rootDir,'workspace_analysis','outputs'));
options = setDefault(options,'densityBins',50);
options = setDefault(options,'volumeBins',35);

validatePositiveInteger(options.numSamples,'numSamples');
validatePositiveInteger(options.densityBins,'densityBins');
validatePositiveInteger(options.volumeBins,'volumeBins');

if ~(islogical(options.showPlot) || isnumeric(options.showPlot))
    error('reachable_workspace:InvalidOptions', ...
        'options.showPlot must be logical.');
end

if ~(islogical(options.saveFigure) || isnumeric(options.saveFigure))
    error('reachable_workspace:InvalidOptions', ...
        'options.saveFigure must be logical.');
end

options.showPlot = logical(options.showPlot);
options.saveFigure = logical(options.saveFigure);

if ~(isempty(options.seed) || ...
     (isnumeric(options.seed) && isscalar(options.seed) && isfinite(options.seed)))
    error('reachable_workspace:InvalidOptions', ...
        'options.seed must be empty or a finite numeric scalar.');
end

end

function options = setDefault(options,name,value)

if ~isfield(options,name) || isempty(options.(name))
    options.(name) = value;
end

end

function validatePositiveInteger(value,name)

if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || ...
   value < 1 || value ~= floor(value)
    error('reachable_workspace:InvalidOptions', ...
        'options.%s must be a positive integer.', name);
end

end

function summary = createSummary(robot,metrics,options)

summary = sprintf( ...
    ['Workspace for %s using %d samples: ', ...
     'X[%.3f, %.3f], Y[%.3f, %.3f], Z[%.3f, %.3f], ', ...
     'max reach %.3f m, occupied volume %.3f m^3.'], ...
    robot.id, ...
    options.numSamples, ...
    metrics.bounds.x(1),metrics.bounds.x(2), ...
    metrics.bounds.y(1),metrics.bounds.y(2), ...
    metrics.bounds.z(1),metrics.bounds.z(2), ...
    metrics.radius.max, ...
    metrics.volume.estimate);

end
