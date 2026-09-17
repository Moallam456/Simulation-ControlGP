function qSamples = sample_workspace_configurations(robot,options)

% SAMPLE_WORKSPACE_CONFIGURATIONS Random joint samples inside limits.
%
% The seed option makes workspace studies reproducible without permanently
% changing MATLAB's global random-number state.

if nargin < 2 || isempty(options)
    options.numSamples = 50000;
    options.seed = 1;
end

numSamples = options.numSamples;
limits = robot.params.joints.positionLimits;
dof = robot.structure.dof;

if size(limits,1) ~= dof || size(limits,2) ~= 2
    error('sample_workspace_configurations:InvalidLimits', ...
        'Expected a %d-by-2 joint-limit matrix.', dof);
end

if any(~isfinite(limits(:))) || any(limits(:,1) >= limits(:,2))
    error('sample_workspace_configurations:InvalidLimits', ...
        'Joint limits must be finite and ordered.');
end

if isfield(options,'seed') && ~isempty(options.seed)
    oldState = rng;
    cleanup = onCleanup(@() rng(oldState));
    rng(options.seed,'twister');
end

span = limits(:,2).' - limits(:,1).';
lower = limits(:,1).';

qSamples = lower + rand(numSamples,dof).*span;

end
