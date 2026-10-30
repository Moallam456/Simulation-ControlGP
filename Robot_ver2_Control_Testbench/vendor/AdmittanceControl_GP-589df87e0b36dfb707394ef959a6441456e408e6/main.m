%% Admittance Control - Graduation Project
% MATLAB workspace initializer (run this first from the repository).
% Adds legacy studies, active experiments and Simulink models to the path.
% Path-only initializer: does not clear workspace, close plots or run experiments.

projectRoot = fileparts(mfilename('fullpath'));

folders = {
    fullfile(projectRoot, '1D', 'models')
    fullfile(projectRoot, '1D', 'analysis')
    fullfile(projectRoot, '1D', 'simulink')
    fullfile(projectRoot, '6D')
    fullfile(projectRoot, '6D', 'models')
    fullfile(projectRoot, '6D', 'analysis')
    fullfile(projectRoot, '6D', 'tests')
    fullfile(projectRoot, '6D', 'Simulink')
    fullfile(projectRoot, '6D', 'FT_Sensor')
    fullfile(projectRoot, '6D', 'FT_Sensor', 'simulink')
    fullfile(projectRoot, '6D', 'FT_Sensor', 'tests')
    fullfile(projectRoot, 'experiments')
};

for k = 1:numel(folders)
    assert(isfolder(folders{k}), ...
        'Missing expected project folder: %s', folders{k});
    addpath(folders{k});
end

fprintf('Admittance Control - MATLAB paths initialized.\n');
fprintf('Repository: %s\n', projectRoot);
fprintf('Main 6D design: design_6axis_admittance\n');
