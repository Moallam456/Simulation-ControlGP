function [solver,status] = analyticalIKFunction(robot)
% ANALYTICALIKFUNCTION Cache a generated solver by exact robot kinematics.
persistent cache
if isempty(cache)
    cache = containers.Map('KeyType','char','ValueType','any');
end
values = cellfun(@(T) T(:),robot.chain.fixedTransforms,'UniformOutput',false);
signature = [vertcat(values{:});robot.structure.jointAxes(:); ...
    robot.params.joints.positionLimits(:);robot.params.tool.TFlangeTCP(:)];
key = [char(robot.id) '|' char(robot.structure.frames.base) '|' ...
    char(robot.structure.frames.endEffector) '|' sprintf('%.17g,',signature)];
if isKey(cache,key)
    entry = cache(key);
    solver = entry.solver;
    status = entry.status;
    return;
end
solver = [];
status = "UNSUPPORTED";
try
    generator = analyticalInverseKinematics(robot.model);
    generator.KinematicGroup = struct('BaseName', ...
        char(robot.structure.frames.base),'EndEffectorBodyName', ...
        char(robot.structure.frames.endEffector));
    if generator.IsValidGroupForIK
        folder = tempname;
        mkdir(folder);
        [~,uniqueName] = fileparts(folder);
        name = ['ik_' matlab.lang.makeValidName(uniqueName)];
        original = pwd;
        restore = onCleanup(@() cd(original)); %#ok<NASGU>
        cd(folder);
        solver = generateIKFunction(generator,name);
        addpath(folder,'-end');
        status = "AVAILABLE";
    end
catch ME
    status = "GENERATION_FAILED: " + string(ME.message);
end
cache(key) = struct('solver',solver,'status',status);
end
