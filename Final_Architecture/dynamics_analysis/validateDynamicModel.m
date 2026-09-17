function report = validateDynamicModel(robot)
% VALIDATEDYNAMICMODEL Check the loaded model and provenance before analysis.

report.pass = false;
report.errors = strings(0,1);
report.warnings = strings(0,1);
report.dataQuality = strings(0,1);
report.dataSource = strings(0,1);
report.bodyNames = strings(0,1);
report.tests = struct();

if ~isstruct(robot) || ~isfield(robot,'model') || ~isa(robot.model,'rigidBodyTree')
    report.errors(end+1) = "robot.model must be a rigidBodyTree.";
    return;
end
model = robot.model;
report.tests.rowDataFormat = strcmpi(model.DataFormat,'row');
if ~report.tests.rowDataFormat
    report.errors(end+1) = "rigidBodyTree DataFormat must be row.";
end
if ~isfield(robot,'structure') || ~isfield(robot.structure,'dof') || ...
        ~isfield(robot.structure,'jointNames')
    report.errors(end+1) = "Robot structure needs dof and jointNames.";
    return;
end
dof = robot.structure.dof;
nonFixed = sum(cellfun(@(b) ~strcmpi(b.Joint.Type,'fixed'),model.Bodies));
report.tests.dofMatches = nonFixed == dof && ...
    numel(robot.structure.jointNames) == dof;
if ~report.tests.dofMatches
    report.errors(end+1) = "Model DOF and structure joint count differ.";
end
if ~isfield(robot,'params') || ~isfield(robot.params,'joints') || ...
        ~isfield(robot.params.joints,'positionLimits') || ...
        ~isequal(size(robot.params.joints.positionLimits),[dof 2])
    report.errors(end+1) = "Position limits must have DOF-by-2 shape.";
    return;
end
limits = robot.params.joints.positionLimits;
report.tests.limitsValid = all(isfinite(limits(:))) && all(limits(:,1) < limits(:,2));
if ~report.tests.limitsValid
    report.errors(end+1) = "Joint limits must be finite and ordered.";
end
report.tests.jointOrderMatches = true;
if numel(model.Bodies) < dof
    report.tests.jointOrderMatches = false;
else
    for i = 1:dof
        if string(model.Bodies{i}.Joint.Name) ~= string(robot.structure.jointNames(i)) || ...
                strcmpi(model.Bodies{i}.Joint.Type,'fixed')
            report.tests.jointOrderMatches = false;
        end
    end
end
if ~report.tests.jointOrderMatches
    report.errors(end+1) = "Model joint ordering differs from robot.structure.";
end

report.tests.massPropertiesValid = true;
report.tests.massMappingValid = true;
for i = 1:numel(model.Bodies)
    body = model.Bodies{i};
    name = string(body.Name);
    report.bodyNames(end+1,1) = name;
    if i <= dof && isfield(robot.params,'links') && numel(robot.params.links) >= i
        source = robot.params.links(i);
        expectedMass = source.mass;
    elseif isfield(robot.structure,'frames') && ...
            isfield(robot.structure.frames,'endEffector') && ...
            name == string(robot.structure.frames.endEffector) && ...
            isfield(robot.params,'tool') && isfield(robot.params.tool,'mass')
        source = robot.params.tool;
        expectedMass = source.mass;
        if isfield(source,'segmentMass')
            expectedMass = expectedMass + source.segmentMass;
        end
    else
        source = struct();
        expectedMass = [];
    end
    if isfield(source,'dynamics') && isfield(source.dynamics,'status')
        status = string(source.dynamics.status);
    else
        status = "unknown";
    end
    report.dataQuality(end+1,1) = status;
    if isfield(source,'massPropertySource')
        report.dataSource(end+1,1) = string(source.massPropertySource);
    else
        report.dataSource(end+1,1) = "unspecified";
    end
    if status == "unknown"
        report.warnings(end+1) = name + " mass-property provenance is unknown.";
    elseif status == "estimated"
        report.warnings(end+1) = name + " uses estimated mass properties.";
    end
    if ~isempty(expectedMass) && (~isscalar(expectedMass) || ...
            ~isfinite(expectedMass) || ...
            abs(body.Mass-expectedMass) > 1e-9*max(1,abs(expectedMass)))
        report.tests.massMappingValid = false;
        report.errors(end+1) = name + " mass differs from robot parameters.";
    end
    if i <= dof && isfield(source,'centerOfMass') && ...
            isfield(source,'inertia') && numel(source.centerOfMass) == 3 && ...
            isequal(size(source.inertia),[3 3]) && ...
            all(isfinite(source.centerOfMass)) && all(isfinite(source.inertia(:)))
        expectedI = [source.inertia(1,1) source.inertia(2,2) ...
            source.inertia(3,3) source.inertia(2,3) ...
            source.inertia(1,3) source.inertia(1,2)];
        if norm(body.CenterOfMass(:)-source.centerOfMass(:),Inf) > 1e-10 || ...
                norm(body.Inertia(:)-expectedI(:),Inf) > 1e-10
            report.tests.massMappingValid = false;
            report.errors(end+1) = name + " COM/inertia differs from robot parameters.";
        end
    end
    v = body.Inertia;
    c = body.CenterOfMass;
    valid = isscalar(body.Mass) && isfinite(body.Mass) && body.Mass > 0 && ...
        numel(c) == 3 && all(isfinite(c)) && numel(v) == 6 && all(isfinite(v));
    if valid
        I = [v(1) v(6) v(5); v(6) v(2) v(4); v(5) v(4) v(3)];
        e = eig(I);
        valid = min(e) >= -1e-10*max(1,norm(I,2));
    end
    if ~valid
        report.tests.massPropertiesValid = false;
        report.errors(end+1) = name + " has invalid mass, COM, or inertia.";
    end
end
report.pass = isempty(report.errors);
report.preliminary = any(report.dataQuality == "estimated" | ...
    report.dataQuality == "unknown");
end
