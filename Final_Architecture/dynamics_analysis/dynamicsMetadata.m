function meta = dynamicsMetadata(robot,gravity,validation,analysisType)
% DYNAMICSMETADATA Record scope and physical-data quality with results.
meta.robotID = string(robot.id);
meta.gravity = gravity;
meta.analysisType = analysisType;
meta.dataQuality = table(validation.bodyNames,validation.dataQuality, ...
    validation.dataSource,'VariableNames',{'Body','Status','Source'});
meta.preliminary = validation.preliminary;
meta.label = "ideal rigid-body joint torque";
if validation.preliminary
    meta.label = "PRELIMINARY ideal rigid-body joint torque";
end
meta.assumptions = "No friction, transmission, motor rotor inertia, external wrench, or safety margin.";
if isfield(robot,'readiness')
    meta.readiness = robot.readiness;
    meta.warnings = robot.readiness.warnings;
    if isfield(robot.readiness,'massPropertiesPreliminary') && ...
            robot.readiness.massPropertiesPreliminary
        meta.preliminary = true;
        meta.label = "PRELIMINARY ideal rigid-body joint torque";
    end
else
    meta.readiness.status = "UNSPECIFIED";
    meta.warnings = strings(0,1);
end
end
