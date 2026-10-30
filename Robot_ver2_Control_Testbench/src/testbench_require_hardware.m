function missing = testbench_require_hardware(cfg,throwOnMissing)
% Hardware claims require specifications AND independent model validation.
if nargin<2, throwOnMissing=true; end
missing={};
for j=1:6
    h=cfg.hardware.joints(j); fields=fieldnames(h);
    for k=1:numel(fields)
        value=h.(fields{k});
        if isempty(value) || (isnumeric(value) && any(~isfinite(value(:)))) || ...
                (islogical(value) && strcmp(fields{k},'verified') && ~value)
            missing{end+1}=sprintf('J%d.%s',j,fields{k}); %#ok<AGROW>
        end
    end
end
if ~cfg.hardware.sensor.verified, missing{end+1}='sensor specification/mounting verification'; end
if ~cfg.hardware.actuatorModelValidated, missing{end+1}='validated physical actuator model'; end
if ~cfg.hardware.massAccountingVerified, missing{end+1}='actuator/sensor CAD mass accounting'; end
if ~cfg.hardware.stopBehaviorValidated, missing{end+1}='validated stop/brake behavior'; end
% This release always executes cfg.demo. Entering ratings or flipping flags
% cannot turn that synthetic implementation into a hardware actuator model.
missing{end+1}='implemented hardware actuator backend (not supplied in this demonstration release)';
if throwOnMissing && ~isempty(missing)
    error('testbench:HardwareUnconfigured','Hardware validation refused. Missing: %s',strjoin(missing,', '));
end
end
