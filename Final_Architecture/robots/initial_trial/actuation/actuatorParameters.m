function actuation = actuatorParameters(structure)

% ACTUATORPARAMETERS Placeholder actuator interface for initial_trial.
%
% No trustworthy motor/gearbox data is available yet, so the returned
% fields are intentionally empty/NaN. Kinematic analyses must still work.

for i = 1:structure.dof
    actuation(i).jointName = structure.jointNames(i);
    actuation(i).motor = struct();
    actuation(i).gearbox = struct();
end

end
