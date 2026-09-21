function poses = gravityCandidatePoses(params,structure)
% GRAVITYCANDIDATEPOSES Stationary screening poses for initial_trial.
% These are search seeds, not certified worst cases or collision-safe poses.
if structure.dof ~= 6
    error('initial_trial:InvalidDOF','Expected six joints.');
end

home = params.joints.homePosition(:).';
if numel(home) ~= 6
    error('initial_trial:InvalidHome','Expected six home joint values.');
end

angles = [
     0    0    0    0    0    0
     0   90    0    0    0    0
     0  -90    0    0    0    0
     0   90  -90    0    0    0
     0  -90   90    0    0    0
     0    0   90    0    0    0
     0    0  -90    0    0    0];
names = ["home";"shoulder_horizontal_positive"; ...
    "shoulder_horizontal_negative";"elbow_counterfold_positive"; ...
    "elbow_counterfold_negative";"elbow_positive";"elbow_negative"];
descriptions = ["Model home pose"; ...
    "Upper arm horizontal, positive shoulder direction"; ...
    "Upper arm horizontal, negative shoulder direction"; ...
    "Upper arm horizontal with opposite elbow rotation"; ...
    "Upper arm horizontal with opposite elbow rotation"; ...
    "Elbow rotated positive from home"; ...
    "Elbow rotated negative from home"];
angles(1,:) = rad2deg(home);

% Sample shoulder/elbow elevation and wrist orientations systematically.
% Base yaw is omitted because gravity is parallel to its vertical axis.
for shoulder = [-90 0 90]
    for elbow = [-90 0 90]
        for roll = [0 90 180]
            for pitch = [-90 0 90]
                candidate = [0 shoulder elbow roll pitch 0];
                angles(end+1,:) = candidate; %#ok<AGROW>
                names(end+1,1) = sprintf( ...
                    'screen_s%+d_e%+d_r%+d_p%+d', ...
                    shoulder,elbow,roll,pitch); %#ok<AGROW>
                descriptions(end+1,1) = ...
                    "Joint-limit-domain gravity screening pose"; %#ok<AGROW>
            end
        end
    end
end

q = deg2rad(angles);
limits = params.joints.positionLimits;
inLimits = all(q >= limits(:,1).'-1e-12 & ...
    q <= limits(:,2).'+1e-12,2);
q = q(inLimits,:);
names = names(inLimits);
descriptions = descriptions(inLimits);
[q,keep] = unique(q,'rows','stable');
poses.q = q;
poses.names = names(keep);
poses.descriptions = descriptions(keep);
poses.units = "rad";
end
