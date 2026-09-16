function collision = collisionGeometry(params,structure,dh)

% COLLISIONGEOMETRY Placeholder collision model for initial_trial.
%
% Accurate collision geometry is not available yet. This function exists so
% the common robot interface has a stable place for future simplified
% boxes/capsules/meshes.

collision.available = false;
collision.geometry = struct([]);
collision.notes = params.notes.missingData;
collision.bodyNames = structure.bodyNames;
collision.homeFrames = dh.homeFrames;

end
