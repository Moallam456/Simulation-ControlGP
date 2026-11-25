function fit = pathProgressFitSegment(segment,tcpFK,solveSample,limits,startQ,inheritStart)
%PATHPROGRESSFITSEGMENT Fit and validate a segment before returning references.

positionTolerance = 1e-5;   % m
rotationTolerance = 1e-4;   % rad
stepTolerance = 0.10;       % rad, actual joint coordinates
maxRefinements = 6;
maxNodes = 5000;
spatialLength = segment.length;
if isfield(segment,'exactCircle')
    spatialLength = segment.exactLength;
end
nodeIntervals = max(3,ceil(spatialLength/0.0025));
if nodeIntervals+1 > maxNodes
    error('PathProgress:NodeBudget','The initial spatial grid exceeds %d nodes.',maxNodes);
end
nodes = (0:nodeIntervals)/nodeIntervals;
execution = segment.timing(2,:);

for refinement = 0:maxRefinements
    poses = pathProgressPose(segment,nodes);
    qNodes = zeros(size(limits,1),numel(nodes));
    seed = startQ;
    for k = 1:numel(nodes)
        if k == 1 && inheritStart
            q = startQ;
        else
            [q,info] = solveSample(poses(:,:,k),seed);
            if ~info.success
                error('PathProgress:SpatialIK','IK failed at s=%.9f: %s',nodes(k),info.message);
            end
        end
        qNodes(:,k) = q;
        seed = q;
    end
    coefficients = pathProgressBuildCubic(nodes,qNodes);

    widths = diff(nodes);
    interior = nodes(1:end-1) + (1:7).'/8*widths;
    % Include the original chord-corner progress values in both variants
    % and every actual execution query.
    spatial = unique([nodes,reshape(interior,1,[]),segment.arcLength/segment.length]);
    queries = unique([spatial,execution]);
    target = pathProgressPose(segment,queries);
    [q,~,~,~,interval] = pathProgressEvaluateCubic(nodes,coefficients,queries);
    [positionError,rotationError,achieved] = checkFK(tcpFK,q,target);
    within = all(isfinite(q) & q >= limits(:,1) & q <= limits(:,2),1);
    valid = positionError <= positionTolerance & rotationError <= rotationTolerance & within;
    continuous = [true,all(abs(diff(q,1,2)) <= stepTolerance,1)];
    refine = ~jointLimitHulls(coefficients,qNodes,widths,limits);
    refine(interval(~valid | ~continuous)) = true;
    % A query exactly at a knot is owned by the right interval. Split the
    % left neighbor too so both sides of a failing corner can improve.
    atKnot = queries == nodes(interval);
    left = interval(~valid & atKnot & interval > 1)-1;
    refine(left) = true;
    if ~any(refine)
        [~,timedIndex] = ismember(execution,queries);
        [~,spatialIndex] = ismember(spatial,queries);
        [fit.q,qs,qss,qsss] = pathProgressEvaluateCubic(nodes,coefficients,execution);
        [fit.velocity,fit.acceleration,fit.jerk] = pathProgressChainRule( ...
            qs,qss,qsss,segment.timing(3,:),segment.timing(4,:),segment.timing(5,:));
        definedJerk = isfinite(segment.timing(5,:));
        if any(~isfinite([fit.velocity,fit.acceleration,fit.jerk(:,definedJerk)]),'all')
            error('PathProgress:NonfiniteRates','The cubic or timing produced nonfinite reference rates.');
        end
        if any(abs(diff(fit.q,1,2)) > stepTolerance,'all')
            error('PathProgress:ExecutionStep','An execution sample exceeds the 0.10 rad joint step limit.');
        end
        fit.nodes = nodes;
        fit.coefficients = coefficients;
        fit.qNodes = qNodes;
        fit.spatial = spatial;
        fit.positionError = positionError(timedIndex);
        fit.orientationError = rotationError(timedIndex);
        fit.spatialPositionError = positionError(spatialIndex);
        fit.spatialOrientationError = rotationError(spatialIndex);
        fit.pAchieved = achieved(:,timedIndex);
        fit.spatialQ = q(:,spatialIndex);
        fit.refinementPasses = refinement;
        return;
    end
    if refinement == maxRefinements || numel(nodes)+nnz(refine) > maxNodes
        error('PathProgress:RefinementExhausted', ...
            ['Spatial checks failed at s=%.9f after %d refinements (%d nodes). ' ...
             'Maximum FK errors %.3g m and %.3g rad.'], ...
            nodes(find(refine,1)),refinement,numel(nodes),max(positionError),max(rotationError));
    end
    nodes = sort([nodes,nodes(find(refine))+0.5*widths(refine)]);
end
end

function valid = jointLimitHulls(coefficients,qNodes,widths,limits)
% Every cubic stays within its Bezier control-point hull for the interval.
leftSlope = coefficients(:,:,2);
rightSlope = leftSlope + 2*coefficients(:,:,3).*widths + 3*coefficients(:,:,4).*widths.^2;
control = cat(3,qNodes(:,1:end-1),qNodes(:,1:end-1)+leftSlope.*widths/3, ...
    qNodes(:,2:end)-rightSlope.*widths/3,qNodes(:,2:end));
valid = reshape(all(all(isfinite(control) & control >= limits(:,1) & ...
    control <= limits(:,2),1),3),1,[]);
end

function [positionError,rotationError,achieved] = checkFK(tcpFK,q,target)
n = size(q,2);
positionError = zeros(1,n);
rotationError = zeros(1,n);
achieved = zeros(3,n);
for i = 1:n
    actual = tcpFK(q(:,i));
    achieved(:,i) = actual(1:3,4);
    positionError(i) = norm(actual(1:3,4)-target(1:3,4,i));
    R = target(1:3,1:3,i).'*actual(1:3,1:3);
    v = [R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
    rotationError(i) = atan2(norm(v),trace(R)-1);
end
end
