function metrics = compute_workspace_metrics(points,options)

% COMPUTE_WORKSPACE_METRICS Bounds, radius, density, and volume estimates.

if nargin < 2 || isempty(options)
    options.densityBins = 50;
    options.volumeBins = 35;
end

validatePoints(points);

metrics.bounds.x = [min(points(:,1)) max(points(:,1))];
metrics.bounds.y = [min(points(:,2)) max(points(:,2))];
metrics.bounds.z = [min(points(:,3)) max(points(:,3))];

radius = sqrt(sum(points.^2,2));

metrics.radius.min = min(radius);
metrics.radius.max = max(radius);
metrics.radius.mean = mean(radius);

metrics.density.xy = density2D(points(:,1),points(:,2),options.densityBins);
metrics.density.xz = density2D(points(:,1),points(:,3),options.densityBins);
metrics.density.yz = density2D(points(:,2),points(:,3),options.densityBins);

metrics.volume = estimateVolume(points,options.volumeBins);

end

function validatePoints(points)

if ~isnumeric(points) || size(points,2) ~= 3 || isempty(points)
    error('compute_workspace_metrics:InvalidPoints', ...
        'points must be a nonempty N-by-3 numeric matrix.');
end

if any(~isfinite(points(:)))
    error('compute_workspace_metrics:InvalidPoints', ...
        'points must contain only finite values.');
end

end

function density = density2D(a,b,numBins)

aEdges = makeEdges(a,numBins);
bEdges = makeEdges(b,numBins);

counts = histcounts2(a,b,aEdges,bEdges);

density.counts = counts;
density.edgesA = aEdges;
density.edgesB = bEdges;
density.numBins = numBins;

end

function volume = estimateVolume(points,numBins)

xEdges = makeEdges(points(:,1),numBins);
yEdges = makeEdges(points(:,2),numBins);
zEdges = makeEdges(points(:,3),numBins);

xIndex = discretize(points(:,1),xEdges);
yIndex = discretize(points(:,2),yEdges);
zIndex = discretize(points(:,3),zEdges);

valid = ~isnan(xIndex) & ~isnan(yIndex) & ~isnan(zIndex);

if ~any(valid)
    occupiedCount = 0;
else
    linearIndex = sub2ind( ...
        [numel(xEdges)-1 numel(yEdges)-1 numel(zEdges)-1], ...
        xIndex(valid), ...
        yIndex(valid), ...
        zIndex(valid));

    occupiedCount = numel(unique(linearIndex));
end

voxelSize = [
    mean(diff(xEdges))
    mean(diff(yEdges))
    mean(diff(zEdges))];

volume.estimate = occupiedCount * prod(voxelSize);
volume.occupiedVoxels = occupiedCount;
volume.numBins = numBins;
volume.edges.x = xEdges;
volume.edges.y = yEdges;
volume.edges.z = zEdges;

end

function edges = makeEdges(values,numBins)

lo = min(values);
hi = max(values);

if lo == hi
    pad = max(1,abs(lo))*1e-6;
    lo = lo - pad;
    hi = hi + pad;
end

edges = linspace(lo,hi,numBins + 1);
edges(end) = edges(end) + eps(edges(end));

end
