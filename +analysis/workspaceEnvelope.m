function envelope = workspaceEnvelope(workspaceResult, numBins)
%WORKSPACEENVELOPE Estimate radial workspace boundaries versus height.
%
% envelope = analysis.workspaceEnvelope(workspaceResult,numBins)
%
% The workspace is represented using:
%
%       rho = sqrt(X^2 + Y^2)
%
% versus:
%
%       Z
%
% For each Z band, the minimum and maximum sampled rho are calculated.
%
% This is useful for identifying:
%   - inner unreachable regions
%   - outer reach boundary
%   - workspace shape versus height

if nargin < 2
    numBins = 60;
end


%% Data

points = workspaceResult.points;

z = points(:,3);

rho = workspaceResult.horizontalRadius;


%% Create Z bins

zEdges = linspace( ...
    min(z), ...
    max(z), ...
    numBins+1);

zCenters = ...
    (zEdges(1:end-1) + zEdges(2:end))/2;


%% Assign samples to bins

binIndex = discretize(z,zEdges);


%% Allocate

rhoMin = NaN(numBins,1);
rhoMax = NaN(numBins,1);
sampleCount = zeros(numBins,1);


%% Calculate envelope

for k = 1:numBins

    mask = binIndex == k;

    sampleCount(k) = sum(mask);

    if sampleCount(k) > 0

        rhoMin(k) = min(rho(mask));
        rhoMax(k) = max(rho(mask));

    end

end


%% Store

envelope.z = zCenters(:);

envelope.rhoMin = rhoMin;

envelope.rhoMax = rhoMax;

envelope.sampleCount = sampleCount;

end
