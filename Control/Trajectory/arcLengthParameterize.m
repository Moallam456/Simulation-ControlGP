function [resampledPath, sOriginal, sNew] = ...
    arcLengthParameterize(pathPoints, spacing)
%ARCLENGTHPARAMETERIZE Resample a Cartesian path using arc length.
%
% Inputs:
%   pathPoints - 3xN Cartesian path points
%   spacing    - desired distance between samples [m]
%
% Outputs:
%   resampledPath - 3xM arc-length-spaced path
%   sOriginal     - cumulative arc length of original path
%   sNew          - arc-length locations of new samples

    %% Validate inputs
    

    if size(pathPoints,1) ~= 3
        error('pathPoints must be a 3xN matrix.');
    end

    if spacing <= 0
        error('spacing must be greater than zero.');
    end

    %% Calculate distances between consecutive points

    segmentVectors = diff(pathPoints,1,2);

    segmentLengths = vecnorm(segmentVectors,2,1);

    %% Calculate cumulative arc length

    sOriginal = [0, cumsum(segmentLengths)];

    totalLength = sOriginal(end);

    %% Generate desired arc-length locations

    sNew = 0:spacing:totalLength;

    % Make sure final point is included
    if sNew(end) < totalLength
        sNew = [sNew, totalLength];
    end

    %% Interpolate Cartesian coordinates

    resampledPath = zeros(3,length(sNew));

    for k = 1:3

        resampledPath(k,:) = interp1( ...
            sOriginal, ...
            pathPoints(k,:), ...
            sNew, ...
            'linear');

    end

end
