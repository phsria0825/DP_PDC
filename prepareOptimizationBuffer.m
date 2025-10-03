function [segments, segmentSteps, totalSteps, parameterVector, speedVector, spatBaseSel, spatDurSel, spatGreenSel] = ...
    prepareOptimizationBuffer(routeQueue, horizonIndices, vehicleState, spatBase, spatDurations, spatGreen, speedGrid, options)
%PREPAREOPTIMIZATIONBUFFER Assemble numeric inputs for the DP optimiser.
%   This helper collects all required arrays while avoiding structs and
%   cells. The output arguments are:
%       segments        - Matrix describing the selected intersections.
%       segmentSteps    - Row vector with discretisation steps per segment.
%       totalSteps      - Total number of spatial steps.
%       parameterVector - [ds, maxAccel, maxDecel, initialSpeed, travelledDistance, horizonTime].
%       speedVector     - Candidate speed grid.
%       spatBaseSel     - SPaT base information for the horizon intersections.
%       spatDurSel      - Phase duration matrix aligned with spatBaseSel.
%       spatGreenSel    - Phase green flags aligned with spatBaseSel.
%
%   routeQueue : Nx5 matrix from initializeRouteQueue.
%   horizonIndices : row vector specifying which intersections are within the horizon.
%   vehicleState : [currentSpeed; travelledDistance].
%   spatBase : Mx3 matrix [id, currentPhase, timeToPhaseEnd].
%   spatDurations : PxM matrix (duration per phase, columns align with spatBase rows).
%   spatGreen : PxM matrix containing 0/1 flags for green availability.
%   speedGrid : row vector of candidate speeds (optional).
%   options : row vector [ds, maxAccel, maxDecel, horizonTime] (optional).

    if isempty(horizonIndices)
        segments = zeros(0, 5);
        segmentSteps = zeros(1, 0);
        totalSteps = 0;
        parameterVector = zeros(1, 6);
        speedVector = zeros(1, 0);
        spatBaseSel = zeros(0, 3);
        spatDurSel = zeros(0, 0);
        spatGreenSel = zeros(0, 0);
        return;
    end

    defaults = [10, 1.2, 1.5, 180];
    if nargin < 8 || isempty(options)
        opt = defaults;
    else
        opt = defaults;
        opt(1:min(end, numel(options))) = options(1:min(end, numel(options)));
    end

    if nargin < 7 || isempty(speedGrid)
        speedVector = 0:2:30;
    else
        speedVector = speedGrid(:)';
    end

    segments = routeQueue(horizonIndices, 1:5);
    numSegments = size(segments, 1);

    ds = opt(1);
    segmentSteps = zeros(1, numSegments);
    totalSteps = 0;
    for idx = 1:numSegments
        lengthValue = segments(idx, 3);
        steps = max(1, round(lengthValue / ds));
        segmentSteps(idx) = steps;
        totalSteps = totalSteps + steps;
    end

    parameterVector = [ds, opt(2), opt(3), vehicleState(1), vehicleState(2), opt(4)];

    [spatBaseSel, spatDurSel, spatGreenSel] = filterSpatForHorizon(spatBase, spatDurations, spatGreen, segments(:, 1));
end

function [spatBaseSel, spatDurSel, spatGreenSel] = filterSpatForHorizon(spatBase, spatDurations, spatGreen, segmentIds)
%FILTERSPATFORHORIZON Filter SPaT tables for the selected intersections.
%   The function returns empty matrices when any of the inputs are empty.

    if isempty(spatBase) || isempty(spatDurations) || isempty(spatGreen)
        spatBaseSel = zeros(0, 3);
        spatDurSel = zeros(0, 0);
        spatGreenSel = zeros(0, 0);
        return;
    end

    numSegments = numel(segmentIds);
    phases = size(spatDurations, 1);

    spatBaseSel = zeros(numSegments, 3);
    spatDurSel = zeros(phases, numSegments);
    spatGreenSel = zeros(phases, numSegments);

    for idx = 1:numSegments
        intersectionId = segmentIds(idx);
        match = find(spatBase(:, 1) == intersectionId, 1);
        if isempty(match)
            spatBaseSel(idx, :) = [intersectionId, 1, 0];
            spatDurSel(:, idx) = zeros(phases, 1);
            spatGreenSel(:, idx) = zeros(phases, 1);
        else
            spatBaseSel(idx, :) = spatBase(match, 1:3);
            spatDurSel(:, idx) = spatDurations(:, match);
            spatGreenSel(:, idx) = spatGreen(:, match);
        end
    end
end
