function [horizonIndices, currentIdx] = updateHorizon(routeQueue, vehicleState, currentIdx)
%UPDATEHORIZON Determine the active prediction horizon.
%   [horizonIndices, currentIdx] = updateHorizon(routeQueue, vehicleState,
%   currentIdx) selects the next two intersections that fall within the
%   rolling horizon. The function operates purely on numeric arrays.
%
%   routeQueue : Nx5 matrix returned by initializeRouteQueue.
%   vehicleState : [currentSpeed; travelledDistance].
%   currentIdx : current pointer into routeQueue (scalar index).
%
%   horizonIndices : row vector containing the indices of the intersections
%                    that form the optimisation horizon (up to two).

    horizonIndices = zeros(1, 0);

    if isempty(routeQueue)
        currentIdx = 1;
        return;
    end

    if isempty(vehicleState) || numel(vehicleState) < 2
        error('updateHorizon:InvalidState', ...
              'vehicleState must contain [currentSpeed; travelledDistance].');
    end

    travelledDistance = vehicleState(2);
    numIntersections = size(routeQueue, 1);

    if currentIdx < 1
        currentIdx = 1;
    end

    while currentIdx <= numIntersections && ...
          travelledDistance >= routeQueue(currentIdx, 2) - 1e-3
        currentIdx = currentIdx + 1;
    end

    if currentIdx > numIntersections
        horizonIndices = zeros(1, 0);
        return;
    end

    lastIdx = min(numIntersections, currentIdx + 1);
    horizonIndices = currentIdx:lastIdx;
end
