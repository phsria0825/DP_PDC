function routeQueue = initializeRouteQueue(mapData)
%INITIALIZEROUTEQUEUE Build a sorted queue of intersection descriptors.
%   routeQueue = initializeRouteQueue(mapData) converts the intersection
%   table provided in mapData into an ordered queue without using structs or
%   cell arrays. Each row of mapData (and of the resulting queue) must
%   contain:
%       [intersectionId, cumulativeDistance, segmentLength, grade, speedLimit]
%
%   The intersections are sorted by the cumulative distance so that the
%   first row corresponds to the closest upcoming intersection.

    if isempty(mapData)
        routeQueue = zeros(0, 5);
        return;
    end

    if size(mapData, 2) < 5
        error('initializeRouteQueue:InvalidMapData', ...
              'mapData must have at least five columns.');
    end

    [~, order] = sort(mapData(:, 2));
    routeQueue = mapData(order, 1:5);
end
