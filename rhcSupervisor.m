function [vSet, trajectorySpeeds, trajectoryTimes, diagVector] = rhcSupervisor(vehicleState, mapData, spatBase, spatDurations, spatGreen, speedGrid, options)
%RHCSUPERVISOR Rolling horizon supervisor implemented with numeric arrays.
%   [vSet, trajectorySpeeds, trajectoryTimes, diagVector] = rhcSupervisor(...)
%   orchestrates the rolling horizon strategy described in the README
%   without relying on structs or cell arrays. The inputs are:
%       vehicleState : [currentSpeed; travelledDistance]
%       mapData      : matrix with rows [id, distance, segmentLength, grade, speedLimit]
%       spatBase     : matrix [id, currentPhase, timeToPhaseEnd]
%       spatDurations: phase duration matrix (seconds)
%       spatGreen    : phase green availability matrix (0/1)
%       speedGrid    : optional candidate speed grid
%       options      : optional vector [ds, maxAccel, maxDecel, horizonTime, reinitFlag]
%
%   Outputs:
%       vSet            - target speed to apply in the next control interval
%       trajectorySpeeds- row vector describing the optimal speed profile
%       trajectoryTimes - arrival times associated with trajectorySpeeds
%       diagVector      - diagnostic vector [currentIdx, firstHIdx, horizonCount, bestCost, penaltyCount]

    persistent routeQueue currentIdx

    if isempty(routeQueue)
        routeQueue = zeros(0, 5);
        currentIdx = 1;
    end

    if nargin < 7
        options = [];
    end

    if isempty(vehicleState) || numel(vehicleState) < 2
        error('rhcSupervisor:InvalidState', ...
              'vehicleState must contain [currentSpeed; travelledDistance].');
    end

    reinitFlag = 0;
    optVector = [];
    if ~isempty(options)
        if numel(options) >= 5
            reinitFlag = options(5);
            optVector = options(1:4);
        else
            optVector = options;
        end
    end

    if isempty(routeQueue) || reinitFlag > 0.5
        routeQueue = initializeRouteQueue(mapData);
        currentIdx = 1;
    end

    if isempty(routeQueue)
        vSet = vehicleState(1);
        trajectorySpeeds = vehicleState(1);
        trajectoryTimes = 0;
        diagVector = [0, 0, 0, 0, 0];
        return;
    end

    [horizonIndices, currentIdx] = updateHorizon(routeQueue, vehicleState, currentIdx);

    if isempty(horizonIndices)
        vSet = vehicleState(1);
        trajectorySpeeds = zeros(1, 0);
        trajectoryTimes = zeros(1, 0);
        diagVector = [currentIdx, 0, 0, 0, 0];
        routeQueue = zeros(0, 5);
        currentIdx = 1;
        return;
    end

    [segments, segmentSteps, totalSteps, parameterVector, speedVector, spatBaseSel, spatDurSel, spatGreenSel] = ...
        prepareOptimizationBuffer(routeQueue, horizonIndices, vehicleState, spatBase, spatDurations, spatGreen, speedGrid, optVector);

    if totalSteps <= 0 || isempty(segments)
        vSet = vehicleState(1);
        trajectorySpeeds = zeros(1, 0);
        trajectoryTimes = zeros(1, 0);
        diagVector = [currentIdx, horizonIndices(1), numel(horizonIndices), 0, 0];
        return;
    end

    horizonTime = parameterVector(6);
    signalWindows = fcn_stateless(spatBaseSel, spatDurSel, spatGreenSel, horizonTime);

    [trajectorySpeeds, trajectoryTimes, dpInfo] = velocityOptimiz_DP(segments, segmentSteps, totalSteps, parameterVector, speedVector, signalWindows);

    if isempty(trajectorySpeeds)
        vSet = vehicleState(1);
    else
        if numel(trajectorySpeeds) >= 2
            vSet = trajectorySpeeds(2);
        else
            vSet = trajectorySpeeds(1);
        end
    end

    if isempty(dpInfo)
        dpInfo = [0, 0, 0];
    end

    firstHIdx = horizonIndices(1);
    diagVector = [currentIdx, firstHIdx, numel(horizonIndices), dpInfo(1), dpInfo(end)];
end
