function [trajectorySpeeds, trajectoryTimes, dpInfo] = velocityOptimiz_DP(segments, segmentSteps, totalSteps, parameterVector, speedVector, signalWindows)
%VELOCITYOPTIMIZ_DP Dynamic programming eco-driving optimiser without structs.
%   [trajectorySpeeds, trajectoryTimes, dpInfo] = velocityOptimiz_DP(...)
%   computes the minimum-fuel trajectory using only numeric arrays.
%
%   segments       : matrix where each row is [id, distance, length, grade, speedLimit].
%   segmentSteps   : row vector with discretisation steps per segment.
%   totalSteps     : total number of spatial steps across the horizon.
%   parameterVector: [ds, maxAccel, maxDecel, initialSpeed, travelledDistance, horizonTime].
%   speedVector    : candidate speed grid (row vector).
%   signalWindows  : matrix with rows [intersectionId, startTime, endTime].
%
%   Outputs:
%       trajectorySpeeds - optimal speed profile (row vector).
%       trajectoryTimes  - cumulative arrival times (row vector).
%       dpInfo           - [bestCost, totalSteps, penaltyCount].

    if totalSteps <= 0 || isempty(segments)
        trajectorySpeeds = zeros(1, 0);
        trajectoryTimes = zeros(1, 0);
        dpInfo = [inf, 0, 0];
        return;
    end

    ds = parameterVector(1);
    maxAccel = parameterVector(2);
    maxDecel = parameterVector(3);
    initialSpeed = parameterVector(4);

    if isempty(speedVector)
        speeds = 0:2:30;
    else
        speeds = speedVector(:)';
    end
    numSpeeds = numel(speeds);

    J = inf(numSpeeds, totalSteps + 1);
    T = inf(numSpeeds, totalSteps + 1);
    policy = zeros(numSpeeds, totalSteps, 'uint16');

    [~, startIdx] = min(abs(speeds - initialSpeed));
    J(startIdx, 1) = 0;
    T(startIdx, 1) = 0;

    segmentBoundary = cumsum(segmentSteps);
    grades = segments(:, 4)';
    limits = segments(:, 5)';
    stepGrades = repelem(grades, segmentSteps);
    stepLimits = repelem(limits, segmentSteps);

    windowIds = signalWindows(:, 1);
    windowStarts = signalWindows(:, 2);
    windowEnds = signalWindows(:, 3);

    for step = 1:totalSteps
        grade = stepGrades(step);
        limit = stepLimits(step);

        for vIdx = 1:numSpeeds
            if ~isfinite(J(vIdx, step))
                continue;
            end

            vCurrent = speeds(vIdx);
            tCurrent = T(vIdx, step);

            for vNextIdx = 1:numSpeeds
                vNext = speeds(vNextIdx);
                if vNext > limit + 1e-3
                    continue;
                end

                accel = computeAcceleration(vCurrent, vNext, ds);
                if accel > maxAccel + 1e-6 || accel < -maxDecel - 1e-6
                    continue;
                end

                [stageTime, feasible] = computeTraversalTime(vCurrent, vNext, ds);
                if ~feasible
                    continue;
                end

                tArrival = tCurrent + stageTime;
                fuelRate = fuel_prediction_model((vCurrent + vNext) / 2, accel, grade);
                stageCost = fuelRate * stageTime;

                penalty = computeSignalPenalty(step, tArrival, segmentBoundary, segments(:, 1), windowIds, windowStarts, windowEnds);
                totalCost = J(vIdx, step) + stageCost + penalty;

                if totalCost + 1e-9 < J(vNextIdx, step + 1)
                    J(vNextIdx, step + 1) = totalCost;
                    T(vNextIdx, step + 1) = tArrival;
                    policy(vNextIdx, step) = uint16(vIdx);
                end
            end
        end
    end

    [bestCost, terminalIdx] = min(J(:, end));
    if ~isfinite(bestCost)
        trajectorySpeeds = zeros(1, 0);
        trajectoryTimes = zeros(1, 0);
        dpInfo = [inf, totalSteps, numel(segmentBoundary)];
        return;
    end

    pathIdx = zeros(totalSteps + 1, 1);
    pathIdx(end) = terminalIdx;
    for step = totalSteps:-1:1
        prevIdx = policy(pathIdx(step + 1), step);
        if prevIdx == 0
            prevIdx = pathIdx(step + 1);
        end
        pathIdx(step) = prevIdx;
    end

    trajectorySpeeds = speeds(pathIdx)';
    trajectoryTimes = zeros(1, totalSteps + 1);
    for node = 1:(totalSteps + 1)
        trajectoryTimes(node) = T(pathIdx(node), node);
    end

    penaltyCount = evaluateSignalCompliance(trajectoryTimes, segmentBoundary, segments(:, 1), windowIds, windowStarts, windowEnds);
    dpInfo = [bestCost, totalSteps, penaltyCount];
end

function accel = computeAcceleration(vCurrent, vNext, ds)
%COMPUTEACCELERATION Estimate acceleration using spatial discretisation.
    if ds <= 0
        error('velocityOptimiz_DP:InvalidStep', 'ds must be positive.');
    end
    accel = (vNext^2 - vCurrent^2) / (2 * ds);
end

function [dt, feasible] = computeTraversalTime(vCurrent, vNext, ds)
%COMPUTETRAVERSALTIME Compute travel time between two speed states.
    meanSpeed = (vCurrent + vNext) / 2;
    if meanSpeed <= 0
        dt = inf;
        feasible = false;
        return;
    end
    dt = ds / meanSpeed;
    feasible = isfinite(dt) && dt > 0;
end

function penalty = computeSignalPenalty(step, arrivalTime, segmentBoundary, segmentIds, windowIds, windowStarts, windowEnds)
%COMPUTESIGNALPENALTY Apply a soft penalty if the vehicle arrives on red.
    penalty = 0;
    if isempty(segmentBoundary) || isempty(windowIds)
        return;
    end

    boundaryIndex = find(step == segmentBoundary, 1);
    if isempty(boundaryIndex)
        return;
    end

    intersectionId = segmentIds(boundaryIndex);
    mask = windowIds == intersectionId;

    if ~any(mask)
        penalty = 1e6;
        return;
    end

    starts = windowStarts(mask);
    ends = windowEnds(mask);
    isGreen = any(arrivalTime >= starts & arrivalTime <= ends);

    if ~isGreen
        timeToGreen = min(abs([starts(:); ends(:)] - arrivalTime));
        penalty = 1e5 + 1e3 * timeToGreen;
    end
end

function penaltyCount = evaluateSignalCompliance(trajectoryTimes, segmentBoundary, segmentIds, windowIds, windowStarts, windowEnds)
%EVALUATESIGNALCOMPLIANCE Count how many intersections are violated.
    penaltyCount = 0;
    if isempty(segmentBoundary)
        return;
    end

    numBoundaries = numel(segmentBoundary);
    for idx = 1:numBoundaries
        stepIndex = segmentBoundary(idx);
        if stepIndex + 1 > numel(trajectoryTimes)
            continue;
        end
        arrivalTime = trajectoryTimes(stepIndex + 1);
        intersectionId = segmentIds(idx);

        mask = windowIds == intersectionId;
        if ~any(mask)
            penaltyCount = penaltyCount + 1;
            continue;
        end

        starts = windowStarts(mask);
        ends = windowEnds(mask);
        isGreen = any(arrivalTime >= starts & arrivalTime <= ends);
        if ~isGreen
            penaltyCount = penaltyCount + 1;
        end
    end
end
