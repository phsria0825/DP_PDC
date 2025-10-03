function scheduleWindows = fcn_stateless(spatBase, spatDurations, spatGreen, horizonTime)
%FCN_STATELESS Generate green windows using numeric arrays only.
%   scheduleWindows = fcn_stateless(spatBase, spatDurations, spatGreen,
%   horizonTime) returns a matrix where each row is
%       [intersectionId, windowStart, windowEnd]
%   describing a green window that occurs within the provided horizon.
%
%   spatBase      : Kx3 matrix [id, currentPhase, timeToPhaseEnd].
%   spatDurations : PxK matrix containing phase durations (seconds).
%   spatGreen     : PxK matrix with 0/1 flags (1 indicates green).
%   horizonTime   : scalar look-ahead window (seconds).

    if nargin < 4 || isempty(horizonTime)
        horizonTime = 180;
    end

    if isempty(spatBase)
        scheduleWindows = zeros(0, 3);
        return;
    end

    numIntersections = size(spatBase, 1);
    scheduleWindows = zeros(0, 3);

    for idx = 1:numIntersections
        intersectionId = spatBase(idx, 1);
        currentPhase = max(1, round(spatBase(idx, 2)));
        timeRemaining = spatBase(idx, 3);

        durations = spatDurations(:, idx);
        greenFlags = spatGreen(:, idx);
        validMask = durations > 0;
        durations = durations(validMask);
        greenFlags = greenFlags(validMask);

        if isempty(durations)
            scheduleWindows = appendWindow(scheduleWindows, intersectionId, 0, horizonTime);
            continue;
        end

        numPhases = numel(durations);
        currentPhase = max(1, min(numPhases, currentPhase));

        tCursor = 0;
        remaining = timeRemaining;
        if remaining <= 0
            remaining = durations(currentPhase);
        end

        while tCursor < horizonTime
            phaseEnd = tCursor + min(remaining, horizonTime - tCursor);
            if greenFlags(currentPhase) > 0.5
                scheduleWindows = appendWindow(scheduleWindows, intersectionId, tCursor, phaseEnd);
            end

            tCursor = tCursor + remaining;
            if tCursor >= horizonTime
                break;
            end

            currentPhase = mod(currentPhase, numPhases) + 1;
            remaining = durations(currentPhase);
            if remaining <= 0
                break;
            end
        end
    end
end

function scheduleWindows = appendWindow(scheduleWindows, intersectionId, startTime, endTime)
%APPENDWINDOW Append a row describing a green window.
    newRow = [intersectionId, startTime, endTime];
    scheduleWindows = [scheduleWindows; newRow]; %#ok<AGROW>
end
