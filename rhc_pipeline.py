"""Rolling horizon eco-driving pipeline implemented in Python.

This module mirrors the MATLAB implementation shipped with the repository
so that the algorithm can be exercised and tested directly in Python.
The functions operate purely on NumPy arrays to stay close to the
MATLAB numeric workflow and avoid struct- or cell-based data structures.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


ArrayLike = np.ndarray


@dataclass
class RhcState:
    """Mutable state for the rolling-horizon supervisor."""

    route_queue: ArrayLike = field(default_factory=lambda: np.zeros((0, 5)))
    current_idx: int = 0


class RhcSupervisor:
    """Python translation of the MATLAB ``rhcSupervisor`` function."""

    def __init__(self) -> None:
        self.state = RhcState()

    def step(
        self,
        vehicle_state: ArrayLike,
        map_data: ArrayLike,
        spat_base: ArrayLike,
        spat_durations: ArrayLike,
        spat_green: ArrayLike,
        speed_grid: Optional[ArrayLike] = None,
        options: Optional[ArrayLike] = None,
    ) -> Tuple[float, ArrayLike, ArrayLike, ArrayLike]:
        """Execute a single rolling-horizon update.

        Parameters correspond to the MATLAB implementation. ``options`` may
        contain up to five elements ``[ds, maxAccel, maxDecel, horizonTime,
        reinitFlag]``.
        """

        vehicle_state = np.asarray(vehicle_state, dtype=float).reshape(-1)
        if vehicle_state.size < 2:
            raise ValueError("vehicle_state must contain [currentSpeed, distance]")

        reinit_flag = 0.0
        opt_vector = np.asarray([], dtype=float)
        if options is not None and len(options) > 0:
            options = np.asarray(options, dtype=float).reshape(-1)
            if options.size >= 5:
                reinit_flag = float(options[4])
                opt_vector = options[:4]
            else:
                opt_vector = options

        if self.state.route_queue.size == 0 or reinit_flag > 0.5:
            self.state.route_queue = initialize_route_queue(map_data)
            self.state.current_idx = 0

        if self.state.route_queue.size == 0:
            v_current = float(vehicle_state[0])
            return v_current, np.array([v_current]), np.array([0.0]), np.zeros(5)

        horizon_indices, self.state.current_idx = update_horizon(
            self.state.route_queue, vehicle_state, self.state.current_idx
        )

        if horizon_indices.size == 0:
            v_current = float(vehicle_state[0])
            self.state.route_queue = np.zeros((0, 5))
            self.state.current_idx = 0
            return v_current, np.zeros(0), np.zeros(0), np.array(
                [self.state.current_idx, 0, 0, 0.0, 0.0], dtype=float
            )

        (
            segments,
            segment_steps,
            total_steps,
            parameter_vector,
            speed_vector,
            spat_base_sel,
            spat_dur_sel,
            spat_green_sel,
        ) = prepare_optimization_buffer(
            self.state.route_queue,
            horizon_indices,
            vehicle_state,
            spat_base,
            spat_durations,
            spat_green,
            speed_grid,
            opt_vector,
        )

        if total_steps <= 0 or segments.size == 0:
            v_current = float(vehicle_state[0])
            return v_current, np.zeros(0), np.zeros(0), np.array(
                [self.state.current_idx, horizon_indices[0], horizon_indices.size, 0.0, 0.0],
                dtype=float,
            )

        horizon_time = float(parameter_vector[5])
        signal_windows = fcn_stateless(
            spat_base_sel, spat_dur_sel, spat_green_sel, horizon_time
        )

        (
            trajectory_speeds,
            trajectory_times,
            dp_info,
        ) = velocity_optimiz_dp(
            segments,
            segment_steps,
            int(total_steps),
            parameter_vector,
            speed_vector,
            signal_windows,
        )

        if trajectory_speeds.size == 0:
            v_set = float(vehicle_state[0])
        elif trajectory_speeds.size >= 2:
            v_set = float(trajectory_speeds[1])
        else:
            v_set = float(trajectory_speeds[0])

        if dp_info.size == 0:
            dp_info = np.array([0.0, 0.0, 0.0])

        diag_vector = np.array(
            [
                float(self.state.current_idx),
                float(horizon_indices[0]),
                float(horizon_indices.size),
                float(dp_info[0]),
                float(dp_info[-1]),
            ]
        )

        return v_set, trajectory_speeds, trajectory_times, diag_vector


def initialize_route_queue(map_data: ArrayLike) -> ArrayLike:
    """Match the MATLAB ``initializeRouteQueue`` helper."""

    map_data = np.asarray(map_data, dtype=float)
    if map_data.size == 0:
        return np.zeros((0, 5))
    if map_data.shape[1] < 5:
        raise ValueError("map_data must have at least five columns")

    order = np.argsort(map_data[:, 1])
    return map_data[order, :5]


def update_horizon(
    route_queue: ArrayLike, vehicle_state: ArrayLike, current_idx: int
) -> Tuple[ArrayLike, int]:
    """Replicate the MATLAB horizon selection routine."""

    route_queue = np.asarray(route_queue, dtype=float)
    vehicle_state = np.asarray(vehicle_state, dtype=float).reshape(-1)

    if route_queue.size == 0:
        return np.zeros(0, dtype=int), 0
    if vehicle_state.size < 2:
        raise ValueError("vehicle_state must contain [currentSpeed, distance]")

    travelled_distance = float(vehicle_state[1])
    num_intersections = route_queue.shape[0]

    idx = max(0, int(current_idx))
    while idx < num_intersections and travelled_distance >= route_queue[idx, 1] - 1e-3:
        idx += 1

    if idx >= num_intersections:
        return np.zeros(0, dtype=int), idx

    last_idx = min(num_intersections - 1, idx + 1)
    horizon_indices = np.arange(idx, last_idx + 1, dtype=int)
    return horizon_indices, idx


def prepare_optimization_buffer(
    route_queue: ArrayLike,
    horizon_indices: ArrayLike,
    vehicle_state: ArrayLike,
    spat_base: ArrayLike,
    spat_durations: ArrayLike,
    spat_green: ArrayLike,
    speed_grid: Optional[ArrayLike] = None,
    options: Optional[ArrayLike] = None,
) -> Tuple[ArrayLike, ArrayLike, int, ArrayLike, ArrayLike, ArrayLike, ArrayLike, ArrayLike]:
    """Assemble DP inputs while staying close to the MATLAB variant."""

    horizon_indices = np.asarray(horizon_indices, dtype=int)
    if horizon_indices.size == 0:
        return (
            np.zeros((0, 5)),
            np.zeros(0),
            0,
            np.zeros(6),
            np.zeros(0),
            np.zeros((0, 3)),
            np.zeros((0, 0)),
            np.zeros((0, 0)),
        )

    defaults = np.array([10.0, 1.2, 1.5, 180.0])
    opt = defaults.copy()
    if options is not None and len(options) > 0:
        options = np.asarray(options, dtype=float).reshape(-1)
        count = min(opt.size, options.size)
        opt[:count] = options[:count]

    if speed_grid is None or len(speed_grid) == 0:
        speed_vector = np.arange(0.0, 30.0 + 1e-9, 2.0)
    else:
        speed_vector = np.asarray(speed_grid, dtype=float).reshape(-1)

    segments = np.asarray(route_queue, dtype=float)[horizon_indices, :5]
    num_segments = segments.shape[0]
    ds = float(opt[0])

    segment_steps = np.zeros(num_segments, dtype=int)
    total_steps = 0
    for idx in range(num_segments):
        length_value = float(segments[idx, 2])
        steps = max(1, int(round(length_value / ds)))
        segment_steps[idx] = steps
        total_steps += steps

    vehicle_state = np.asarray(vehicle_state, dtype=float).reshape(-1)
    parameter_vector = np.array(
        [
            ds,
            float(opt[1]),
            float(opt[2]),
            float(vehicle_state[0]),
            float(vehicle_state[1]),
            float(opt[3]),
        ]
    )

    (
        spat_base_sel,
        spat_dur_sel,
        spat_green_sel,
    ) = filter_spat_for_horizon(spat_base, spat_durations, spat_green, segments[:, 0])

    return (
        segments,
        segment_steps,
        int(total_steps),
        parameter_vector,
        speed_vector,
        spat_base_sel,
        spat_dur_sel,
        spat_green_sel,
    )


def filter_spat_for_horizon(
    spat_base: ArrayLike,
    spat_durations: ArrayLike,
    spat_green: ArrayLike,
    segment_ids: ArrayLike,
) -> Tuple[ArrayLike, ArrayLike, ArrayLike]:
    """Helper that mirrors the MATLAB SPaT filtering routine."""

    if (
        spat_base is None
        or spat_durations is None
        or spat_green is None
        or np.asarray(spat_base).size == 0
        or np.asarray(spat_durations).size == 0
        or np.asarray(spat_green).size == 0
    ):
        return np.zeros((0, 3)), np.zeros((0, 0)), np.zeros((0, 0))

    spat_base = np.asarray(spat_base, dtype=float)
    spat_durations = np.asarray(spat_durations, dtype=float)
    spat_green = np.asarray(spat_green, dtype=float)
    segment_ids = np.asarray(segment_ids, dtype=float).reshape(-1)

    num_segments = segment_ids.size
    phases = spat_durations.shape[0]

    spat_base_sel = np.zeros((num_segments, 3))
    spat_dur_sel = np.zeros((phases, num_segments))
    spat_green_sel = np.zeros((phases, num_segments))

    for idx, intersection_id in enumerate(segment_ids):
        matches = np.where(np.isclose(spat_base[:, 0], intersection_id))[0]
        if matches.size == 0:
            spat_base_sel[idx, :] = np.array([intersection_id, 1.0, 0.0])
            continue

        match = matches[0]
        spat_base_sel[idx, :] = spat_base[match, :3]
        spat_dur_sel[:, idx] = spat_durations[:, match]
        spat_green_sel[:, idx] = spat_green[:, match]

    return spat_base_sel, spat_dur_sel, spat_green_sel


def fcn_stateless(
    spat_base: ArrayLike,
    spat_durations: ArrayLike,
    spat_green: ArrayLike,
    horizon_time: float,
) -> ArrayLike:
    """Generate green windows for each intersection within the horizon."""

    if horizon_time is None or horizon_time <= 0:
        horizon_time = 180.0

    if spat_base is None or np.asarray(spat_base).size == 0:
        return np.zeros((0, 3))

    spat_base = np.asarray(spat_base, dtype=float)
    spat_durations = np.asarray(spat_durations, dtype=float)
    spat_green = np.asarray(spat_green, dtype=float)

    windows: list[list[float]] = []
    for idx in range(spat_base.shape[0]):
        intersection_id = float(spat_base[idx, 0])
        current_phase = int(round(spat_base[idx, 1]))
        time_remaining = float(spat_base[idx, 3 - 1])

        durations = spat_durations[:, idx]
        green_flags = spat_green[:, idx]
        valid_mask = durations > 0
        durations = durations[valid_mask]
        green_flags = green_flags[valid_mask]

        if durations.size == 0:
            windows.append([intersection_id, 0.0, horizon_time])
            continue

        num_phases = durations.size
        current_phase = max(1, min(num_phases, current_phase))

        t_cursor = 0.0
        remaining = time_remaining if time_remaining > 0 else float(durations[current_phase - 1])

        while t_cursor < horizon_time and remaining > 0:
            phase_end = min(horizon_time, t_cursor + remaining)
            if green_flags[current_phase - 1] > 0.5:
                windows.append([intersection_id, t_cursor, phase_end])

            t_cursor += remaining
            if t_cursor >= horizon_time:
                break

            current_phase = (current_phase % num_phases) + 1
            remaining = float(durations[current_phase - 1])

    if not windows:
        return np.zeros((0, 3))

    return np.asarray(windows, dtype=float)


def velocity_optimiz_dp(
    segments: ArrayLike,
    segment_steps: ArrayLike,
    total_steps: int,
    parameter_vector: ArrayLike,
    speed_vector: ArrayLike,
    signal_windows: ArrayLike,
) -> Tuple[ArrayLike, ArrayLike, ArrayLike]:
    """Dynamic-programming eco-driving optimiser."""

    if total_steps <= 0 or segments.size == 0:
        return np.zeros(0), np.zeros(0), np.array([np.inf, 0.0, 0.0])

    segments = np.asarray(segments, dtype=float)
    segment_steps = np.asarray(segment_steps, dtype=int)
    parameter_vector = np.asarray(parameter_vector, dtype=float)
    speed_vector = np.asarray(speed_vector, dtype=float).reshape(-1)
    signal_windows = np.asarray(signal_windows, dtype=float)

    ds = float(parameter_vector[0])
    max_accel = float(parameter_vector[1])
    max_decel = float(parameter_vector[2])
    initial_speed = float(parameter_vector[3])

    if speed_vector.size == 0:
        speeds = np.arange(0.0, 30.0 + 1e-9, 2.0)
    else:
        speeds = speed_vector
    num_speeds = speeds.size

    J = np.full((num_speeds, total_steps + 1), np.inf)
    T = np.full((num_speeds, total_steps + 1), np.inf)
    policy = np.zeros((num_speeds, total_steps), dtype=np.int32)

    start_idx = int(np.argmin(np.abs(speeds - initial_speed)))
    J[start_idx, 0] = 0.0
    T[start_idx, 0] = 0.0

    segment_boundary = np.cumsum(segment_steps)
    grades = segments[:, 3]
    limits = segments[:, 4]
    step_grades = np.repeat(grades, segment_steps)
    step_limits = np.repeat(limits, segment_steps)

    if signal_windows.size == 0:
        window_ids = np.zeros(0)
        window_starts = np.zeros(0)
        window_ends = np.zeros(0)
    else:
        window_ids = signal_windows[:, 0]
        window_starts = signal_windows[:, 1]
        window_ends = signal_windows[:, 2]

    for step in range(total_steps):
        grade = step_grades[step]
        limit = step_limits[step]

        for v_idx in range(num_speeds):
            if not np.isfinite(J[v_idx, step]):
                continue

            v_current = speeds[v_idx]
            t_current = T[v_idx, step]

            for v_next_idx in range(num_speeds):
                v_next = speeds[v_next_idx]
                if v_next > limit + 1e-3:
                    continue

                accel = compute_acceleration(v_current, v_next, ds)
                if accel > max_accel + 1e-6 or accel < -max_decel - 1e-6:
                    continue

                stage_time, feasible = compute_traversal_time(v_current, v_next, ds)
                if not feasible:
                    continue

                t_arrival = t_current + stage_time
                fuel_rate = fuel_prediction_model((v_current + v_next) / 2.0, accel, grade)
                stage_cost = fuel_rate * stage_time

                penalty = compute_signal_penalty(
                    step,
                    t_arrival,
                    segment_boundary,
                    segments[:, 0],
                    window_ids,
                    window_starts,
                    window_ends,
                )
                total_cost = J[v_idx, step] + stage_cost + penalty

                if total_cost + 1e-9 < J[v_next_idx, step + 1]:
                    J[v_next_idx, step + 1] = total_cost
                    T[v_next_idx, step + 1] = t_arrival
                    policy[v_next_idx, step] = v_idx

    terminal_idx = int(np.argmin(J[:, -1]))
    best_cost = J[terminal_idx, -1]
    if not np.isfinite(best_cost):
        return np.zeros(0), np.zeros(0), np.array([np.inf, float(total_steps), float(segment_boundary.size)])

    path_idx = np.zeros(total_steps + 1, dtype=int)
    path_idx[-1] = terminal_idx
    for step in range(total_steps - 1, -1, -1):
        prev_idx = policy[path_idx[step + 1], step]
        if prev_idx <= 0 and step != 0:
            prev_idx = path_idx[step + 1]
        path_idx[step] = prev_idx

    trajectory_speeds = speeds[path_idx]
    trajectory_times = np.array([T[path_idx[node], node] for node in range(total_steps + 1)])

    penalty_count = evaluate_signal_compliance(
        trajectory_times,
        segment_boundary,
        segments[:, 0],
        window_ids,
        window_starts,
        window_ends,
    )
    dp_info = np.array([best_cost, float(total_steps), float(penalty_count)])

    return trajectory_speeds, trajectory_times, dp_info


def compute_acceleration(v_current: float, v_next: float, ds: float) -> float:
    if ds <= 0:
        raise ValueError("ds must be positive")
    return (v_next**2 - v_current**2) / (2.0 * ds)


def compute_traversal_time(v_current: float, v_next: float, ds: float) -> Tuple[float, bool]:
    mean_speed = (v_current + v_next) / 2.0
    if mean_speed <= 0:
        return float("inf"), False
    dt = ds / mean_speed
    return float(dt), np.isfinite(dt) and dt > 0


def compute_signal_penalty(
    step: int,
    arrival_time: float,
    segment_boundary: ArrayLike,
    segment_ids: ArrayLike,
    window_ids: ArrayLike,
    window_starts: ArrayLike,
    window_ends: ArrayLike,
) -> float:
    if segment_boundary.size == 0 or window_ids.size == 0:
        return 0.0

    matches = np.where(segment_boundary == (step + 1))[0]
    if matches.size == 0:
        return 0.0

    intersection_id = segment_ids[matches[0]]
    mask = np.isclose(window_ids, intersection_id)
    if not np.any(mask):
        return 1e6

    starts = window_starts[mask]
    ends = window_ends[mask]
    is_green = np.any((arrival_time >= starts) & (arrival_time <= ends))
    if is_green:
        return 0.0

    time_to_green = np.min(np.abs(np.concatenate((starts, ends)) - arrival_time))
    return 1e5 + 1e3 * float(time_to_green)


def evaluate_signal_compliance(
    trajectory_times: ArrayLike,
    segment_boundary: ArrayLike,
    segment_ids: ArrayLike,
    window_ids: ArrayLike,
    window_starts: ArrayLike,
    window_ends: ArrayLike,
) -> int:
    if segment_boundary.size == 0:
        return 0

    penalty_count = 0
    for idx, boundary in enumerate(segment_boundary):
        step_index = int(boundary)
        if step_index >= trajectory_times.size:
            continue

        arrival_time = trajectory_times[step_index]
        intersection_id = segment_ids[idx]

        mask = np.isclose(window_ids, intersection_id)
        if not np.any(mask):
            penalty_count += 1
            continue

        starts = window_starts[mask]
        ends = window_ends[mask]
        is_green = np.any((arrival_time >= starts) & (arrival_time <= ends))
        if not is_green:
            penalty_count += 1

    return penalty_count


def fuel_prediction_model(v: float, a: float, theta: float) -> float:
    """Polynomial fuel consumption surrogate used by the DP solver."""

    rho = 1.2256
    cd = 0.615
    ch = 1 - 0.085 * 0
    af = 10.2
    cr = 1.25
    c1 = 0.0328
    c2 = 4.575
    mass = 23000
    eta_d = 0.85

    r_drag = (rho / 25.92) * cd * ch * af * (v**2)
    r_roll = 9.81 * (cr / 1000.0) * (c1 * v + c2)
    r_clm = 9.81 * mass * np.sin(theta)
    resistance = r_drag + r_roll + r_clm

    power = ((resistance + 1.04 * mass * a) / (3600.0 * eta_d)) * v

    a0 = 0.7607578263
    a1 = 0.0336310284
    a2 = 0.0000630368

    fc_raw = a0 + a1 * power + a2 * (power**2)
    return fc_raw / 850.0
