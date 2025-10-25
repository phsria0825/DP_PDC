# -*- coding: utf-8 -*-
"""Rolling Horizon Control (RHC) based Eco-Driving System

This module implements a fuel-efficient driving system for navigating
through multiple signalized intersections using Rolling Horizon Control
and Dynamic Programming optimization.

Signal Format:
    Each traffic light provides: [Δd, tRemain, TrfState, red, yellow, green]
    - Δd: Relative distance to traffic light (m)
    - tRemain: Remaining time in current signal state (s)
    - TrfState: Current signal state (0=Red, 1=Yellow, 2=Green)
    - red: Red phase duration (s)
    - yellow: Yellow phase duration (s)
    - green: Green phase duration (s)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import numpy as np


ArrayLike = np.ndarray


@dataclass
class VehicleState:
    """Current state of the vehicle."""
    speed: float  # Current speed (m/s)
    distance: float  # Cumulative traveled distance (m)

    def to_array(self) -> ArrayLike:
        return np.array([self.speed, self.distance])


@dataclass
class SignalInfo:
    """Traffic signal information in the new format."""
    delta_distance: float  # Relative distance to signal (m)
    time_remaining: float  # Remaining time in current state (s)
    current_state: int  # 0=Red, 1=Yellow, 2=Green
    red_duration: float  # Red phase duration (s)
    yellow_duration: float  # Yellow phase duration (s)
    green_duration: float  # Green phase duration (s)

    @classmethod
    def from_array(cls, arr: ArrayLike) -> SignalInfo:
        """Create SignalInfo from array [Δd, tRemain, TrfState, red, yellow, green]."""
        arr = np.asarray(arr, dtype=float).reshape(-1)
        if arr.size < 6:
            raise ValueError("Signal array must have 6 elements")
        return cls(
            delta_distance=float(arr[0]),
            time_remaining=float(arr[1]),
            current_state=int(arr[2]),
            red_duration=float(arr[3]),
            yellow_duration=float(arr[4]),
            green_duration=float(arr[5])
        )

    def to_array(self) -> ArrayLike:
        return np.array([
            self.delta_distance,
            self.time_remaining,
            float(self.current_state),
            self.red_duration,
            self.yellow_duration,
            self.green_duration
        ])


@dataclass
class RoadSegment:
    """Road segment information."""
    intersection_id: int  # Intersection identifier
    distance: float  # Distance from start to intersection (m)
    length: float  # Segment length (m)
    grade: float  # Road gradient (rad)
    speed_limit: float  # Speed limit (m/s)

    def to_array(self) -> ArrayLike:
        return np.array([
            float(self.intersection_id),
            self.distance,
            self.length,
            self.grade,
            self.speed_limit
        ])


@dataclass
class OptimizationParams:
    """Parameters for DP optimization."""
    ds: float = 10.0  # Spatial discretization step (m)
    max_accel: float = 1.2  # Maximum acceleration (m/s^2)
    max_decel: float = 1.5  # Maximum deceleration (m/s^2)
    horizon_time: float = 180.0  # Prediction horizon time (s)
    speed_grid: Optional[ArrayLike] = None  # Speed discretization grid

    def to_array(self) -> ArrayLike:
        return np.array([self.ds, self.max_accel, self.max_decel, self.horizon_time])


class SignalScheduler:
    """Computes green windows for traffic signals based on new signal format."""

    @staticmethod
    def compute_green_windows(
        signals: List[SignalInfo],
        vehicle_distance: float,
        horizon_time: float = 180.0
    ) -> ArrayLike:
        """Compute green signal windows for each traffic light.

        Parameters
        ----------
        signals : List[SignalInfo]
            List of traffic signal information (up to 4 signals)
        vehicle_distance : float
            Current cumulative distance traveled by vehicle (m)
        horizon_time : float
            Time horizon for prediction (s)

        Returns
        -------
        ArrayLike
            Array of green windows with shape (N, 3) where each row is
            [absolute_distance, start_time, end_time]
        """
        windows = []

        for signal in signals:
            # Calculate absolute distance to intersection
            abs_distance = vehicle_distance + signal.delta_distance

            # Get signal cycle information
            cycle_time = signal.red_duration + signal.yellow_duration + signal.green_duration
            if cycle_time <= 0:
                continue

            # Determine current phase order: R->Y->G (0->1->2)
            phase_durations = [signal.red_duration, signal.yellow_duration, signal.green_duration]
            current_phase = signal.current_state
            time_remaining = signal.time_remaining

            # Simulate signal phases over the horizon
            t_cursor = 0.0
            phase_idx = current_phase
            remaining = time_remaining

            while t_cursor < horizon_time:
                phase_end = min(horizon_time, t_cursor + remaining)

                # If current phase is green (state 2), record the window
                if phase_idx == 2:
                    windows.append([abs_distance, t_cursor, phase_end])

                t_cursor += remaining
                if t_cursor >= horizon_time:
                    break

                # Move to next phase (R->Y->G->R...)
                phase_idx = (phase_idx + 1) % 3
                remaining = phase_durations[phase_idx]

        if not windows:
            return np.zeros((0, 3))

        return np.array(windows, dtype=float)


class EcoDrivingController:
    """Main Rolling Horizon Control (RHC) supervisor for eco-driving."""

    def __init__(self, params: Optional[OptimizationParams] = None):
        """Initialize the eco-driving controller.

        Parameters
        ----------
        params : OptimizationParams, optional
            Optimization parameters. Uses defaults if not provided.
        """
        self.params = params if params is not None else OptimizationParams()
        self.route_queue: List[RoadSegment] = []
        self.current_idx: int = 0

    def initialize_route(self, segments: List[RoadSegment]) -> None:
        """Initialize the route with road segments.

        Parameters
        ----------
        segments : List[RoadSegment]
            List of road segments sorted by distance
        """
        # Sort by distance to ensure proper ordering
        self.route_queue = sorted(segments, key=lambda s: s.distance)
        self.current_idx = 0

    def step(
        self,
        vehicle_state: VehicleState,
        signals: List[SignalInfo]
    ) -> Tuple[float, ArrayLike, ArrayLike, dict]:
        """Execute one RHC optimization step.

        Parameters
        ----------
        vehicle_state : VehicleState
            Current vehicle state (speed, distance)
        signals : List[SignalInfo]
            Traffic signal information (up to 4 signals)

        Returns
        -------
        v_set : float
            Target speed for next control interval (m/s)
        trajectory_speeds : ArrayLike
            Optimal speed profile
        trajectory_times : ArrayLike
            Time stamps for trajectory
        diagnostics : dict
            Diagnostic information
        """
        # Check if route is initialized
        if not self.route_queue:
            return vehicle_state.speed, np.array([vehicle_state.speed]), np.array([0.0]), {
                'current_idx': 0,
                'horizon_count': 0,
                'cost': 0.0,
                'penalties': 0
            }

        # Update horizon: select next 2 intersections
        horizon_indices = self._update_horizon(vehicle_state.distance)

        if len(horizon_indices) == 0:
            # Route completed
            self.route_queue = []
            self.current_idx = 0
            return vehicle_state.speed, np.zeros(0), np.zeros(0), {
                'current_idx': self.current_idx,
                'horizon_count': 0,
                'cost': 0.0,
                'penalties': 0,
                'status': 'route_completed'
            }

        # Prepare optimization buffer
        horizon_segments = [self.route_queue[i] for i in horizon_indices]
        segments_array, segment_steps, total_steps = self._prepare_segments(
            horizon_segments, vehicle_state.distance
        )

        if total_steps <= 0:
            return vehicle_state.speed, np.zeros(0), np.zeros(0), {
                'current_idx': self.current_idx,
                'horizon_count': len(horizon_indices),
                'cost': 0.0,
                'penalties': 0
            }

        # Compute signal green windows
        signal_windows = SignalScheduler.compute_green_windows(
            signals, vehicle_state.distance, self.params.horizon_time
        )

        # Prepare parameter vector
        speed_grid = self.params.speed_grid
        if speed_grid is None:
            speed_grid = np.arange(0.0, 30.0 + 1e-9, 2.0)

        parameter_vector = np.array([
            self.params.ds,
            self.params.max_accel,
            self.params.max_decel,
            vehicle_state.speed,
            vehicle_state.distance,
            self.params.horizon_time
        ])

        # Run DP optimization
        trajectory_speeds, trajectory_times, dp_info = velocity_optimiz_dp(
            segments_array,
            segment_steps,
            total_steps,
            parameter_vector,
            speed_grid,
            signal_windows
        )

        # Extract target speed
        if trajectory_speeds.size == 0:
            v_set = vehicle_state.speed
        elif trajectory_speeds.size >= 2:
            v_set = float(trajectory_speeds[1])
        else:
            v_set = float(trajectory_speeds[0])

        # Prepare diagnostics
        diagnostics = {
            'current_idx': self.current_idx,
            'horizon_indices': horizon_indices,
            'horizon_count': len(horizon_indices),
            'cost': float(dp_info[0]) if dp_info.size > 0 else 0.0,
            'total_steps': int(dp_info[1]) if dp_info.size > 1 else 0,
            'penalties': int(dp_info[2]) if dp_info.size > 2 else 0,
            'status': 'optimized'
        }

        return v_set, trajectory_speeds, trajectory_times, diagnostics

    def _update_horizon(self, current_distance: float) -> List[int]:
        """Update rolling horizon to select next 2 intersections.

        Parameters
        ----------
        current_distance : float
            Current traveled distance (m)

        Returns
        -------
        List[int]
            Indices of intersections in the horizon (up to 2)
        """
        # Find first intersection ahead of current position
        num_intersections = len(self.route_queue)
        idx = max(0, self.current_idx)

        while idx < num_intersections and current_distance >= self.route_queue[idx].distance - 1e-3:
            idx += 1

        self.current_idx = idx

        if idx >= num_intersections:
            return []

        # Select up to 2 intersections
        last_idx = min(num_intersections - 1, idx + 1)
        return list(range(idx, last_idx + 1))

    def _prepare_segments(
        self,
        segments: List[RoadSegment],
        current_distance: float
    ) -> Tuple[ArrayLike, ArrayLike, int]:
        """Prepare segment data for DP optimization.

        Parameters
        ----------
        segments : List[RoadSegment]
            Road segments in the horizon
        current_distance : float
            Current traveled distance (m)

        Returns
        -------
        segments_array : ArrayLike
            Segment data array (N, 5)
        segment_steps : ArrayLike
            Number of discretization steps per segment
        total_steps : int
            Total number of steps
        """
        if not segments:
            return np.zeros((0, 5)), np.zeros(0, dtype=int), 0

        segments_array = np.array([seg.to_array() for seg in segments])
        num_segments = len(segments)
        ds = self.params.ds

        segment_steps = np.zeros(num_segments, dtype=int)
        total_steps = 0

        for idx, seg in enumerate(segments):
            length = seg.length
            steps = max(1, int(round(length / ds)))
            segment_steps[idx] = steps
            total_steps += steps

        return segments_array, segment_steps, total_steps


def velocity_optimiz_dp(
    segments: ArrayLike,
    segment_steps: ArrayLike,
    total_steps: int,
    parameter_vector: ArrayLike,
    speed_vector: ArrayLike,
    signal_windows: ArrayLike,
) -> Tuple[ArrayLike, ArrayLike, ArrayLike]:
    """Dynamic Programming optimizer for eco-driving.

    Parameters
    ----------
    segments : ArrayLike
        Segment data (N, 5): [id, distance, length, grade, speed_limit]
    segment_steps : ArrayLike
        Number of steps per segment
    total_steps : int
        Total discretization steps
    parameter_vector : ArrayLike
        [ds, maxAccel, maxDecel, initialSpeed, currentDistance, horizonTime]
    speed_vector : ArrayLike
        Speed discretization grid
    signal_windows : ArrayLike
        Green windows (M, 3): [distance, start_time, end_time]

    Returns
    -------
    trajectory_speeds : ArrayLike
        Optimal speed profile
    trajectory_times : ArrayLike
        Arrival times at each step
    dp_info : ArrayLike
        [best_cost, total_steps, penalty_count]
    """
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
    current_distance = float(parameter_vector[4])

    speeds = speed_vector
    num_speeds = speeds.size

    # Initialize DP tables
    J = np.full((num_speeds, total_steps + 1), np.inf)  # Cost-to-go
    T = np.full((num_speeds, total_steps + 1), np.inf)  # Time-to-go
    policy = np.zeros((num_speeds, total_steps), dtype=np.int32)

    # Initial condition
    start_idx = int(np.argmin(np.abs(speeds - initial_speed)))
    J[start_idx, 0] = 0.0
    T[start_idx, 0] = 0.0

    # Precompute segment properties
    segment_boundary = np.cumsum(segment_steps)
    grades = segments[:, 3]
    limits = segments[:, 4]
    step_grades = np.repeat(grades, segment_steps)
    step_limits = np.repeat(limits, segment_steps)

    # Compute absolute distances for each step
    step_distances = current_distance + np.arange(1, total_steps + 1) * ds

    # Extract signal window data
    if signal_windows.size == 0:
        window_distances = np.zeros(0)
        window_starts = np.zeros(0)
        window_ends = np.zeros(0)
    else:
        window_distances = signal_windows[:, 0]
        window_starts = signal_windows[:, 1]
        window_ends = signal_windows[:, 2]

    # Forward DP
    for step in range(total_steps):
        grade = step_grades[step]
        limit = step_limits[step]
        step_distance = step_distances[step]

        for v_idx in range(num_speeds):
            if not np.isfinite(J[v_idx, step]):
                continue

            v_current = speeds[v_idx]
            t_current = T[v_idx, step]

            for v_next_idx in range(num_speeds):
                v_next = speeds[v_next_idx]

                # Speed limit constraint
                if v_next > limit + 1e-3:
                    continue

                # Acceleration constraint
                accel = compute_acceleration(v_current, v_next, ds)
                if accel > max_accel + 1e-6 or accel < -max_decel - 1e-6:
                    continue

                # Time feasibility
                stage_time, feasible = compute_traversal_time(v_current, v_next, ds)
                if not feasible:
                    continue

                t_arrival = t_current + stage_time

                # Fuel cost
                fuel_rate = fuel_prediction_model((v_current + v_next) / 2.0, accel, grade)
                stage_cost = fuel_rate * stage_time

                # Signal penalty
                penalty = compute_signal_penalty(
                    step,
                    step_distance,
                    t_arrival,
                    segment_boundary,
                    segments[:, 1],  # intersection distances
                    window_distances,
                    window_starts,
                    window_ends
                )

                total_cost = J[v_idx, step] + stage_cost + penalty

                # Update if better
                if total_cost + 1e-9 < J[v_next_idx, step + 1]:
                    J[v_next_idx, step + 1] = total_cost
                    T[v_next_idx, step + 1] = t_arrival
                    policy[v_next_idx, step] = v_idx

    # Backward path extraction
    terminal_idx = int(np.argmin(J[:, -1]))
    best_cost = J[terminal_idx, -1]

    if not np.isfinite(best_cost):
        return np.zeros(0), np.zeros(0), np.array([np.inf, float(total_steps), 0.0])

    path_idx = np.zeros(total_steps + 1, dtype=int)
    path_idx[-1] = terminal_idx
    for step in range(total_steps - 1, -1, -1):
        prev_idx = policy[path_idx[step + 1], step]
        if prev_idx <= 0 and step != 0:
            prev_idx = path_idx[step + 1]
        path_idx[step] = prev_idx

    trajectory_speeds = speeds[path_idx]
    trajectory_times = np.array([T[path_idx[node], node] for node in range(total_steps + 1)])

    # Evaluate signal compliance
    penalty_count = evaluate_signal_compliance(
        trajectory_times,
        step_distances,
        segment_boundary,
        segments[:, 1],
        window_distances,
        window_starts,
        window_ends
    )

    dp_info = np.array([best_cost, float(total_steps), float(penalty_count)])

    return trajectory_speeds, trajectory_times, dp_info


def compute_acceleration(v_current: float, v_next: float, ds: float) -> float:
    """Compute acceleration given current and next speeds."""
    if ds <= 0:
        raise ValueError("ds must be positive")
    return (v_next**2 - v_current**2) / (2.0 * ds)


def compute_traversal_time(v_current: float, v_next: float, ds: float) -> Tuple[float, bool]:
    """Compute time to traverse distance ds."""
    mean_speed = (v_current + v_next) / 2.0
    if mean_speed <= 0:
        return float("inf"), False
    dt = ds / mean_speed
    return float(dt), np.isfinite(dt) and dt > 0


def compute_signal_penalty(
    step: int,
    step_distance: float,
    arrival_time: float,
    segment_boundary: ArrayLike,
    intersection_distances: ArrayLike,
    window_distances: ArrayLike,
    window_starts: ArrayLike,
    window_ends: ArrayLike,
) -> float:
    """Compute penalty for signal violation.

    Parameters
    ----------
    step : int
        Current step index
    step_distance : float
        Absolute distance at this step
    arrival_time : float
        Time of arrival at this step
    segment_boundary : ArrayLike
        Cumulative step boundaries for segments
    intersection_distances : ArrayLike
        Absolute distances to intersections
    window_distances : ArrayLike
        Distances for green windows
    window_starts : ArrayLike
        Start times of green windows
    window_ends : ArrayLike
        End times of green windows

    Returns
    -------
    float
        Penalty value
    """
    if window_distances.size == 0:
        return 0.0

    # Check if this step reaches an intersection
    matches = np.where(segment_boundary == (step + 1))[0]
    if matches.size == 0:
        return 0.0

    intersection_distance = intersection_distances[matches[0]]

    # Find green windows for this intersection
    tolerance = 5.0  # Distance tolerance (m)
    mask = np.abs(window_distances - intersection_distance) < tolerance

    if not np.any(mask):
        # No signal information available - large penalty
        return 1e6

    starts = window_starts[mask]
    ends = window_ends[mask]

    # Check if arrival time falls in any green window
    is_green = np.any((arrival_time >= starts) & (arrival_time <= ends))
    if is_green:
        return 0.0

    # Red light penalty proportional to waiting time
    time_to_green = np.min(np.abs(np.concatenate((starts, ends)) - arrival_time))
    return 1e5 + 1e3 * float(time_to_green)


def evaluate_signal_compliance(
    trajectory_times: ArrayLike,
    step_distances: ArrayLike,
    segment_boundary: ArrayLike,
    intersection_distances: ArrayLike,
    window_distances: ArrayLike,
    window_starts: ArrayLike,
    window_ends: ArrayLike,
) -> int:
    """Evaluate how many signals were violated in the trajectory."""
    if segment_boundary.size == 0:
        return 0

    penalty_count = 0
    tolerance = 5.0

    for idx, boundary in enumerate(segment_boundary):
        step_index = int(boundary) - 1
        if step_index >= trajectory_times.size or step_index >= step_distances.size:
            continue

        arrival_time = trajectory_times[step_index]
        intersection_distance = intersection_distances[idx]

        mask = np.abs(window_distances - intersection_distance) < tolerance
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
    """Fuel consumption model for heavy vehicle (23 tons).

    Parameters
    ----------
    v : float
        Speed (m/s)
    a : float
        Acceleration (m/s^2)
    theta : float
        Road gradient (rad)

    Returns
    -------
    float
        Fuel consumption rate
    """
    # Vehicle parameters (23-ton heavy vehicle)
    rho = 1.2256  # Air density (kg/m^3)
    cd = 0.615  # Drag coefficient
    ch = 1 - 0.085 * 0  # Altitude correction
    af = 10.2  # Frontal area (m^2)
    cr = 1.25  # Rolling resistance coefficient
    c1 = 0.0328  # Rolling resistance velocity coefficient 1
    c2 = 4.575  # Rolling resistance velocity coefficient 2
    mass = 23000  # Vehicle mass (kg)
    eta_d = 0.85  # Drivetrain efficiency

    # Resistance forces
    r_drag = (rho / 25.92) * cd * ch * af * (v**2)
    r_roll = 9.81 * (cr / 1000.0) * (c1 * v + c2)
    r_clm = 9.81 * mass * np.sin(theta)
    resistance = r_drag + r_roll + r_clm

    # Required power
    power = ((resistance + 1.04 * mass * a) / (3600.0 * eta_d)) * v

    # Polynomial fuel consumption model (experimental data)
    a0 = 0.7607578263
    a1 = 0.0336310284
    a2 = 0.0000630368

    fc_raw = a0 + a1 * power + a2 * (power**2)
    return fc_raw / 850.0


def plot_trajectory(
    times: ArrayLike,
    speeds: ArrayLike,
    signals: Optional[List[SignalInfo]] = None,
    vehicle_start_distance: float = 0.0,
    ax: Optional["matplotlib.axes.Axes"] = None,
) -> Tuple["matplotlib.figure.Figure", "matplotlib.axes.Axes"]:
    """Plot speed trajectory with signal information.

    Parameters
    ----------
    times : ArrayLike
        Time stamps
    speeds : ArrayLike
        Speed profile
    signals : List[SignalInfo], optional
        Traffic signal information for visualization
    vehicle_start_distance : float
        Starting distance of vehicle
    ax : matplotlib Axes, optional
        Axes to plot on

    Returns
    -------
    figure, axes
        Matplotlib figure and axes
    """
    import matplotlib.pyplot as plt

    times = np.asarray(times, dtype=float).reshape(-1)
    speeds = np.asarray(speeds, dtype=float).reshape(-1)

    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 6))
    else:
        fig = ax.figure

    # Plot speed trajectory
    ax.plot(times, speeds, 'b-o', linewidth=2, markersize=4, label='Optimal Speed')

    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Speed (m/s)', fontsize=12)
    ax.set_title('Eco-Driving Speed Profile', fontsize=14, fontweight='bold')
    ax.grid(True, which='both', linestyle='--', alpha=0.3)
    ax.legend(fontsize=10)

    fig.tight_layout()
    return fig, ax
