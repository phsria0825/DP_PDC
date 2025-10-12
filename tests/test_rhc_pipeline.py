# -*- coding: utf-8 -*-
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rhc_pipeline import (
    RhcSupervisor,
    compute_acceleration,
    compute_traversal_time,
    fuel_prediction_model,
    plot_speed_profile,
    initialize_route_queue,
    update_horizon,
)


def build_sample_inputs():
    vehicle_state = np.array([12.0, 40.0])
    map_data = np.array(
        [
            [1, 120, 60, 0.01, 18],
            [2, 200, 80, 0.0, 20],
            [3, 320, 90, -0.015, 22],
        ]
    )
    spat_base = np.array(
        [
            [1, 2, 15],
            [2, 1, 20],
            [3, 3, 10],
        ]
    )
    spat_durations = np.array(
        [
            [30, 25, 20],
            [30, 35, 25],
            [20, 40, 30],
        ]
    )
    spat_green = np.array(
        [
            [1, 0, 0],
            [0, 1, 1],
            [1, 0, 1],
        ]
    )
    return vehicle_state, map_data, spat_base, spat_durations, spat_green


def test_initialize_route_queue_orders_by_distance():
    _, map_data, *_ = build_sample_inputs()
    queue = initialize_route_queue(map_data)
    assert np.all(np.diff(queue[:, 1]) >= 0)


def test_update_horizon_selects_two_intersections():
    vehicle_state, map_data, *_ = build_sample_inputs()
    queue = initialize_route_queue(map_data)
    horizon, idx = update_horizon(queue, vehicle_state, 0)
    assert horizon.size == 2
    assert idx == horizon[0]


def test_supervisor_runs_complete_cycle():
    vehicle_state, map_data, spat_base, spat_durations, spat_green = build_sample_inputs()
    supervisor = RhcSupervisor()
    v_set, speeds, times, diag = supervisor.step(
        vehicle_state,
        map_data,
        spat_base,
        spat_durations,
        spat_green,
        options=[8.0, 1.5, 1.8, 160.0, 1.0],
    )

    assert isinstance(v_set, float)
    assert speeds.ndim == 1 and times.ndim == 1
    assert speeds.size == times.size
    assert diag.size == 5

    # Call again without reinitialisation to ensure horizon progress is tracked.
    vehicle_state_followup = np.array([speeds[0] if speeds.size else 10.0, 150.0])
    v_set_2, speeds_2, times_2, diag_2 = supervisor.step(
        vehicle_state_followup,
        map_data,
        spat_base,
        spat_durations,
        spat_green,
        options=[8.0, 1.5, 1.8, 160.0, 0.0],
    )
    assert isinstance(v_set_2, float)
    assert diag_2[0] >= diag[0]
    assert speeds_2.size == times_2.size


def test_low_level_utilities_behave_reasonably():
    accel = compute_acceleration(10.0, 12.0, 5.0)
    assert accel > 0

    dt, feasible = compute_traversal_time(10.0, 12.0, 5.0)
    assert feasible and dt > 0

    fc = fuel_prediction_model(10.0, accel, 0.01)
    assert fc > 0


def test_plot_speed_profile_returns_figure_and_axes():
    times = np.array([0.0, 1.0, 2.0])
    speeds = np.array([10.0, 11.0, 12.5])

    fig, ax = plot_speed_profile(times, speeds, label="Test trajectory")
    assert fig is ax.figure
    assert len(ax.lines) == 1
    assert ax.lines[0].get_label() == "Test trajectory"

    fig.canvas.draw()

    import matplotlib.pyplot as plt

    plt.close(fig)
