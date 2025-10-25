# -*- coding: utf-8 -*-
"""Test suite for the eco-driving system with new signal format."""

import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rhc_eco_driving import (
    VehicleState,
    SignalInfo,
    RoadSegment,
    OptimizationParams,
    SignalScheduler,
    EcoDrivingController,
    fuel_prediction_model,
    compute_acceleration,
    compute_traversal_time,
)


def test_signal_info_creation():
    """Test SignalInfo data structure."""
    signal_array = np.array([150.0, 12.0, 2, 30.0, 3.0, 35.0])
    signal = SignalInfo.from_array(signal_array)

    assert signal.delta_distance == 150.0
    assert signal.time_remaining == 12.0
    assert signal.current_state == 2  # Green
    assert signal.red_duration == 30.0
    assert signal.yellow_duration == 3.0
    assert signal.green_duration == 35.0

    # Test round-trip conversion
    recovered = signal.to_array()
    np.testing.assert_array_almost_equal(signal_array, recovered)


def test_signal_scheduler_green_windows():
    """Test signal scheduler computes correct green windows."""
    # Signal 1: 150m ahead, currently green with 12s remaining
    signal1 = SignalInfo(
        delta_distance=150.0,
        time_remaining=12.0,
        current_state=2,  # Green
        red_duration=30.0,
        yellow_duration=3.0,
        green_duration=35.0
    )

    # Signal 2: 300m ahead, currently red with 20s remaining
    signal2 = SignalInfo(
        delta_distance=300.0,
        time_remaining=20.0,
        current_state=0,  # Red
        red_duration=30.0,
        yellow_duration=3.0,
        green_duration=35.0
    )

    vehicle_distance = 50.0  # Current position
    horizon_time = 120.0

    windows = SignalScheduler.compute_green_windows(
        [signal1, signal2], vehicle_distance, horizon_time
    )

    assert windows.shape[1] == 3  # [distance, start_time, end_time]
    assert windows.shape[0] > 0  # At least one window

    # Signal 1 should have green window starting at t=0
    signal1_windows = windows[windows[:, 0] == 200.0]  # 50 + 150
    assert signal1_windows.shape[0] > 0
    assert signal1_windows[0, 1] == 0.0  # Starts immediately
    assert signal1_windows[0, 2] == 12.0  # Ends after 12s


def test_eco_driving_controller_initialization():
    """Test controller initialization and route setup."""
    controller = EcoDrivingController()

    segments = [
        RoadSegment(1, 200.0, 100.0, 0.01, 20.0),
        RoadSegment(2, 400.0, 150.0, 0.0, 22.0),
        RoadSegment(3, 650.0, 120.0, -0.01, 20.0),
    ]

    controller.initialize_route(segments)
    assert len(controller.route_queue) == 3
    assert controller.current_idx == 0


def test_eco_driving_controller_step():
    """Test complete optimization cycle."""
    # Setup controller
    params = OptimizationParams(
        ds=10.0,
        max_accel=1.2,
        max_decel=1.5,
        horizon_time=120.0
    )
    controller = EcoDrivingController(params)

    # Define route (3 intersections)
    segments = [
        RoadSegment(1, 200.0, 100.0, 0.01, 20.0),
        RoadSegment(2, 400.0, 150.0, 0.0, 22.0),
        RoadSegment(3, 650.0, 120.0, -0.01, 20.0),
    ]
    controller.initialize_route(segments)

    # Vehicle state: 50m traveled, speed 15 m/s
    vehicle = VehicleState(speed=15.0, distance=50.0)

    # Signal information (2 signals visible)
    signals = [
        SignalInfo(
            delta_distance=150.0,  # 200m absolute
            time_remaining=12.0,
            current_state=2,  # Green
            red_duration=30.0,
            yellow_duration=3.0,
            green_duration=35.0
        ),
        SignalInfo(
            delta_distance=350.0,  # 400m absolute
            time_remaining=20.0,
            current_state=0,  # Red
            red_duration=30.0,
            yellow_duration=3.0,
            green_duration=35.0
        ),
    ]

    # Run optimization
    v_set, speeds, times, diag = controller.step(vehicle, signals)

    # Assertions
    assert isinstance(v_set, float)
    assert v_set > 0
    assert speeds.ndim == 1
    assert times.ndim == 1
    assert speeds.size == times.size
    assert diag['horizon_count'] > 0
    assert diag['status'] == 'optimized'
    print(f"Optimized speed: {v_set:.2f} m/s")
    print(f"Cost: {diag['cost']:.4f}")
    print(f"Penalties: {diag['penalties']}")


def test_horizon_update():
    """Test horizon update mechanism."""
    controller = EcoDrivingController()

    segments = [
        RoadSegment(1, 200.0, 100.0, 0.0, 20.0),
        RoadSegment(2, 400.0, 150.0, 0.0, 22.0),
        RoadSegment(3, 650.0, 120.0, 0.0, 20.0),
    ]
    controller.initialize_route(segments)

    # At start, should select first 2 intersections
    horizon = controller._update_horizon(50.0)
    assert len(horizon) == 2
    assert horizon[0] == 0
    assert horizon[1] == 1

    # After passing first intersection
    horizon = controller._update_horizon(250.0)
    assert len(horizon) == 2
    assert horizon[0] == 1
    assert horizon[1] == 2

    # After passing second intersection
    horizon = controller._update_horizon(450.0)
    assert len(horizon) == 1
    assert horizon[0] == 2

    # After passing all intersections
    horizon = controller._update_horizon(700.0)
    assert len(horizon) == 0


def test_fuel_prediction_model():
    """Test fuel consumption model."""
    # Test with typical driving conditions
    v = 15.0  # m/s (~54 km/h)
    a = 0.5  # m/s^2 (mild acceleration)
    theta = 0.01  # rad (~0.57 degrees uphill)

    fc = fuel_prediction_model(v, a, theta)
    assert fc > 0  # Fuel consumption should be positive

    # Test with deceleration (should be lower fuel consumption)
    fc_decel = fuel_prediction_model(v, -0.5, theta)
    assert fc_decel < fc  # Decelerating uses less fuel


def test_compute_acceleration():
    """Test acceleration computation."""
    v_current = 10.0
    v_next = 12.0
    ds = 10.0

    accel = compute_acceleration(v_current, v_next, ds)
    expected = (12.0**2 - 10.0**2) / (2.0 * 10.0)
    assert abs(accel - expected) < 1e-6


def test_compute_traversal_time():
    """Test traversal time computation."""
    v_current = 10.0
    v_next = 12.0
    ds = 10.0

    dt, feasible = compute_traversal_time(v_current, v_next, ds)
    assert feasible
    expected = ds / ((v_current + v_next) / 2.0)
    assert abs(dt - expected) < 1e-6

    # Test infeasible case (zero speed)
    dt, feasible = compute_traversal_time(0.0, 0.0, ds)
    assert not feasible


def test_multiple_signals():
    """Test with maximum 4 signals."""
    controller = EcoDrivingController()

    segments = [
        RoadSegment(1, 200.0, 100.0, 0.0, 20.0),
        RoadSegment(2, 400.0, 150.0, 0.0, 22.0),
        RoadSegment(3, 650.0, 120.0, 0.0, 20.0),
        RoadSegment(4, 850.0, 100.0, 0.0, 20.0),
    ]
    controller.initialize_route(segments)

    vehicle = VehicleState(speed=15.0, distance=50.0)

    # Provide 4 signals
    signals = [
        SignalInfo(150.0, 10.0, 2, 30.0, 3.0, 35.0),
        SignalInfo(350.0, 20.0, 0, 30.0, 3.0, 35.0),
        SignalInfo(600.0, 5.0, 1, 30.0, 3.0, 35.0),
        SignalInfo(800.0, 30.0, 0, 30.0, 3.0, 35.0),
    ]

    v_set, speeds, times, diag = controller.step(vehicle, signals)

    assert isinstance(v_set, float)
    assert v_set > 0
    assert diag['status'] == 'optimized'


def test_route_completion():
    """Test behavior when route is completed."""
    controller = EcoDrivingController()

    segments = [
        RoadSegment(1, 200.0, 100.0, 0.0, 20.0),
    ]
    controller.initialize_route(segments)

    # Vehicle has passed all intersections
    vehicle = VehicleState(speed=15.0, distance=250.0)
    signals = []

    v_set, speeds, times, diag = controller.step(vehicle, signals)

    assert v_set == vehicle.speed  # Should maintain current speed
    assert speeds.size == 0
    assert diag['status'] == 'route_completed'


def test_signal_phases_cycling():
    """Test signal phase cycling: Red -> Yellow -> Green -> Red."""
    signal = SignalInfo(
        delta_distance=200.0,
        time_remaining=5.0,
        current_state=0,  # Red
        red_duration=30.0,
        yellow_duration=3.0,
        green_duration=35.0
    )

    vehicle_distance = 0.0
    horizon_time = 100.0

    windows = SignalScheduler.compute_green_windows(
        [signal], vehicle_distance, horizon_time
    )

    # Should have green windows
    assert windows.shape[0] > 0

    # First green should start after red (5s) + yellow (3s) = 8s
    assert abs(windows[0, 1] - 8.0) < 0.1


if __name__ == "__main__":
    print("Running eco-driving system tests...")
    pytest.main([__file__, "-v"])
