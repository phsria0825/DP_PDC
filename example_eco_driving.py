# -*- coding: utf-8 -*-
"""Example usage of the eco-driving system.

This example demonstrates a complete eco-driving scenario with multiple
signalized intersections using the new signal format.
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rhc_eco_driving import (
    VehicleState,
    SignalInfo,
    RoadSegment,
    OptimizationParams,
    EcoDrivingController,
    plot_trajectory,
)


def create_example_route():
    """Create an example route with 4 intersections."""
    segments = [
        # [id, distance(m), length(m), grade(rad), speed_limit(m/s)]
        RoadSegment(
            intersection_id=1,
            distance=250.0,
            length=150.0,
            grade=0.005,  # Slight uphill
            speed_limit=20.0  # ~72 km/h
        ),
        RoadSegment(
            intersection_id=2,
            distance=450.0,
            length=200.0,
            grade=0.0,  # Flat
            speed_limit=22.0  # ~79 km/h
        ),
        RoadSegment(
            intersection_id=3,
            distance=700.0,
            length=180.0,
            grade=-0.008,  # Slight downhill
            speed_limit=20.0
        ),
        RoadSegment(
            intersection_id=4,
            distance=950.0,
            length=150.0,
            grade=0.002,
            speed_limit=20.0
        ),
    ]
    return segments


def create_example_signals(vehicle_distance: float):
    """Create example signal information based on vehicle position.

    Signal format: [Δd, tRemain, TrfState, red, yellow, green]
    - Δd: Relative distance to signal (m)
    - tRemain: Remaining time in current state (s)
    - TrfState: Current state (0=Red, 1=Yellow, 2=Green)
    - red, yellow, green: Phase durations (s)
    """
    # Signal 1 at 250m: Currently green, ending soon
    signal1 = SignalInfo(
        delta_distance=250.0 - vehicle_distance,
        time_remaining=15.0,
        current_state=2,  # Green
        red_duration=35.0,
        yellow_duration=3.0,
        green_duration=30.0
    )

    # Signal 2 at 450m: Currently red, will turn green soon
    signal2 = SignalInfo(
        delta_distance=450.0 - vehicle_distance,
        time_remaining=18.0,
        current_state=0,  # Red
        red_duration=35.0,
        yellow_duration=3.0,
        green_duration=30.0
    )

    # Signal 3 at 700m: Currently green
    signal3 = SignalInfo(
        delta_distance=700.0 - vehicle_distance,
        time_remaining=25.0,
        current_state=2,  # Green
        red_duration=40.0,
        yellow_duration=3.0,
        green_duration=35.0
    )

    # Signal 4 at 950m: Currently yellow (about to turn red)
    signal4 = SignalInfo(
        delta_distance=950.0 - vehicle_distance,
        time_remaining=2.0,
        current_state=1,  # Yellow
        red_duration=40.0,
        yellow_duration=3.0,
        green_duration=35.0
    )

    # Return only visible signals (within reasonable range)
    signals = []
    for sig in [signal1, signal2, signal3, signal4]:
        if 0 < sig.delta_distance < 1000.0:
            signals.append(sig)

    return signals[:4]  # Maximum 4 signals


def simulate_eco_driving():
    """Run a complete eco-driving simulation."""
    print("=" * 70)
    print("Eco-Driving Simulation with RHC and New Signal Format")
    print("=" * 70)

    # Setup optimization parameters
    params = OptimizationParams(
        ds=10.0,  # Spatial discretization: 10m
        max_accel=1.2,  # Maximum acceleration: 1.2 m/s^2
        max_decel=1.5,  # Maximum deceleration: 1.5 m/s^2
        horizon_time=150.0,  # Prediction horizon: 150s
    )

    # Initialize controller
    controller = EcoDrivingController(params)

    # Create route
    route = create_example_route()
    controller.initialize_route(route)

    print(f"\nRoute Information:")
    print(f"  Number of intersections: {len(route)}")
    for seg in route:
        print(f"    Intersection {seg.intersection_id}: {seg.distance:.0f}m, "
              f"limit={seg.speed_limit:.1f} m/s, grade={seg.grade:.4f} rad")

    # Initial vehicle state
    initial_speed = 15.0  # m/s (~54 km/h)
    initial_distance = 80.0  # m

    vehicle = VehicleState(speed=initial_speed, distance=initial_distance)

    print(f"\nInitial Vehicle State:")
    print(f"  Speed: {vehicle.speed:.2f} m/s ({vehicle.speed * 3.6:.1f} km/h)")
    print(f"  Distance: {vehicle.distance:.1f} m")

    # Get signal information
    signals = create_example_signals(vehicle.distance)

    print(f"\nTraffic Signal Information ({len(signals)} signals):")
    for i, sig in enumerate(signals, 1):
        state_name = ['Red', 'Yellow', 'Green'][sig.current_state]
        abs_dist = vehicle.distance + sig.delta_distance
        print(f"  Signal {i} at {abs_dist:.0f}m:")
        print(f"    Current: {state_name} ({sig.time_remaining:.1f}s remaining)")
        print(f"    Cycle: R={sig.red_duration:.0f}s, Y={sig.yellow_duration:.0f}s, "
              f"G={sig.green_duration:.0f}s")

    # Run optimization
    print(f"\nRunning RHC Optimization...")
    v_set, speeds, times, diagnostics = controller.step(vehicle, signals)

    # Display results
    print(f"\n" + "=" * 70)
    print("Optimization Results")
    print("=" * 70)
    print(f"\nTarget Speed: {v_set:.2f} m/s ({v_set * 3.6:.1f} km/h)")
    print(f"\nDiagnostics:")
    print(f"  Status: {diagnostics['status']}")
    print(f"  Horizon count: {diagnostics['horizon_count']} intersections")
    print(f"  Current index: {diagnostics['current_idx']}")
    print(f"  Total steps: {diagnostics['total_steps']}")
    print(f"  Optimal cost: {diagnostics['cost']:.6f}")
    print(f"  Signal violations: {diagnostics['penalties']}")

    if speeds.size > 0:
        print(f"\nSpeed Profile:")
        print(f"  Initial speed: {speeds[0]:.2f} m/s")
        print(f"  Final speed: {speeds[-1]:.2f} m/s")
        print(f"  Max speed: {speeds.max():.2f} m/s")
        print(f"  Min speed: {speeds.min():.2f} m/s")
        print(f"  Total time: {times[-1]:.2f} s")

    return speeds, times, signals, vehicle, diagnostics


def plot_results(speeds, times, signals, vehicle, diagnostics):
    """Create visualization of results."""
    if speeds.size == 0:
        print("\nNo trajectory to plot.")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))

    # Plot 1: Speed vs Time
    ax1.plot(times, speeds, 'b-o', linewidth=2, markersize=3, label='Optimal Speed')
    ax1.axhline(y=20.0, color='r', linestyle='--', alpha=0.5, label='Speed Limit')
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Speed (m/s)', fontsize=12)
    ax1.set_title('Eco-Driving Speed Profile', fontsize=14, fontweight='bold')
    ax1.grid(True, which='both', linestyle='--', alpha=0.3)
    ax1.legend(fontsize=10)

    # Add speed in km/h on right axis
    ax1_right = ax1.twinx()
    ax1_right.set_ylabel('Speed (km/h)', fontsize=12, color='gray')
    ax1_right.set_ylim(np.array(ax1.get_ylim()) * 3.6)
    ax1_right.tick_params(axis='y', labelcolor='gray')

    # Plot 2: Distance vs Time with signal information
    ds = 10.0  # discretization step
    distances = vehicle.distance + np.arange(len(speeds)) * ds
    ax2.plot(times, distances, 'g-o', linewidth=2, markersize=3, label='Vehicle Position')

    # Plot signal locations
    colors = {0: 'red', 1: 'yellow', 2: 'green'}
    for i, sig in enumerate(signals, 1):
        abs_dist = vehicle.distance + sig.delta_distance
        if abs_dist < distances.max():
            color = colors[sig.current_state]
            ax2.axhline(y=abs_dist, color=color, linestyle=':', alpha=0.6, linewidth=2)
            ax2.text(times.max() * 0.02, abs_dist + 10, f'Signal {i}',
                    fontsize=9, color=color, fontweight='bold')

    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Distance (m)', fontsize=12)
    ax2.set_title('Vehicle Position vs Time', fontsize=14, fontweight='bold')
    ax2.grid(True, which='both', linestyle='--', alpha=0.3)
    ax2.legend(fontsize=10)

    # Add overall title
    fig.suptitle(
        f'RHC Eco-Driving Optimization Results\n'
        f'Cost: {diagnostics["cost"]:.6f} | Violations: {diagnostics["penalties"]} | '
        f'Horizon: {diagnostics["horizon_count"]} intersections',
        fontsize=12,
        fontweight='bold'
    )

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    return fig


def main():
    """Main function to run the example."""
    # Run simulation
    speeds, times, signals, vehicle, diagnostics = simulate_eco_driving()

    # Create visualization
    print("\n" + "=" * 70)
    print("Generating visualization...")
    print("=" * 70)

    fig = plot_results(speeds, times, signals, vehicle, diagnostics)

    # Save plot
    output_path = ROOT / "eco_driving_result.png"
    if fig is not None:
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\nPlot saved to: {output_path}")

        # Try to display
        try:
            plt.show()
        except Exception:
            print("(Display not available - plot saved to file)")
    else:
        print("\nNo plot generated.")

    print("\n" + "=" * 70)
    print("Simulation Complete!")
    print("=" * 70)


if __name__ == "__main__":
    main()
