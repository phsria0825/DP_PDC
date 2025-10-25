#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run complete eco-driving simulation with multiple scenarios."""

import sys
from pathlib import Path
import argparse

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulator import (
    EcoDrivingSimulator,
    SimulationConfig,
    TrafficLight
)
from rhc_eco_driving import RoadSegment


def create_scenario_1():
    """Scenario 1: Simple route with 3 well-coordinated signals."""
    print("\n" + "=" * 70)
    print("SCENARIO 1: Well-Coordinated Signals")
    print("=" * 70)
    print("Description: 3 traffic lights with coordinated timing")
    print("             Designed for smooth flow if optimal speed is maintained")
    print()

    config = SimulationConfig(
        dt=0.1,
        total_time=150.0,
        control_interval=1.0,
        initial_speed=15.0,
        initial_distance=0.0,
        ds=10.0,
        max_accel=1.2,
        max_decel=1.5,
        horizon_time=180.0,
        save_plots=True,
        output_dir="simulation_results/scenario_1"
    )

    simulator = EcoDrivingSimulator(config)

    # Add traffic lights - coordinated green wave
    simulator.add_traffic_light(
        intersection_id=1,
        distance=250.0,
        red=30.0,
        yellow=3.0,
        green=35.0,
        initial_state=2,  # Green
        initial_time=5.0
    )

    simulator.add_traffic_light(
        intersection_id=2,
        distance=500.0,
        red=30.0,
        yellow=3.0,
        green=35.0,
        initial_state=2,  # Green
        initial_time=20.0
    )

    simulator.add_traffic_light(
        intersection_id=3,
        distance=800.0,
        red=30.0,
        yellow=3.0,
        green=35.0,
        initial_state=0,  # Red
        initial_time=10.0
    )

    # Setup route
    segments = [
        RoadSegment(1, 250.0, 150.0, 0.005, 20.0),
        RoadSegment(2, 500.0, 200.0, 0.0, 22.0),
        RoadSegment(3, 800.0, 180.0, -0.003, 20.0),
    ]
    simulator.setup_route(segments)

    return simulator


def create_scenario_2():
    """Scenario 2: Challenging route with poorly coordinated signals."""
    print("\n" + "=" * 70)
    print("SCENARIO 2: Challenging Coordination")
    print("=" * 70)
    print("Description: 4 traffic lights with difficult timing")
    print("             Tests ability to optimize under constraints")
    print()

    config = SimulationConfig(
        dt=0.1,
        total_time=200.0,
        control_interval=1.0,
        initial_speed=12.0,
        initial_distance=0.0,
        ds=8.0,
        max_accel=1.0,
        max_decel=1.8,
        horizon_time=200.0,
        save_plots=True,
        output_dir="simulation_results/scenario_2"
    )

    simulator = EcoDrivingSimulator(config)

    # Add traffic lights - challenging coordination
    simulator.add_traffic_light(
        intersection_id=1,
        distance=200.0,
        red=35.0,
        yellow=3.0,
        green=25.0,
        initial_state=0,  # Red
        initial_time=5.0
    )

    simulator.add_traffic_light(
        intersection_id=2,
        distance=400.0,
        red=40.0,
        yellow=3.0,
        green=30.0,
        initial_state=1,  # Yellow
        initial_time=1.0
    )

    simulator.add_traffic_light(
        intersection_id=3,
        distance=650.0,
        red=35.0,
        yellow=3.0,
        green=28.0,
        initial_state=2,  # Green
        initial_time=15.0
    )

    simulator.add_traffic_light(
        intersection_id=4,
        distance=950.0,
        red=38.0,
        yellow=3.0,
        green=32.0,
        initial_state=0,  # Red
        initial_time=20.0
    )

    # Setup route with varying grades
    segments = [
        RoadSegment(1, 200.0, 120.0, 0.01, 18.0),
        RoadSegment(2, 400.0, 180.0, 0.005, 20.0),
        RoadSegment(3, 650.0, 200.0, 0.0, 22.0),
        RoadSegment(4, 950.0, 150.0, -0.008, 20.0),
    ]
    simulator.setup_route(segments)

    return simulator


def create_scenario_3():
    """Scenario 3: Long route with mixed conditions."""
    print("\n" + "=" * 70)
    print("SCENARIO 3: Long Route - Mixed Conditions")
    print("=" * 70)
    print("Description: Extended route with varied terrain and signal patterns")
    print("             Tests long-term optimization and adaptation")
    print()

    config = SimulationConfig(
        dt=0.1,
        total_time=250.0,
        control_interval=1.0,
        initial_speed=18.0,
        initial_distance=0.0,
        ds=12.0,
        max_accel=1.5,
        max_decel=2.0,
        horizon_time=150.0,
        save_plots=True,
        output_dir="simulation_results/scenario_3"
    )

    simulator = EcoDrivingSimulator(config)

    # Add traffic lights - mixed patterns
    simulator.add_traffic_light(
        intersection_id=1,
        distance=300.0,
        red=30.0,
        yellow=3.0,
        green=40.0,
        initial_state=2,  # Green
        initial_time=25.0
    )

    simulator.add_traffic_light(
        intersection_id=2,
        distance=600.0,
        red=35.0,
        yellow=3.0,
        green=35.0,
        initial_state=0,  # Red
        initial_time=15.0
    )

    simulator.add_traffic_light(
        intersection_id=3,
        distance=900.0,
        red=32.0,
        yellow=3.0,
        green=38.0,
        initial_state=2,  # Green
        initial_time=10.0
    )

    simulator.add_traffic_light(
        intersection_id=4,
        distance=1250.0,
        red=40.0,
        yellow=3.0,
        green=35.0,
        initial_state=1,  # Yellow
        initial_time=2.0
    )

    # Setup route with varied terrain
    segments = [
        RoadSegment(1, 300.0, 200.0, 0.008, 22.0),   # Uphill
        RoadSegment(2, 600.0, 250.0, 0.002, 25.0),   # Gentle uphill
        RoadSegment(3, 900.0, 220.0, -0.005, 23.0),  # Downhill
        RoadSegment(4, 1250.0, 200.0, 0.0, 20.0),    # Flat
    ]
    simulator.setup_route(segments)

    return simulator


def run_scenario(simulator: EcoDrivingSimulator, scenario_name: str):
    """Run a single scenario."""
    print(f"\nRunning {scenario_name}...")

    # Run simulation
    record = simulator.run()

    # Visualize results
    print(f"\nGenerating visualization for {scenario_name}...")
    simulator.visualize_results(save=True, show=False)

    # Save metrics to file
    metrics = record.compute_metrics()
    output_dir = Path(simulator.config.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    metrics_file = output_dir / "metrics.txt"
    with open(metrics_file, 'w') as f:
        f.write(f"{scenario_name} - Performance Metrics\n")
        f.write("=" * 60 + "\n\n")
        for key, value in metrics.items():
            f.write(f"{key:20s}: {value:15.6f}\n")

    print(f"Metrics saved to: {metrics_file}")

    return record, metrics


def compare_scenarios(results: dict):
    """Create comparison visualization."""
    import matplotlib.pyplot as plt
    import numpy as np

    scenario_names = list(results.keys())
    metrics_keys = ['total_fuel', 'fuel_per_km', 'avg_speed', 'total_violations', 'rms_jerk']
    metrics_labels = ['Total Fuel', 'Fuel/km', 'Avg Speed\n(m/s)', 'Violations', 'RMS Jerk\n(m/s³)']

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    axes = axes.flatten()

    for idx, (key, label) in enumerate(zip(metrics_keys, metrics_labels)):
        ax = axes[idx]
        values = [results[name][key] for name in scenario_names]

        bars = ax.bar(scenario_names, values, color=['skyblue', 'lightcoral', 'lightgreen'])
        ax.set_ylabel(label, fontsize=11)
        ax.set_title(f'{label}', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3, axis='y')

        # Add value labels on bars
        for bar, val in zip(bars, values):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{val:.3f}', ha='center', va='bottom', fontsize=9)

    # Remove extra subplot
    fig.delaxes(axes[5])

    fig.suptitle('Scenario Comparison', fontsize=14, fontweight='bold')
    plt.tight_layout()

    output_path = Path("simulation_results/scenario_comparison.png")
    output_path.parent.mkdir(exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nComparison plot saved to: {output_path}")
    plt.close(fig)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Run eco-driving simulation scenarios"
    )
    parser.add_argument(
        '--scenario',
        type=int,
        choices=[1, 2, 3],
        help='Run specific scenario (1, 2, or 3). If not specified, runs all.'
    )
    parser.add_argument(
        '--compare',
        action='store_true',
        help='Generate comparison plot (only when running all scenarios)'
    )

    args = parser.parse_args()

    scenarios = {
        'Scenario 1': create_scenario_1,
        'Scenario 2': create_scenario_2,
        'Scenario 3': create_scenario_3,
    }

    if args.scenario:
        # Run single scenario
        scenario_name = f'Scenario {args.scenario}'
        scenario_func = list(scenarios.values())[args.scenario - 1]
        simulator = scenario_func()
        run_scenario(simulator, scenario_name)
    else:
        # Run all scenarios
        print("\n" + "=" * 70)
        print("RUNNING ALL SCENARIOS")
        print("=" * 70)

        all_results = {}

        for name, create_func in scenarios.items():
            simulator = create_func()
            record, metrics = run_scenario(simulator, name)
            all_results[name] = metrics

        if args.compare:
            print("\n" + "=" * 70)
            print("GENERATING COMPARISON")
            print("=" * 70)
            compare_scenarios(all_results)

        # Summary
        print("\n" + "=" * 70)
        print("ALL SCENARIOS COMPLETED")
        print("=" * 70)
        print("\nSummary:")
        for name, metrics in all_results.items():
            print(f"\n{name}:")
            print(f"  Fuel: {metrics['total_fuel']:.6f}")
            print(f"  Distance: {metrics['total_distance']:.1f}m")
            print(f"  Violations: {metrics['total_violations']}")
            print(f"  Avg Speed: {metrics['avg_speed']:.2f}m/s")

    print("\n" + "=" * 70)
    print("SIMULATION COMPLETE")
    print("=" * 70)


if __name__ == "__main__":
    main()
