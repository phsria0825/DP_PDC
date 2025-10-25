#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Analyze and compare simulation results."""

import sys
from pathlib import Path
import pickle
import argparse

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


def load_results(results_dir: Path):
    """Load simulation results from directory."""
    # This is a placeholder for loading saved results
    # In practice, you would save SimulationRecord objects and load them here
    pass


def create_detailed_analysis(record, output_dir: Path):
    """Create detailed analysis plots."""
    from simulator import SimulationRecord

    if not isinstance(record, SimulationRecord):
        raise TypeError("record must be a SimulationRecord instance")

    data = record.to_arrays()
    metrics = record.compute_metrics()

    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. Speed-Distance plot
    fig1, ax1 = plt.subplots(figsize=(12, 6))
    ax1.plot(data['distance'], data['speed'], 'b-', linewidth=2, label='Actual Speed')
    ax1.plot(data['distance'], data['target_speed'], 'r--', linewidth=1.5,
             alpha=0.7, label='Target Speed')
    ax1.set_xlabel('Distance (m)', fontsize=12)
    ax1.set_ylabel('Speed (m/s)', fontsize=12)
    ax1.set_title('Speed vs Distance', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=11)
    fig1.savefig(output_dir / 'speed_distance.png', dpi=150, bbox_inches='tight')
    plt.close(fig1)

    # 2. Acceleration histogram
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    ax2.hist(data['acceleration'], bins=50, color='purple', alpha=0.7, edgecolor='black')
    ax2.axvline(x=0, color='k', linestyle='-', linewidth=1)
    ax2.axvline(x=np.mean(data['acceleration']), color='r', linestyle='--',
               linewidth=2, label=f'Mean: {np.mean(data["acceleration"]):.3f}')
    ax2.set_xlabel('Acceleration (m/s²)', fontsize=12)
    ax2.set_ylabel('Frequency', fontsize=12)
    ax2.set_title('Acceleration Distribution', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=11)
    ax2.grid(True, alpha=0.3, axis='y')
    fig2.savefig(output_dir / 'acceleration_histogram.png', dpi=150, bbox_inches='tight')
    plt.close(fig2)

    # 3. Fuel efficiency analysis
    fig3, (ax3a, ax3b) = plt.subplots(1, 2, figsize=(14, 5))

    # Fuel rate vs speed
    ax3a.scatter(data['speed'], data['fuel_rate'], c=data['acceleration'],
                cmap='RdYlGn_r', s=10, alpha=0.5)
    ax3a.set_xlabel('Speed (m/s)', fontsize=12)
    ax3a.set_ylabel('Fuel Rate', fontsize=12)
    ax3a.set_title('Fuel Rate vs Speed', fontsize=13, fontweight='bold')
    ax3a.grid(True, alpha=0.3)
    cbar = plt.colorbar(ax3a.collections[0], ax=ax3a)
    cbar.set_label('Acceleration (m/s²)', fontsize=10)

    # Fuel rate vs acceleration
    ax3b.scatter(data['acceleration'], data['fuel_rate'], c=data['speed'],
                cmap='viridis', s=10, alpha=0.5)
    ax3b.set_xlabel('Acceleration (m/s²)', fontsize=12)
    ax3b.set_ylabel('Fuel Rate', fontsize=12)
    ax3b.set_title('Fuel Rate vs Acceleration', fontsize=13, fontweight='bold')
    ax3b.grid(True, alpha=0.3)
    cbar2 = plt.colorbar(ax3b.collections[0], ax=ax3b)
    cbar2.set_label('Speed (m/s)', fontsize=10)

    fig3.tight_layout()
    fig3.savefig(output_dir / 'fuel_analysis.png', dpi=150, bbox_inches='tight')
    plt.close(fig3)

    # 4. Metrics summary
    fig4, ax4 = plt.subplots(figsize=(10, 6))
    ax4.axis('off')

    metrics_text = f"""
    PERFORMANCE METRICS SUMMARY
    {'=' * 60}

    Fuel Consumption:
      Total Fuel                : {metrics['total_fuel']:.6f}
      Fuel per km               : {metrics['fuel_per_km']:.6f}

    Distance & Speed:
      Total Distance            : {metrics['total_distance']:.1f} m
      Average Speed             : {metrics['avg_speed']:.2f} m/s ({metrics['avg_speed']*3.6:.1f} km/h)

    Acceleration:
      Maximum Acceleration      : {metrics['max_accel']:.2f} m/s²
      Maximum Deceleration      : {metrics['min_decel']:.2f} m/s²
      RMS Jerk                  : {metrics['rms_jerk']:.3f} m/s³

    Optimization:
      Average Optimization Cost : {metrics['avg_cost']:.6f}
      Total Signal Violations   : {metrics['total_violations']}

    {'=' * 60}
    """

    ax4.text(0.1, 0.5, metrics_text, fontsize=11, family='monospace',
            verticalalignment='center', transform=ax4.transAxes)

    fig4.savefig(output_dir / 'metrics_summary.png', dpi=150, bbox_inches='tight')
    plt.close(fig4)

    print(f"\nDetailed analysis saved to: {output_dir}")
    print(f"  - speed_distance.png")
    print(f"  - acceleration_histogram.png")
    print(f"  - fuel_analysis.png")
    print(f"  - metrics_summary.png")


def export_to_csv(record, output_path: Path):
    """Export simulation data to CSV file."""
    import csv

    data = record.to_arrays()

    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)

        # Header
        writer.writerow([
            'time', 'distance', 'speed', 'target_speed',
            'acceleration', 'fuel_rate', 'rhc_cost', 'rhc_penalty'
        ])

        # Data rows
        for i in range(len(data['time'])):
            writer.writerow([
                data['time'][i],
                data['distance'][i],
                data['speed'][i],
                data['target_speed'][i],
                data['acceleration'][i],
                data['fuel_rate'][i],
                data['rhc_cost'][i],
                data['rhc_penalty'][i],
            ])

    print(f"Data exported to CSV: {output_path}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Analyze simulation results"
    )
    parser.add_argument(
        '--scenario',
        type=str,
        help='Scenario directory to analyze (e.g., simulation_results/scenario_1)'
    )
    parser.add_argument(
        '--export-csv',
        action='store_true',
        help='Export data to CSV file'
    )

    args = parser.parse_args()

    if args.scenario:
        scenario_dir = Path(args.scenario)
        if not scenario_dir.exists():
            print(f"Error: Directory not found: {scenario_dir}")
            return

        print(f"Analyzing results from: {scenario_dir}")

        # For now, this is a placeholder
        # In full implementation, you would load saved SimulationRecord objects
        print("\nNote: This script requires saved simulation data.")
        print("Run 'python run_simulation.py' first to generate results.")
    else:
        print("Usage: python analyze_results.py --scenario <path_to_scenario>")
        print("\nExample:")
        print("  python analyze_results.py --scenario simulation_results/scenario_1")


if __name__ == "__main__":
    main()
