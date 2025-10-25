# -*- coding: utf-8 -*-
"""Complete simulation framework for eco-driving system.

This module provides a full closed-loop simulation environment that:
- Simulates vehicle dynamics over time
- Updates traffic signal states in real-time
- Calls RHC controller at each time step
- Records trajectory and performance metrics
- Visualizes results
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
import numpy as np
from pathlib import Path


@dataclass
class SimulationConfig:
    """Configuration for simulation."""
    dt: float = 0.1  # Time step (s)
    total_time: float = 300.0  # Total simulation time (s)
    control_interval: float = 1.0  # RHC update interval (s)

    # Vehicle initial state
    initial_speed: float = 15.0  # m/s
    initial_distance: float = 0.0  # m

    # Optimization parameters
    ds: float = 10.0
    max_accel: float = 1.2
    max_decel: float = 1.5
    horizon_time: float = 180.0

    # Visualization
    save_plots: bool = True
    output_dir: str = "simulation_results"
    show_plots: bool = False


@dataclass
class TrafficLight:
    """Dynamic traffic light with state updates."""
    intersection_id: int
    distance: float  # Absolute distance from start (m)

    # Signal timing
    red_duration: float
    yellow_duration: float
    green_duration: float

    # Current state
    current_state: int = 0  # 0=Red, 1=Yellow, 2=Green
    time_in_state: float = 0.0  # Time elapsed in current state (s)

    def get_cycle_time(self) -> float:
        """Get total cycle time."""
        return self.red_duration + self.yellow_duration + self.green_duration

    def get_time_remaining(self) -> float:
        """Get remaining time in current state."""
        durations = [self.red_duration, self.yellow_duration, self.green_duration]
        return durations[self.current_state] - self.time_in_state

    def update(self, dt: float) -> None:
        """Update signal state based on time step."""
        self.time_in_state += dt

        durations = [self.red_duration, self.yellow_duration, self.green_duration]
        current_duration = durations[self.current_state]

        if self.time_in_state >= current_duration:
            # Transition to next state
            self.time_in_state -= current_duration
            self.current_state = (self.current_state + 1) % 3

    def get_signal_info_array(self, vehicle_distance: float) -> np.ndarray:
        """Get signal info in the format [Δd, tRemain, TrfState, red, yellow, green]."""
        delta_d = self.distance - vehicle_distance
        t_remain = self.get_time_remaining()

        return np.array([
            delta_d,
            t_remain,
            float(self.current_state),
            self.red_duration,
            self.yellow_duration,
            self.green_duration
        ])


@dataclass
class VehicleDynamics:
    """Simple vehicle dynamics model."""
    mass: float = 23000.0  # kg
    max_accel: float = 1.2  # m/s^2
    max_decel: float = 1.5  # m/s^2
    max_speed: float = 30.0  # m/s

    # State
    speed: float = 15.0  # m/s
    distance: float = 0.0  # m

    def update(self, target_speed: float, dt: float) -> Tuple[float, float]:
        """Update vehicle state based on target speed.

        Returns:
            (actual_accel, actual_speed)
        """
        # Compute required acceleration
        speed_error = target_speed - self.speed
        required_accel = speed_error / dt

        # Apply acceleration limits
        actual_accel = np.clip(required_accel, -self.max_decel, self.max_accel)

        # Update speed
        new_speed = self.speed + actual_accel * dt
        new_speed = np.clip(new_speed, 0.0, self.max_speed)

        # Update distance
        avg_speed = (self.speed + new_speed) / 2.0
        self.distance += avg_speed * dt
        self.speed = new_speed

        return actual_accel, self.speed


@dataclass
class SimulationRecord:
    """Record of simulation history."""
    times: List[float] = field(default_factory=list)
    distances: List[float] = field(default_factory=list)
    speeds: List[float] = field(default_factory=list)
    target_speeds: List[float] = field(default_factory=list)
    accelerations: List[float] = field(default_factory=list)
    fuel_rates: List[float] = field(default_factory=list)

    # RHC diagnostics
    rhc_costs: List[float] = field(default_factory=list)
    rhc_penalties: List[int] = field(default_factory=list)

    # Signal violations
    signal_states: List[Dict] = field(default_factory=list)

    def add_step(
        self,
        t: float,
        distance: float,
        speed: float,
        target_speed: float,
        accel: float,
        fuel_rate: float,
        rhc_cost: float = 0.0,
        rhc_penalty: int = 0,
        signal_state: Optional[Dict] = None
    ):
        """Add a simulation step."""
        self.times.append(t)
        self.distances.append(distance)
        self.speeds.append(speed)
        self.target_speeds.append(target_speed)
        self.accelerations.append(accel)
        self.fuel_rates.append(fuel_rate)
        self.rhc_costs.append(rhc_cost)
        self.rhc_penalties.append(rhc_penalty)
        self.signal_states.append(signal_state if signal_state else {})

    def to_arrays(self) -> Dict[str, np.ndarray]:
        """Convert to numpy arrays."""
        return {
            'time': np.array(self.times),
            'distance': np.array(self.distances),
            'speed': np.array(self.speeds),
            'target_speed': np.array(self.target_speeds),
            'acceleration': np.array(self.accelerations),
            'fuel_rate': np.array(self.fuel_rates),
            'rhc_cost': np.array(self.rhc_costs),
            'rhc_penalty': np.array(self.rhc_penalties),
        }

    def compute_metrics(self) -> Dict[str, float]:
        """Compute performance metrics."""
        data = self.to_arrays()

        total_fuel = np.trapz(data['fuel_rate'], data['time'])
        total_distance = data['distance'][-1] - data['distance'][0]
        avg_speed = np.mean(data['speed'])
        max_accel = np.max(data['acceleration'])
        min_accel = np.min(data['acceleration'])

        # Jerk (rate of change of acceleration)
        dt = np.diff(data['time'])
        jerk = np.diff(data['acceleration']) / dt
        rms_jerk = np.sqrt(np.mean(jerk**2))

        # Signal violations
        total_violations = sum(self.rhc_penalties)

        metrics = {
            'total_fuel': total_fuel,
            'total_distance': total_distance,
            'fuel_per_km': (total_fuel / total_distance * 1000) if total_distance > 0 else 0,
            'avg_speed': avg_speed,
            'max_accel': max_accel,
            'min_decel': abs(min_accel),
            'rms_jerk': rms_jerk,
            'total_violations': total_violations,
            'avg_cost': np.mean(data['rhc_cost']),
        }

        return metrics


class EcoDrivingSimulator:
    """Main simulation environment."""

    def __init__(self, config: SimulationConfig):
        """Initialize simulator.

        Parameters
        ----------
        config : SimulationConfig
            Simulation configuration
        """
        self.config = config
        self.vehicle = VehicleDynamics(
            max_accel=config.max_accel,
            max_decel=config.max_decel,
            speed=config.initial_speed,
            distance=config.initial_distance
        )
        self.traffic_lights: List[TrafficLight] = []
        self.record = SimulationRecord()

        # RHC controller (lazy initialization)
        self.controller = None
        self.route_segments = None

    def add_traffic_light(
        self,
        intersection_id: int,
        distance: float,
        red: float,
        yellow: float,
        green: float,
        initial_state: int = 0,
        initial_time: float = 0.0
    ):
        """Add a traffic light to the simulation."""
        light = TrafficLight(
            intersection_id=intersection_id,
            distance=distance,
            red_duration=red,
            yellow_duration=yellow,
            green_duration=green,
            current_state=initial_state,
            time_in_state=initial_time
        )
        self.traffic_lights.append(light)

    def setup_route(self, segments: List):
        """Setup route for RHC controller."""
        from rhc_eco_driving import (
            EcoDrivingController,
            OptimizationParams
        )

        params = OptimizationParams(
            ds=self.config.ds,
            max_accel=self.config.max_accel,
            max_decel=self.config.max_decel,
            horizon_time=self.config.horizon_time
        )

        self.controller = EcoDrivingController(params)
        self.controller.initialize_route(segments)
        self.route_segments = segments

    def get_visible_signals(self, max_range: float = 1000.0) -> List[np.ndarray]:
        """Get signals visible to the vehicle (within range).

        Returns list of signal info arrays in the format:
        [Δd, tRemain, TrfState, red, yellow, green]
        """
        from rhc_eco_driving import SignalInfo

        signals = []
        for light in self.traffic_lights:
            delta_d = light.distance - self.vehicle.distance

            # Only include signals ahead and within range
            if 0 < delta_d < max_range:
                info_array = light.get_signal_info_array(self.vehicle.distance)
                signals.append(SignalInfo.from_array(info_array))

        # Return up to 4 signals (closest first)
        signals.sort(key=lambda s: s.delta_distance)
        return signals[:4]

    def run(self) -> SimulationRecord:
        """Run the complete simulation.

        Returns
        -------
        SimulationRecord
            Simulation history and results
        """
        from rhc_eco_driving import VehicleState, fuel_prediction_model

        if self.controller is None:
            raise RuntimeError("Route not setup. Call setup_route() first.")

        print("=" * 70)
        print("Starting Eco-Driving Simulation")
        print("=" * 70)
        print(f"Total time: {self.config.total_time:.1f}s")
        print(f"Time step: {self.config.dt:.2f}s")
        print(f"Control interval: {self.config.control_interval:.1f}s")
        print(f"Traffic lights: {len(self.traffic_lights)}")
        print()

        t = 0.0
        time_since_control = 0.0
        target_speed = self.vehicle.speed

        step_count = 0
        control_count = 0

        while t < self.config.total_time:
            # Update traffic lights
            for light in self.traffic_lights:
                light.update(self.config.dt)

            # RHC control update
            if time_since_control >= self.config.control_interval:
                # Get vehicle state
                vehicle_state = VehicleState(
                    speed=self.vehicle.speed,
                    distance=self.vehicle.distance
                )

                # Get visible signals
                signals = self.get_visible_signals()

                # Run RHC optimization
                if signals:
                    v_set, speeds, times, diag = self.controller.step(vehicle_state, signals)
                    target_speed = v_set
                    rhc_cost = diag.get('cost', 0.0)
                    rhc_penalty = diag.get('penalties', 0)

                    control_count += 1
                    if control_count % 10 == 0:
                        print(f"t={t:.1f}s, d={self.vehicle.distance:.1f}m, "
                              f"v={self.vehicle.speed:.1f}m/s, v_set={target_speed:.1f}m/s, "
                              f"cost={rhc_cost:.6f}, violations={rhc_penalty}")
                else:
                    rhc_cost = 0.0
                    rhc_penalty = 0

                time_since_control = 0.0

            # Update vehicle dynamics
            accel, actual_speed = self.vehicle.update(target_speed, self.config.dt)

            # Compute fuel consumption
            # Find current road segment
            grade = 0.0
            for seg in self.route_segments:
                if self.vehicle.distance <= seg.distance:
                    grade = seg.grade
                    break

            fuel_rate = fuel_prediction_model(self.vehicle.speed, accel, grade)

            # Record step
            signal_state = {}
            for i, light in enumerate(self.traffic_lights):
                state_name = ['Red', 'Yellow', 'Green'][light.current_state]
                signal_state[f'signal_{i+1}'] = {
                    'state': state_name,
                    'distance': light.distance,
                    'time_remaining': light.get_time_remaining()
                }

            self.record.add_step(
                t=t,
                distance=self.vehicle.distance,
                speed=self.vehicle.speed,
                target_speed=target_speed,
                accel=accel,
                fuel_rate=fuel_rate,
                rhc_cost=rhc_cost if time_since_control == 0 else 0,
                rhc_penalty=rhc_penalty if time_since_control == 0 else 0,
                signal_state=signal_state
            )

            # Advance time
            t += self.config.dt
            time_since_control += self.config.dt
            step_count += 1

            # Check if route is complete
            if self.route_segments and self.vehicle.distance > self.route_segments[-1].distance + 50:
                print(f"\nRoute completed at t={t:.1f}s")
                break

        print("\n" + "=" * 70)
        print("Simulation Complete")
        print("=" * 70)
        print(f"Total steps: {step_count}")
        print(f"Total control updates: {control_count}")
        print(f"Final distance: {self.vehicle.distance:.1f}m")
        print(f"Final speed: {self.vehicle.speed:.1f}m/s")
        print()

        # Compute metrics
        metrics = self.record.compute_metrics()
        print("Performance Metrics:")
        print(f"  Total fuel: {metrics['total_fuel']:.6f}")
        print(f"  Total distance: {metrics['total_distance']:.1f}m")
        print(f"  Fuel per km: {metrics['fuel_per_km']:.6f}")
        print(f"  Average speed: {metrics['avg_speed']:.2f}m/s ({metrics['avg_speed']*3.6:.1f}km/h)")
        print(f"  Max acceleration: {metrics['max_accel']:.2f}m/s²")
        print(f"  Max deceleration: {metrics['min_decel']:.2f}m/s²")
        print(f"  RMS jerk: {metrics['rms_jerk']:.3f}m/s³")
        print(f"  Signal violations: {metrics['total_violations']}")
        print()

        return self.record

    def visualize_results(self, save: bool = True, show: bool = False):
        """Create comprehensive visualization of results."""
        import matplotlib.pyplot as plt
        from matplotlib.gridspec import GridSpec

        data = self.record.to_arrays()

        fig = plt.figure(figsize=(16, 12))
        gs = GridSpec(4, 2, figure=fig, hspace=0.3, wspace=0.25)

        # 1. Speed profile
        ax1 = fig.add_subplot(gs[0, :])
        ax1.plot(data['time'], data['speed'], 'b-', linewidth=2, label='Actual Speed')
        ax1.plot(data['time'], data['target_speed'], 'r--', linewidth=1.5,
                 alpha=0.7, label='Target Speed')
        ax1.set_xlabel('Time (s)', fontsize=11)
        ax1.set_ylabel('Speed (m/s)', fontsize=11)
        ax1.set_title('Speed Profile', fontsize=13, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=10)

        # Add km/h on right axis
        ax1_right = ax1.twinx()
        ax1_right.set_ylabel('Speed (km/h)', fontsize=11, color='gray')
        ax1_right.set_ylim(np.array(ax1.get_ylim()) * 3.6)
        ax1_right.tick_params(axis='y', labelcolor='gray')

        # 2. Distance vs Time with traffic lights
        ax2 = fig.add_subplot(gs[1, 0])
        ax2.plot(data['time'], data['distance'], 'g-', linewidth=2)

        # Plot traffic light locations
        for light in self.traffic_lights:
            ax2.axhline(y=light.distance, color='orange', linestyle=':',
                       alpha=0.6, linewidth=1.5)
            ax2.text(data['time'][-1] * 0.02, light.distance + 20,
                    f'TL{light.intersection_id}', fontsize=9, color='orange')

        ax2.set_xlabel('Time (s)', fontsize=11)
        ax2.set_ylabel('Distance (m)', fontsize=11)
        ax2.set_title('Position vs Time', fontsize=13, fontweight='bold')
        ax2.grid(True, alpha=0.3)

        # 3. Acceleration
        ax3 = fig.add_subplot(gs[1, 1])
        ax3.plot(data['time'], data['acceleration'], 'purple', linewidth=1.5)
        ax3.axhline(y=0, color='k', linestyle='-', linewidth=0.8, alpha=0.5)
        ax3.axhline(y=self.config.max_accel, color='r', linestyle='--',
                   linewidth=1, alpha=0.5, label='Accel limit')
        ax3.axhline(y=-self.config.max_decel, color='r', linestyle='--',
                   linewidth=1, alpha=0.5, label='Decel limit')
        ax3.set_xlabel('Time (s)', fontsize=11)
        ax3.set_ylabel('Acceleration (m/s²)', fontsize=11)
        ax3.set_title('Acceleration Profile', fontsize=13, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.legend(fontsize=9)

        # 4. Fuel consumption rate
        ax4 = fig.add_subplot(gs[2, 0])
        ax4.plot(data['time'], data['fuel_rate'], 'brown', linewidth=1.5)
        ax4.fill_between(data['time'], 0, data['fuel_rate'], alpha=0.3, color='brown')
        ax4.set_xlabel('Time (s)', fontsize=11)
        ax4.set_ylabel('Fuel Rate', fontsize=11)
        ax4.set_title('Fuel Consumption Rate', fontsize=13, fontweight='bold')
        ax4.grid(True, alpha=0.3)

        # 5. Cumulative fuel
        ax5 = fig.add_subplot(gs[2, 1])
        cumulative_fuel = np.cumsum(data['fuel_rate'] * np.diff(data['time'], prepend=0))
        ax5.plot(data['time'], cumulative_fuel, 'darkgreen', linewidth=2)
        ax5.set_xlabel('Time (s)', fontsize=11)
        ax5.set_ylabel('Cumulative Fuel', fontsize=11)
        ax5.set_title('Cumulative Fuel Consumption', fontsize=13, fontweight='bold')
        ax5.grid(True, alpha=0.3)

        # 6. RHC diagnostics
        ax6 = fig.add_subplot(gs[3, 0])
        non_zero_cost = data['rhc_cost'][data['rhc_cost'] > 0]
        non_zero_time = data['time'][data['rhc_cost'] > 0]
        if len(non_zero_cost) > 0:
            ax6.scatter(non_zero_time, non_zero_cost, c='red', s=20, alpha=0.6)
            ax6.plot(non_zero_time, non_zero_cost, 'r-', linewidth=1, alpha=0.3)
        ax6.set_xlabel('Time (s)', fontsize=11)
        ax6.set_ylabel('Optimization Cost', fontsize=11)
        ax6.set_title('RHC Optimization Cost', fontsize=13, fontweight='bold')
        ax6.grid(True, alpha=0.3)

        # 7. Signal violations
        ax7 = fig.add_subplot(gs[3, 1])
        non_zero_penalty = data['rhc_penalty'][data['rhc_penalty'] > 0]
        non_zero_penalty_time = data['time'][data['rhc_penalty'] > 0]
        if len(non_zero_penalty) > 0:
            ax7.scatter(non_zero_penalty_time, non_zero_penalty,
                       c='darkred', s=50, alpha=0.6, marker='x')
        ax7.set_xlabel('Time (s)', fontsize=11)
        ax7.set_ylabel('Violation Count', fontsize=11)
        ax7.set_title('Signal Violations', fontsize=13, fontweight='bold')
        ax7.grid(True, alpha=0.3)

        # Overall title
        metrics = self.record.compute_metrics()
        fig.suptitle(
            f'Eco-Driving Simulation Results\n'
            f'Fuel: {metrics["total_fuel"]:.6f} | '
            f'Distance: {metrics["total_distance"]:.0f}m | '
            f'Violations: {metrics["total_violations"]} | '
            f'Avg Speed: {metrics["avg_speed"]:.1f}m/s ({metrics["avg_speed"]*3.6:.1f}km/h)',
            fontsize=14,
            fontweight='bold',
            y=0.98
        )

        if save:
            output_dir = Path(self.config.output_dir)
            output_dir.mkdir(exist_ok=True)
            output_path = output_dir / "simulation_results.png"
            fig.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"Results saved to: {output_path}")

        if show:
            plt.show()
        else:
            plt.close(fig)

        return fig
