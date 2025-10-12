# -*- coding: utf-8 -*-
"""Demonstration script for :func:`plot_speed_profile`."""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rhc_pipeline import plot_speed_profile


def build_demo_plan() -> Tuple[np.ndarray, np.ndarray]:
    """Construct a smooth demo trajectory for visualisation."""
    times = np.linspace(0.0, 60.0, 13)
    speeds = 12.0 + 3.5 * np.sin(times / 8.0) * np.exp(-times / 90.0)
    return times, speeds


def main(output: Path | None = None) -> None:
    """Plot the demo speed profile and optionally save it to disk."""
    times, speeds = build_demo_plan()
    fig, _ = plot_speed_profile(times, speeds, label="Demo trajectory")
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=150)
    else:
        fig.show()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Visualise a demo speed profile.")
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional path to write the generated plot as a PNG image.",
    )
    args = parser.parse_args()
    main(args.output)
