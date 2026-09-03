# RoboWeave

A ROS2-based multi-AMR (Autonomous Mobile Robot) fleet coordination system with a live SCADA-style monitoring dashboard.

## Structure

- **`src/amr_coordination2/`** — ROS2 package handling robot coordination, including collision watchdog logic and per-robot node behavior (`amr_node.py`).
- **`scada_dashboard/`** — Dashboard components for visualizing fleet state, including floorplan and chart rendering (`ui_charts.py`, `bridge.py`, `cad_parser.py`).
- **`launch/`** — ROS2 launch files to bring up the fleet (`fleet.launch.py`).
- **`resource/`**, **`test/`** — Standard ROS2 package resources and tests.
- **`benchmark.py`** — Benchmarking script for evaluating coordination performance.

## Setup

```bash
pip install -r requirements.txt
```

Refer to `package.xml` and `setup.cfg` for ROS2-specific dependency and build configuration.

## Status

Actively under development.