# Roboweave: Decentralized AMR Space-Time Arbitration

A fully decentralized, peer-to-peer autonomous mobile robot (AMR) coordination platform built for **SIH 2026**. Roboweave eliminates centralized routing servers entirely, using edge computing and a Data Distribution Service (DDS) mesh to guarantee zero inter-robot collisions and a minimum **20% reduction** in task completion time.

---

## Table of Contents

- [System Architecture](#system-architecture)
- [Core Capabilities](#core-capabilities)
- [Installation & Setup](#installation--setup)
- [Execution](#execution)
- [Project Structure](#project-structure)
- [License](#license)

---

## System Architecture

| Component | Technology | Purpose |
| :--- | :--- | :--- |
| **Middleware** | ROS 2 Humble | Core OS environment handling node lifecycles |
| **Network** | DDS Mesh | P2P token broadcasting for coordinate claims |
| **Logic** | Python 3 | Edge-based Space-Time A* and Kinematic Gate processing |
| **Dashboard** | Streamlit | UI for WMS payload dispatch and live telemetry observation |

---

## Core Capabilities

- **Serverless Pathfinding** — Each AMR executes an independent Space-Time A* planner (`R² × T`), calculating optimal routes locally based on shared mesh state.
- **Zero-Trust Kinematic Gate** — A physical failsafe that forces lower-priority robots to yield right-of-way before entering a contested cell.
- **Tier-2 Escape Routing** — Automated deadlock resolution where blocked robots scan adjacent cells and park in yield zones without requiring server intervention.
- **Independent Watchdog** — An isolated mathematical observer (`collision_watchdog.py`) that strictly monitors the mesh to prove zero physical coordinate overlaps during runtime.

---

## Installation & Setup

### Prerequisites

- Ubuntu 22.04 (or compatible) with **ROS 2 Humble** installed
- Python 3.10+
- `colcon` build tools

### 1. Clone the Repository

```bash
git clone https://github.com/shimsha22/roboweave.git
cd roboweave
```

### 2. Build the ROS 2 Workspace

```bash
colcon build
source install/setup.bash
```

### 3. Install Python Dependencies

```bash
pip install streamlit plotly
```

---

## Execution

Launch each component in its **own terminal**, sourced with the ROS 2 environment (`source install/setup.bash`).

### 1. Launch the Independent Watchdog (run first)

```bash
python3 src/amr_coordination2/amr_coordination2/collision_watchdog.py
```

### 2. Launch the Edge Nodes (AMR Fleet)

```bash
ros2 run amr_coordination2 amr_node --ros-args -p amr_id:=AMR_1
ros2 run amr_coordination2 amr_node --ros-args -p amr_id:=AMR_2
ros2 run amr_coordination2 amr_node --ros-args -p amr_id:=AMR_3
```

### 3. Launch the SCADA Dashboard

```bash
streamlit run scada_dashboard/dashboard.py
```

> **Note:** Start the watchdog before the AMR fleet so collision monitoring is active from the first coordinate claim.

---

## Project Structure

```
roboweave/
├── src/
│   └── amr_coordination2/
│       └── amr_coordination2/
│           ├── amr_node.py
│           └── collision_watchdog.py
├── scada_dashboard/
│   └── dashboard.py
├── install/
├── build/
└── README.md
```
