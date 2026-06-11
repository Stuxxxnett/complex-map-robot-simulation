# Complex Map Robot Navigation Simulation

A complete autonomous robot navigation pipeline implemented in MATLAB, featuring binary occupancy grid mapping, probabilistic path planning (PRM, RRT, RRT*), Pure Pursuit trajectory tracking, and real-time differential drive robot simulation with video output.

---

## System Pipeline

![Pipeline Diagram](pipeline_diagram.png)

---

## Project Overview

This project simulates a mobile robot navigating through a complex maze-like environment with obstacles. It covers the full autonomy stack — from raw map ingestion to real-time simulation and quantitative analysis.

```
Raw Map → Obstacle Inflation → Path Planning → Trajectory Generation → Robot Simulation → Analysis
```

| Stage              | Method                | MATLAB Tool             |
|--------------------|----------------------|-------------------------|
| Map Creation       | Binary Occupancy Grid | `binaryOccupancyMap`    |
| Obstacle Inflation | Minkowski Sum         | `inflate()`             |
| Path Planning      | PRM                   | `mobileRobotPRM`        |
| Path Planning      | RRT                   | `plannerRRT`            |
| Path Planning      | RRT*                  | `plannerRRTStar`        |
| Motion Control     | Pure Pursuit          | `controllerPurePursuit` |
| Robot Model        | Differential Drive    | Euler integration       |
| Output             | Video + Plots         | `VideoWriter`           |

---

## Repository Structure

```
complex-map-robot-simulation/
│
├── ComplexMapSimulation.mlx        ← MATLAB Live Script (recommended)
├── ComplexMapSimulation.m          ← Plain MATLAB script
│
├── 01_occupancy_grid.png           ← Raw binary occupancy map
├── 05_simulation_final.png         ← Planned vs actual trajectory overlay
├── pipeline_diagram.png            ← System architecture diagram
│
├── ALGORITHM_NOTES.md              ← Explanation of PRM, RRT, RRT*
├── RESULTS_ANALYSIS.md             ← Quantitative analysis and writeup
├── LICENSE
└── README.md
```

> **Note:** Output figures and the simulation video (`robot_simulation.avi`) are generated locally when you run the script. See [How to Run](#how-to-run) below.

---

## Requirements

| Requirement             | Version                              |
|-------------------------|--------------------------------------|
| MATLAB                  | R2020b or later (R2024a recommended) |
| Robotics System Toolbox | Required                             |
| Navigation Toolbox      | Required                             |

---

## How to Run

1. Clone or download this repository
2. Open MATLAB and navigate to the repository folder
3. Open `ComplexMapSimulation.mlx` (recommended) or `ComplexMapSimulation.m`
4. Click **Run** or press `F5`
5. The simulation will:
   - Display all intermediate plots inline
   - Run the robot simulation in real time
   - Save `robot_simulation.avi` to your current directory
6. All output figures can be saved individually using `saveas()` or MATLAB's figure export

> Open `ComplexMapSimulation.mlx` (Live Script) for the best experience — outputs render inline alongside the code.

---

## How It Works

### Step 1 — Occupancy Grid Map

Loads MATLAB's built-in `complexMap` and converts it to a `binaryOccupancyMap` at 2 cells/meter resolution.

![Occupancy Grid](01_occupancy_grid.png)

### Step 2 — Map Inflation

Inflates obstacles by the robot radius (0.5 m) so that path planning treats the robot as a point, guaranteeing collision-free clearance via Minkowski sum expansion.

### Step 3 — PRM Path Planning

Uses a Probabilistic Roadmap (PRM) with 2000 nodes and a connection distance of 5 m. If no path is found, retries with 5000 nodes and distance 8 m.

- **Start:** `[2, 2]` meters
- **Goal:** `[24, 18]` meters

### Step 4 — Waypoint Trajectory

Converts the PRM path into a full pose trajectory `[x, y, θ]` by computing heading angles between consecutive waypoints. Travel time is estimated at 0.5 m/s.

### Step 5 — Robot Simulation and Video

Runs a Pure Pursuit controller on a differential drive robot model:

```
x(t+dt)  = x(t)  + v · cos(θ) · dt
y(t+dt)  = y(t)  + v · sin(θ) · dt
θ(t+dt)  = θ(t)  + ω · dt
```

Every frame is captured and written to `robot_simulation.avi`.

### Step 6 — Trajectory Analysis

Generates position, heading, and distance-to-goal profiles. A metrics report is printed to the console at the end of each run:

```
============ FINAL ANALYSIS ============
Planned path length:    31.75 meters
Actual travel length:   30.25 meters
Total time simulated:   60.60 seconds
Average speed:          0.50 m/s
Path efficiency:        105.0%
Goal position:          [24.0, 18.0]
Final robot position:   [24.01, 17.51]
Position error:         0.49 meters
========================================
```

The robot successfully reaches the goal with a terminal position error of 0.49 m, well within the 0.5 m goal acceptance radius.

### Bonus — RRT and RRT* Comparison

Runs both RRT and RRT* planners on the same map for side-by-side comparison against PRM. RRT* found a path of **33.91 meters** using **24 waypoints**.

---

## Sample Output

### Occupancy Grid and Path Planning

| Raw Occupancy Grid | Inflated Map (robot radius = 0.5 m) |
|---|---|
| ![Occupancy Grid](01_occupancy_grid.png) | *(generated on run)* |

| PRM Path | RRT* Path |
|---|---|
| *(generated on run)* | *(generated on run)* |

### Simulation Result

![Simulation Final](05_simulation_final.png)

*Planned trajectory (blue) vs actual robot path overlay on the inflated map.*

---

## Tunable Parameters

| Parameter                    | Location | Default     | Effect                    |
|------------------------------|----------|-------------|---------------------------|
| `robotRadius`                | Step 2   | `0.5` m     | Inflation clearance        |
| `planner.NumNodes`           | Step 3   | `2000`      | PRM coverage               |
| `planner.ConnectionDistance` | Step 3   | `5` m       | PRM edge length            |
| `DesiredLinearVelocity`      | Step 5   | `0.5` m/s   | Robot speed                |
| `MaxAngularVelocity`         | Step 5   | `1.0` rad/s | Turn rate limit            |
| `LookaheadDistance`          | Step 5   | `1.5` m     | Pure Pursuit look-ahead    |
| `sampleTime`                 | Step 5   | `0.1` s     | Simulation timestep        |
| `goalRadius`                 | Step 5   | `0.5` m     | Goal acceptance radius     |

---

## Concepts Covered

- Binary occupancy grid mapping
- Configuration space obstacle inflation (Minkowski sum)
- Sampling-based motion planning: PRM, RRT, RRT*
- Pure Pursuit path tracking
- Differential drive kinematics (Euler integration)
- Trajectory analysis and quantitative metrics
- Real-time visualization and video recording in MATLAB

---

## Author

**Ishaan Jha**  
B.Tech Mechatronics Engineering — IIIT Bhagalpur  
Skills: MATLAB · Robotics · Path Planning · Autonomous Navigation

---

## License

MIT License — see [LICENSE](LICENSE) for details.
