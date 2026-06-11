# 🤖 Complex Map Robot Navigation Simulation

A complete **autonomous robot navigation pipeline** in MATLAB — occupancy grid mapping, path planning with PRM, RRT and RRT*, Pure Pursuit trajectory tracking, and real-time differential drive robot simulation with video output.

---

## 📽️ Simulation Demo

> ▶️ **[Watch on YouTube — Differential Drive Robot Simulation](YOUR_YOUTUBE_LINK_HERE)**

[![Watch the simulation](https://img.youtube.com/vi/YOUR_VIDEO_ID/maxresdefault.jpg)](YOUR_YOUTUBE_LINK_HERE)

> After uploading to YouTube, replace `YOUR_YOUTUBE_LINK_HERE` with the full URL and `YOUR_VIDEO_ID` with just the ID from the URL (the part after `?v=`).

---

## 🔧 System Pipeline

![Pipeline](results/assets/pipeline_diagram.png)

---

## 🗺️ How It Works

```
Raw Map Load → Obstacle Inflation → PRM / RRT Path Plan → Waypoint Trajectory → Pure Pursuit Control → Differential Drive Sim → Analysis & Output
```

| Stage | Method | MATLAB Tool |
|---|---|---|
| Map Creation | Binary Occupancy Grid | `binaryOccupancyMap` |
| Obstacle Inflation | Minkowski Sum | `inflate()` |
| Path Planning | PRM | `mobileRobotPRM` |
| Path Planning | RRT | `plannerRRT` |
| Path Planning | RRT* | `plannerRRTStar` |
| Motion Control | Pure Pursuit | `controllerPurePursuit` |
| Robot Model | Differential Drive | Euler integration |
| Output | Video + Plots | `VideoWriter` |

---

## 📊 Sample Outputs

**Occupancy Grid & Inflated Map**

| Raw Occupancy Grid | Inflated Map (robot radius = 0.5 m) |
|:-:|:-:|
| ![Occupancy Grid](results/plots/01_occupancy_grid.png) | ![Inflated Map](results/plots/02_inflated_map.png) |

**Path Planning**

| PRM Path | RRT* Path |
|:-:|:-:|
| ![PRM](results/plots/03_prm_path.png) | ![RRT*](results/plots/09_rrtstar_path.png) |

**Trajectory & Control**

| Waypoint Headings | Final: Planned vs Actual |
|:-:|:-:|
| ![Headings](results/plots/04_waypoint_headings.png) | ![Final](results/plots/05_simulation_final.png) |

**Analysis Plots**

| X/Y Position over Time | Distance to Goal |
|:-:|:-:|
| ![Position](results/plots/06_position_profile.png) | ![Distance](results/plots/08_distance_to_goal.png) |

---

## ⚙️ Requirements

| Requirement | Version |
|---|---|
| MATLAB | R2020b or later (R2024a recommended) |
| Robotics System Toolbox | Required |
| Navigation Toolbox | Required |

---

## 🚀 How to Run

1. Clone or download this repository
2. Open MATLAB and navigate to the repo folder
3. Open `ComplexMapSimulation.mlx` (Live Script — recommended) or `ComplexMapSimulation.m`
4. Click **Run** (or press `F5`)
5. The simulation will display all plots, run the robot in real time, and save `robot_simulation.avi`

---

## 📁 Repository Structure

```
complex-map-robot-simulation/
│
├── ComplexMapSimulation.mlx        ← MATLAB Live Script (main file)
├── ComplexMapSimulation.m          ← Plain MATLAB script (equivalent)
│
├── results/
│   └── plots/                      ← All simulation output figures
│       ├── 01_occupancy_grid.png
│       ├── 02_inflated_map.png
│       ├── 03_prm_path.png
│       ├── 04_waypoint_headings.png
│       ├── 05_simulation_final.png
│       ├── 06_position_profile.png
│       ├── 07_heading_profile.png
│       ├── 08_distance_to_goal.png
│       └── 09_rrtstar_path.png
│
├── assets/
│   └── pipeline_diagram.png        ← System architecture diagram
│
├── README.md
├── LICENSE
├── CHANGELOG.md
└── .gitignore
```

---

## 🔧 Key Parameters

| Parameter | Default | Effect |
|---|---|---|
| `robotRadius` | 0.5 m | Inflation clearance |
| `planner.NumNodes` | 2000 | PRM coverage |
| `planner.ConnectionDistance` | 5 m | PRM edge length |
| `DesiredLinearVelocity` | 0.5 m/s | Robot speed |
| `MaxAngularVelocity` | 1.0 rad/s | Turn rate limit |
| `LookaheadDistance` | 1.5 m | Pure Pursuit lookahead |
| `sampleTime` | 0.1 s | Simulation timestep |
| `goalRadius` | 0.5 m | Goal acceptance radius |

---

## 📈 Concepts Covered

- Binary occupancy grid mapping
- Configuration space obstacle inflation
- Sampling-based motion planning (PRM, RRT, RRT*)
- Pure Pursuit path tracking
- Differential drive kinematics
- Trajectory analysis and metrics
- Real-time visualization and video recording

---

## 👤 Author

**Ishaan Jha**  
B.Tech Mechatronics Engineering — IIIT Bhagalpur  
*MATLAB · Robotics · Path Planning · Autonomous Navigation*

---

## 📄 License

MIT License — see [LICENSE](LICENSE) for details.
