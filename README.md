# 🏭 GlassSync - Digital Twin Platform

> Full Stack: Siemens PLC → Python → CoppeliaSim → ROS2/MoveIt2

## 📌 About The Project

GlassSync is a hardware-in-the-loop (HIL) digital twin for a 6-robot glass factory, connecting Siemens S7-1500 PLC, CoppeliaSim simulation, ROS2 + MoveIt2 for motion planning, and Python middleware for OPC UA, ZMQ, and MQTT bridges.

Current Focus: Phase 2 completed - Python IK solver with CoppeliaSim integration (ABB-IRB-140).

## 🎬 Video Documentation

| # | Phase   |                  Topic                |    Date    |               Video                      |    Status     |
|---|---------|---------------------------------------|------------|------------------------------------------|---------------|
| 1 | Phase 1 | AAS + SCL Integration                 | 2026-08-21 | [▶️ Watch](https://youtu.be/6XjenCZ8NH8) | ✅ Completed |
| 2 | Phase 2 | CoppeliaSim + Python IK Integration   | 2026-08-23 | [▶️ Watch](https://youtu.be/5Lwg1ikyXEQ) | ✅ Completed |
| 3 | Phase 3 | OPC UA Separation + Main Orchestrator | TBD        | [▶️ -----]()                             | ⏳ Planned    |
| 4 | Phase 4 | MQTT + Messaging Layer                | TBD        | [▶️ -----]()                             | ⏳ Planned    |
| 5 | Phase 5 | TimescaleDB + Storage Layer           | TBD        | [▶️ -----]()                             | ⏳ Planned    |
| 6 | Phase 6 | ROS2 + CoppeliaSim Integration        | TBD        | [▶️ -----]()                             | ⏳ Planned    |
| 7 | Phase 7 | Full Stack Integration + Analytics    | TBD        | [▶️ -----]()                             | ⏳ Planned    |

## 🏗️ System Architecture

![GlassSync System Architecture](docs/images/System_Architecture.png)

*Figure 1: GlassSync System Architecture — Siemens PLC ↔ Python Middleware ↔ CoppeliaSim ↔ ROS2/MoveIt2*

**Updated Architecture (After Phase 2):**
<img width="642" height="802" alt="2026-08-24_092420" src="https://github.com/user-attachments/assets/fd922364-f702-4e7d-94c6-cc2f1b8de527" />



## 🔧 Tech Stack

| Layer         | Technology                        |         Role            |
|---------------|-----------------------------------|-------------------------|
| PLC           | Siemens S7-1500T + TIA Portal V17 | Real-time control       |
| Communication | OPC UA, ZMQ, PROFINET, MQTT       | Data exchange           |
| Simulation    | CoppeliaSim 4.6+                  | Robot environment       |
| IK Solver     | Python (ikpy)                     | Digital twin kinematics |
| Robotics      | ROS2 Humble + MoveIt2             | Motion planning         |
| Middleware    | Python 3.12+ (Asyncio)            | Bridge services         |
| AAS           | IDTA (Asset Administration Shell) | Digital twin            |
| Database      | TimescaleDB                       | Time-series storage     |
| Visualization | Grafana                           | Dashboards              |
| SCADA         | Ignition (planned)                | HMI                     |
| Orchestration | Docker, Docker Compose            | Infrastructure          |

## 📁 Project Structure
<img width="472" height="717" alt="2026-08-24_092445" src="https://github.com/user-attachments/assets/5d3ff14c-d40a-45fd-b36e-d9b24d46de50" />


## ✅ Phase 1 — AAS + SCL Integration

Date : 2026-08-21
Video: https://youtu.be/6XjenCZ8NH8
Tag  : [Phase 1 Release Notes](https://github.com/Nebras4u/GlassSync/releases/tag/phase-1-aas-scl)

### What Was Done?

| # | Task                        | Status   | Files                |
|---|-----------------------------|----------|----------------------|
| 1 | Create AAS for ABB IRB 4600 | ✅ Done | aas/GlassSync.json    |
| 2 | Write SCL control logic     | ✅ Done | plc/scl/GlassSync.scl |
| 3 | Define Data Block structure | ✅ Done | plc/db/GlassSync.db   |

### Key Features

Inputs (from HMI/SCADA):
- EnableAll, EnableKinematics, Auto_Mode
- Vlcty (velocity - mm/s)
- Positions: HomePos, UpTheLinePos, PickUp1/2/3, Pos4Maint
- Triggers: Go2Pos4Maint, Go2HomePos, Stacking2Left/Right
- Stacking Points: 6 points each for Left/Right
- AccessControl (0=ReadOnly, 1=Operator, 2=Maintenance)

Outputs (Feedback from PLC):
- ActPos (actual TCP position)
- Faults: Group_Fault, Axis1-4_Fault
- Dynamic_Speed, Dynamic_Torque, Axis_Power
- Axis_Limits (Min/Max per axis)
- Status: Motion_Active, Emergency_Stop_Active, Error_Code

## ✅ Phase 2 — CoppeliaSim + Python IK Integration

Date : 2026-08-23
Video: Upcoming
Tag  : [Phase 2 Release Notes](https://github.com/Nebras4u/GlassSync/releases/tag/Phase-2-CoppeliaSim-Python-IK-Integration)

### What Was Done?

| # | Task                                               | Status  | Files                             |
|---|----------------------------------------------------|---------|-----------------------------------|
| 1 | ABB-IRB-140 CoppeliaSim integration                | ✅ Done | python/src/coppelia_sim_bridge.py |
| 2 | Python IK solver implementation                    | ✅ Done | python/src/ik_solver.py           |
| 3 | DH parameter matching (PLC ↔ Python ↔ CoppeliaSim) | ✅ Done | python/src/dh_parameters.py       |
| 4 | Trajectory redesign for new workspace              | ✅ Done | python/src/trajectory_planner.py  |
| 5 | IK validation tests                                | ✅ Done | python/src/test_ik_validation.py  |
| 6 | Robot configuration file                           | ✅ Done | python/config/robot_config.yaml   |

### Key Technical Decisions

**1. Python IK Over CoppeliaSim SimIK**
- **Decision**: Use Python (ikpy) as primary IK solver instead of CoppeliaSim's SimIK
- **Rationale**:
  - Not tied to CoppeliaSim's proprietary API
  - Easier to switch between simulation environments (Gazebo, Webots, etc.)
  - Better integration with ROS2 (same reasoning applies)
  - Full control over kinematic algorithms and optimization parameters
  - Easier debugging and testing
  - Consistent with future ROS2 integration strategy

**2. DH Parameter Matching**
- Verified consistency across all three systems:
  - Siemens PLC: Transformation matrices in SCL
  - Python (ikpy): DH parameters in ikpy
  - CoppeliaSim: URDF model parameters
- Validated with multiple random WCS points
- Achieved identical joint angle outputs across all systems

**3. Robot Selection: ABB-IRB-140**
- Switched from ABB-IRB-4600 to ABB-IRB-140 due to:
  - Clear and well-documented URDF parameters
  - Correct DH parameter matching verified
  - Better suited for the workspace requirements
- **Critical Lesson**: Always validate URDF parameters before integration - significant time was lost debugging ABB-IRB-4600 inconsistencies

**4. New Trajectory Waypoints**

| Waypoint      | Position (X, Y, Z, A) | Description             |
|---------------|-----------------------|-------------------------|
| HomePos       | [0.3, 0.0, 0.5, 0.0]  | Home position           |
| PickUp1       | [0.4, -0.3, 0.3, 0.0] | Pickup point 1          |
| PickUp2       | [0.4, 0.0, 0.3, 0.0]  | Pickup point 2          |
| PickUp3       | [0.4, 0.3, 0.3, 0.0]  | Pickup point 3          |
| StackingLeft  | [0.2, -0.4, 0.2, 0.0] | Left stacking position  |
| StackingRight | [0.2, 0.4, 0.2, 0.0]  | Right stacking position |
| UpTheLinePos  | [0.5, 0.0, 0.4, 0.0]  | Up-the-line position    |

### Technical Challenges & Solutions

| Challenge                           | Impact                    | Solution                                             |
|-------------------------------------|---------------------------|------------------------------------------------------|
| **ABB-IRB-4600 URDF Inconsistency** | Wasted significant time   | Switched to ABB-IRB-140 with verified URDF           |
| **DH Parameter Mismatch**           | Inconsistent joint angles | Created single source of truth in `dh_parameters.py` |
| **Workspace Adaptation**            | Waypoints out of reach    | Redesigned trajectory for IRB-140 workspace          |
| **Coordinate System Differences**   | Coordinate confusion      | Established clear WCS → Joint → Simulation pipeline  |

### Next Steps

**Phase 3: OPC-UA Separation & Main Orchestrator**

Planned Tasks:
- Separate PLC communication into dedicated `PLC-Driver.py`
- Implement `Main.py` as central orchestrator
- Connect all modules: PLC-Driver → Main → IK/Simulation/MQTT/AAS
- Ensure modular and decoupled architecture

### Future Phases

- Phase 4: MQTT + Messaging Layer
- Phase 5: TimescaleDB + Storage Layer
- Phase 6: ROS2 + CoppeliaSim Integration
- Phase 7: Full Stack Integration + Analytics

## 🚀 Getting Started (Phase 2)

### Prerequisites
- TIA Portal V17 (for PLC development)
- PLCSim Advanced V6 (for simulation)
- Python 3.12+ with required packages:
  ```bash
  pip install numpy ikpy zerorpc pymodbus

* CoppeliaSim 4.6+ (with ABB-IRB-140 model)
* Git (for version control)

Running the Python IK Solver
# Test
python python/src/GlassSync.py

## 📚 Documentation

- 📘 Engineering Logbook: [docs/Engineering_Logbook.md](docs/Engineering_Logbook.md)
- 🏷️ Phase 1 Details: [Phase 1 Release Notes](https://github.com/Nebras4u/GlassSync/releases/tag/phase-1-aas-scl)
- 🏷️ Phase 2 Details: [Phase 2 Release Notes](https://github.com/Nebras4u/GlassSync/releases/tag/Phase-2-CoppeliaSim-Python-IK-Integration)


📬 Contact
GitHub: https://github.com/Nebras4u
YouTube: https://www.youtube.com/playlist?list=PLMadf0IBbtAE

Built for Industry 4.0 — GlassSync © 2026
