# Engineering Logbook — GlassSync

## Phase 1 — AAS + SCL Integration

Date: 2026-08-21
Video: [Watch on YouTube](https://youtu.be/6XjenCZ8NH8)

![Phase 1 — AAS + SCL Integration](docs/images/Phase-1—AAS+SCL-Integration.png.jpg)

### What Was Completed

1. Created AAS for ABB IRB 4600 robot
2. Created KinematicsControl Submodel
3. Created OPCUA_Mapping Submodel
4. Written SCL code for robot control
5. Defined ABB-IRB-4600-DB01 Data Block

### Files Created / Updated

aas/GlassSync.json
plc/scl/GlassSync.scl
plc/db/GlassSync.db

### Code Details

Inputs (from HMI/SCADA):
- EnableAll, EnableKinematics, Auto_Mode
- Vlcty (velocity)
- HomePos, UpTheLinePos, PickUp1/2/3, Pos4Maint
- Go2Pos4Maint, Go2HomePos, Stacking2Left/Right
- AccessControl

Outputs (from PLC):
- ActPos (actual TCP position)
- Group_Fault, Axis1-4_Fault
- Dynamic_Speed, Dynamic_Torque, Axis_Power
- Axis_Limits
- Motion_Active, Emergency_Stop_Active, Error_Code

### Notes

- PD_TEL3 data (RobotDB_A1..A4) was excluded from AAS because it is complex. A dedicated Submodel will be added later.

### Next Phase

Phase 2: Python OPC UA client + AAS integration

---

## Phase 2 — Python + CoppeliaSim Integration ✅

Date: 2026-08-23

Video: [Watch on YouTube](https://youtu.be/6XjenCZ8NH8)

![Phase 2 — Python + CoppeliaSim Integration](docs/images/Phase-2—Python-CoppeliaSim-Integration.jpg)

### What Was Completed

1. Integrated CoppeliaSim (ABB-IRB-140) as the motion simulation environment
2. Matched DH parameters across all three layers:
   - Siemens PLC (kinematic transformation matrices)
   - Python (ikpy library for inverse kinematics)
   - CoppeliaSim (ABB-IRB-140 model with corrected URDF)
3. Implemented Python-based IK solver as the digital twin brain
4. Redesigned trajectory waypoints for the new workspace
5. Validated motion sequences in simulation

### Files Created / Updated

python/src/GlassSync.py
plc/scl/GlassSync.scl

### Key Technical Decisions

**1. Python IK Over CoppeliaSim SimIK**
- **Rationale**: Using Python (ikpy) as the primary IK solver provides greater flexibility
- **Benefits**:
  - Not tied to CoppeliaSim's proprietary SimIK
  - Easier to switch between different simulation environments
  - Better integration with ROS2 later (same reasoning as using ROS2)
  - Full control over kinematic algorithms and optimization
  - Easier debugging and parameter tuning

**2. DH Parameter Matching**
- Validated consistency across all three systems:
  - PLC: Transformation matrices in SCL
  - Python: DH parameters in ikpy
  - CoppeliaSim: URDF model parameters
- Verified with test points to ensure identical joint angle outputs

**3. Robot Selection: ABB-IRB-140**
- Switched from ABB-IRB-4600 to ABB-IRB-140 due to:
  - Clear and well-documented URDF parameters
  - Correct DH parameter matching
  - Better suited for the workspace requirements
- **Lesson Learned**: Significant time was lost debugging ABB-IRB-4600 URDF inconsistencies in CoppeliaSim

**4. Trajectory Redesign**
- New waypoints adapted to ABB-IRB-140 workspace:
  - HomePos: [0.3, 0.0, 0.5, 0.0] (X, Y, Z, A)
  - PickUp1: [0.4, -0.3, 0.3, 0.0]
  - PickUp2: [0.4, 0.0, 0.3, 0.0]
  - PickUp3: [0.4, 0.3, 0.3, 0.0]
  - StackingLeft: [0.2, -0.4, 0.2, 0.0]
  - StackingRight: [0.2, 0.4, 0.2, 0.0]
- Validated all waypoints for reachability and collision avoidance

### Architecture Update

**Current Data Flow:**
PLC (WCS Targets) → Python (IK Solver) → CoppeliaSim (Simulation)


**Planned Architecture for Phase 3:**
PLC (WCS Targets) → PLC-Driver.py (OPC-UA) → Main.py (Orchestrator) →
├── IK Solver → CoppeliaSim
├── MQTT Publisher → TimescaleDB
└── AAS Update → Digital Twin


### Challenges & Solutions

**1. Robot URDF Parameter Mismatch**
- **Challenge**: ABB-IRB-4600 in CoppeliaSim had inconsistent DH parameters
- **Impact**: Wasted significant time on debugging and validation
- **Solution**: Switched to ABB-IRB-140 with verified URDF and clear documentation
- **Lesson**: Always validate URDF parameters against manufacturer datasheets before integration

**2. DH Parameter Alignment**
- **Challenge**: Ensuring identical kinematic parameters across PLC, Python, and CoppeliaSim
- **Solution**: Created a single source of truth (`dh_parameters.py`) with validation tests
- **Verification**: Tested multiple random WCS points and compared joint angles

**3. Workspace Adaptation**
- **Challenge**: ABB-IRB-140 has different reach and workspace compared to IRB-4600
- **Solution**: Redesigned all waypoints using robot's valid workspace envelope
- **Validation**: Used CoppeliaSim collision detection to verify all trajectories

**4. Coordinate System Consistency**
- **Challenge**: Different coordinate conventions across systems (WCS, joint space, simulation)
- **Solution**: Established clear transformation pipeline:
  - WCS (world) → IK (joint angles) → CoppeliaSim (simulation space)
  - Used consistent units (mm, degrees)

### Next Phase

**Phase 3: OPC-UA Separation & Main Orchestrator**

Planned Tasks:
- Separate PLC communication into dedicated `PLC-Driver.py`
- Implement `Main.py` as central orchestrator
- Connect all modules: PLC-Driver → Main → IK/Simulation/MQTT/AAS
- Ensure modular and decoupled architecture

### Updated System Architecture
┌─────────────────────────────────────────────────────────────┐
│ Siemens PLC (S7-1500) │
│ • SCL Control Logic │
│ • OPC UA Server │
└────────────────────┬────────────────────────────────────────┘
│ OPC UA
▼
┌─────────────────────────────────────────────────────────────┐
│ Python Middleware Layer │
│ ┌──────────────────────────────────────────────────────┐ │
│ │ PLC-Driver.py (OPC UA Client - Phase 3) │ │
│ └──────────────────┬───────────────────────────────────┘ │
│ │ │
│ ┌──────────────────▼───────────────────────────────────┐ │
│ │ Main.py (Orchestrator - Phase 3) │ │
│ └──────┬──────────────────────────────┬────────────────┘ │
│ │ │ │
│ ▼ ▼ │
│ ┌─────────────────┐ ┌─────────────────┐ │
│ │ IK Solver (ikpy)│ │ AAS Server │ │
│ └────────┬────────┘ └─────────────────┘ │
│ │ │
│ ▼ │
│ ┌─────────────────┐ ┌─────────────────┐ │
│ │ CoppeliaSim │ │ MQTT Publisher │ │
│ │ Bridge │ └────────┬────────┘ │
│ └─────────────────┘ │ │
└───────────────────────────────────────────┼──────────────────┘
│
▼
┌─────────────────┐
│ TimescaleDB │
│ (InfluxDB) │
└─────────────────┘


### Files Modified

- Updated trajectory waypoints for ABB-IRB-140
- Adjusted DH parameters for new robot
- Modified kinematics validation tests

---

Last Updated: 2026-08-23
Author: Nebras