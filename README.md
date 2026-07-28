# 🏭 GlassSync
*Industrial Digital Twin Platform — Siemens S7-1500 PLC & CoppeliaSim*
[![Python 3.12+](https://img.shields.io/badge/Python-3.12+-blue.svg)]() [![PLC S7-1500T](https://img.shields.io/badge/PLC-S7--1500T-orange.svg)]() [![CoppeliaSim 4.6+](https://img.shields.io/badge/CoppeliaSim-4.6+-green.svg)]() [![ROS2 (Future)](https://img.shields.io/badge/ROS2-Future-red.svg)]() [![MIT License](https://img.shields.io/badge/License-MIT-yellow.svg)]() [![Ubuntu RT (Future)](https://img.shields.io/badge/Ubuntu_RT-Future-purple.svg)]()
## 🎬 YouTube Playlist
[▶️ GlassSync Full Video Documentation](https://www.youtube.com/playlist?list=PLMadf0IBbtAE)
Watch the complete development journey from MVP to full industrial digital twin platform.
## 📌 Overview
**GlassSync** is an industrial digital twin platform that demonstrates: - **Real-time PLC data mirroring** to 3D simulation - **OPC UA communication** with Siemens S7-1500 PLC - **CoppeliaSim robot simulation** with IK (Inverse Kinematics) - **Python-based digital twin core** with AI analytics - **Glass cutting optimization** with minimal waste path planning - **MOVEIT manual control** and **What-If simulation** modes - **TimescaleDB** for real-time data storage with compression
## 🔧 Technologies
| Category | Technologies |
|----------|--------------|
| **PLC** | Siemens S7-1500T, TIA Portal, TO Kinematics |
| **Communication** | OPC UA, ZMQ, Profinet |
| **Simulation** | CoppeliaSim 4.6+, IRB140, IK Solver |
| **Programming** | Python 3.12+, Asyncio, Tkinter |
| **Database** | TimescaleDB, PostgreSQL |
| **Future** | ROS2, URDF, MoveIt2, Gazebo, Ubuntu RT |
## 📁 Project Structure
```
GlassSync/ ├── core/ # Clock and cycle management ├── plc/ # PLC communication (OPC UA) ├── control/ # MOVEIT, What-If, Motion planning ├── bridges/ # CoppeliaSim bridge ├── safety/ # Safety monitoring ├── persistence/ # TimescaleDB storage ├── redundancy/ # Active/Standby failover ├── gui/ # Tkinter dashboard ├── utils/ # Helpers and logging ├── scripts/ # Run scripts ├── config.yaml # Configuration ├── main.py # Main entry point └── scl.txt # PLC SCL code
```
## 🚀 Quick Start
### Prerequisites
| Requirement | Version | Purpose |
|-------------|---------|---------|
| Python | 3.12+ | Core runtime |
| CoppeliaSim | 4.6+ | 3D simulation |
| TimescaleDB | 2.0+ | Time‑series storage |
| Siemens TIA Portal | V17+ | PLC programming |
| PLCSim Advanced | V5.0+ | PLC simulation (optional) |
### Installation
```bash
# Clone git clone https://github.com/Nebras4u/GlassSync.git cd GlassSync # Install dependencies pip install -r requirements.txt # Configure cp config.yaml config.local.yaml # Edit config.local.yaml with your PLC IP and database credentials # Run python main.py
```
### Running Modes
| Command | Mode |
|---------|------|
| `python main.py` | Full system with GUI |
| `python scripts/run_plc.py` | PLC mirroring only |
| `python scripts/moveit_gui.py` | Manual control (MOVEIT) |
| `python scripts/whatif_gui.py` | What-If simulation |
| `python scripts/run_direct.py` | Direct CoppeliaSim test |
| `python scripts/run_manual.py` | Command‑line manual control |
## 🎯 Features
| Feature | Description |
|---------|-------------|
| **🔌 PLC Communication** | OPC UA with 50ms cycle (target: < 1ms with Ubuntu RT) |
| **🎮 3D Simulation** | CoppeliaSim with physics-based IK |
| **🎯 Dual Control** | PLC Online + What‑If Simulation |
| **🖐️ Manual Control** | MOVEIT mode for calibration and teaching |
| **💾 Data Persistence** | TimescaleDB with compression and retention |
| **🔄 Redundancy** | Active/Standby with automatic failover |
| **🛡️ Safety Monitoring** | Joint limits, speed checks, emergency stop |
| **🖥️ GUI Dashboard** | Real‑time monitoring and control |
| **⚡ Real‑Time Ready** | Designed for Ubuntu PREEMPT_RT kernel deployment |
| **🧠 AI Analytics** | Glass cutting optimization & path planning |
## 📊 Current Status (Phase 1 -- MVP)
### ✅ Working
- OPC UA communication with Siemens S7-1500 PLC - Real‑time data mirroring to CoppeliaSim - MOVEIT manual control mode - What‑If simulation mode - TimescaleDB data persistence - GUI dashboard for monitoring
### ⚠️ Known Issues (Phase 2)
- DH parameter mismatch between TIA Portal and CoppeliaSim - Singularity handling in CoppeliaSim - Robot dimension calibration required - Motion alignment needs improvement
### 🔜 Phase 2 Roadmap
- Ubuntu Real‑Time Kernel (< 1ms cycle) - ROS2 integration with URDF, MoveIt2, Gazebo - DH parameter alignment - Singularity resolution
Built with ❤️ for Industry 4.0
© 2026 GlassSync • [GitHub Repository](https://github.com/Nebras4u/GlassSync) • [YouTube Playlist](https://www.youtube.com/playlist?list=PLMadf0IBbtAE)