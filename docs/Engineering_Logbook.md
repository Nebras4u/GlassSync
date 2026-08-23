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

### Files Created

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

## Phase 2 — Python + OPC UA Bridge (Upcoming)

Date: Upcoming
Video: Upcoming

### Planned Tasks

- Create OPC UA client in Python
- Connect AAS with Python
- Synchronize data between PLC and AAS
- Send data via MQTT
- Store data in TimescaleDB

### Planned Files

python/src/opcua_client.py
python/src/aas_server.py
python/src/data_sync.py
python/src/mqtt_publisher.py
python/src/database.py

---

Last Updated: 2026-08-21
Author: Nebras