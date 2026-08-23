#!/usr/bin/env python3
"""
Digital Twin - Complete Display (PLC Targets + CoppeliaSim Feedback)
"""

import sys
import asyncio
import math
import time
import threading
from asyncua import Client, ua
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ============================================================
# 1. SETTINGS
# ============================================================

PLC_IP = "192.168.0.1"
SERVER_URL = f"opc.tcp://{PLC_IP}:4840"

# Shared variables
current_position = [0.0, 0.0, 0.0, 0.0]  # X, Y, Z, Angle
position_lock = threading.Lock()
running = True

# Motion sequence (Target Positions from PLC)
MOTION_SEQUENCE = [
    {"name": "Pos1", "position": [300.0, 100.0, 300.0, 0.0]},
    {"name": "HmPos", "position": [400.0, 150.0, 350.0, 30.0]},
    {"name": "Pos3", "position": [500.0, 200.0, 400.0, 60.0]},
    {"name": "Pos4", "position": [600.0, 250.0, 450.0, 90.0]},
]

# ============================================================
# 2. OPC UA READER
# ============================================================

class OPCUAReader:
    def __init__(self):
        self.client = None
        self.live_node = None
        self.connected = False
    
    async def connect(self):
        try:
            self.client = Client(SERVER_URL, timeout=5)
            await self.client.connect()
            self.live_node = self.client.get_node('ns=3;s="TO_Data_block_1"."Live_XYZ_Angle"')
            self.connected = True
            print("✅ [OPC UA] Connected")
            return True
        except Exception as e:
            print(f"❌ [OPC UA] Connection failed: {e}")
            return False
    
    async def read_position(self):
        if not self.connected or not self.live_node:
            return None
        try:
            value = await self.live_node.read_value()
            if value and len(value) >= 4:
                return list(value)
        except:
            self.connected = False
        return None
    
    async def disconnect(self):
        if self.client and self.connected:
            await self.client.disconnect()
            self.connected = False

def opcua_reader_thread():
    global current_position, running
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    reader = OPCUAReader()
    connected = loop.run_until_complete(reader.connect())
    if not connected:
        return
    while running:
        pos = loop.run_until_complete(reader.read_position())
        if pos:
            with position_lock:
                current_position = pos
        time.sleep(0.05)
    loop.run_until_complete(reader.disconnect())

# ============================================================
# 3. COPPELIASIM SETUP
# ============================================================

def setup_coppelia():
    try:
        sim_client = RemoteAPIClient()
        sim = sim_client.require('sim')
        simIK = sim_client.require('simIK')
        
        sim_client.setStepping(True)
        sim.startSimulation()
        sim.setFloatParam(sim.floatparam_simulation_time_step, 0.01)
        
        # Find objects
        all_objects = sim.getObjectsInTree(sim.handle_scene)
        simBase, simTip, simTarget = None, None, None
        
        for obj in all_objects:
            alias = sim.getObjectAlias(obj)
            if alias:
                if 'IRB140' in alias:
                    simBase = obj
                elif 'target' in alias.lower():
                    simTarget = obj
                elif 'tip' in alias.lower():
                    simTip = obj
        
        all_joints = sim.getObjectsInTree(sim.handle_scene, sim.object_joint_type)
        
        if not simBase and all_joints:
            simBase = sim.getObjectParent(all_joints[0])
        
        # Configure joints
        for j in all_joints:
            sim.setJointMode(j, sim.jointmode_force, 0)
            sim.setJointInterval(j, True, [-3.0, 3.0])
            sim.setJointTargetVelocity(j, 0.0)
            sim.setJointTargetPosition(j, 0.0)
            sim.setJointMaxForce(j, 200.0)
        
        # Setup IK
        if all([simBase, simTip, simTarget]):
            ikEnv = simIK.createEnvironment()
            ikGroup = simIK.createGroup(ikEnv)
            simIK.setGroupCalculation(ikEnv, ikGroup, simIK.method_pseudo_inverse, 0, 10)
            ik_el, simToIk, _ = simIK.addElementFromScene(
                ikEnv, ikGroup, simBase, simTip, simTarget, simIK.constraint_pose
            )
            simIK.setElementConstraints(ikEnv, ikGroup, ik_el, simIK.constraint_pose)
            
            for joint in all_joints:
                if joint in simToIk:
                    simIK.setJointMode(ikEnv, simToIk[joint], simIK.jointmode_ik)
            
            print("✅ [CoppeliaSim] IK Ready")
            return sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup, all_joints
        
        return None, None, None, None, None, None, None, None
    except Exception as e:
        print(f"❌ CoppeliaSim error: {e}")
        return None, None, None, None, None, None, None, None

def get_joint_positions(sim, joints):
    positions = []
    for joint in joints:
        try:
            positions.append(math.degrees(sim.getJointPosition(joint)))
        except:
            positions.append(0.0)
    return positions

def get_robot_pose(sim, simTarget, simBase):
    try:
        pos = list(sim.getObjectPosition(simTarget, simBase))
        return [p * 1000 for p in pos]  # Convert to mm
    except:
        return [0, 0, 0]

def move_to_position(sim, simIK, simBase, simTarget, ikEnv, ikGroup, position):
    x = position[0] / 1000.0
    y = position[1] / 1000.0
    z = position[2] / 1000.0
    angle = math.radians(position[3])
    
    sim.setObjectPosition(simTarget, simBase, [x, y, z])
    euler = list(sim.getObjectOrientation(simTarget, simBase))
    euler[2] = angle
    sim.setObjectOrientation(simTarget, simBase, euler)
    
    result = simIK.handleIkGroup(ikEnv, ikGroup)
    return result[0] == 1 if isinstance(result, tuple) else result == 1

# ============================================================
# 4. MAIN - DISPLAY
# ============================================================

def main():
    global current_position, running
    
    print("=" * 120)
    print("🏭 DIGITAL TWIN - PLC Targets vs CoppeliaSim Feedback".center(120))
    print("=" * 120)
    
    # Start OPC UA reader
    opc_thread = threading.Thread(target=opcua_reader_thread, daemon=True)
    opc_thread.start()
    time.sleep(2)
    
    # Setup CoppeliaSim
    result = setup_coppelia()
    if result[0] is None:
        print("❌ Failed to start CoppeliaSim")
        return
    
    sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup, joints = result
    
    print("\n" + "=" * 120)
    print("🔄 EXECUTING MOTION SEQUENCE".center(120))
    print("=" * 120)
    
    # Home position
    home_pos = [300.0, 0.0, 300.0, 0.0]
    move_to_position(sim, simIK, simBase, simTarget, ikEnv, ikGroup, home_pos)
    for _ in range(10):
        sim_client.step()
        time.sleep(0.02)
    
    print("\n" + "-" * 120)
    print(f"{'Step':<8} {'PLC Target (X,Y,Z,A)':<35} {'Robot Feedback (X,Y,Z)':<30} {'Joint Angles (J1-J6)'}")
    print("-" * 120)
    
    try:
        for step, target in enumerate(MOTION_SEQUENCE, 1):
            target_pos = target["position"]
            target_name = target["name"]
            
            # Move
            success = move_to_position(sim, simIK, simBase, simTarget, ikEnv, ikGroup, target_pos)
            
            # Allow time to move
            for _ in range(30):
                sim_client.step()
                time.sleep(0.02)
            
            # Get feedback
            robot_pos = get_robot_pose(sim, simTarget, simBase)
            joint_angles = get_joint_positions(sim, joints)
            
            # Calculate error
            dx = target_pos[0] - robot_pos[0]
            dy = target_pos[1] - robot_pos[1]
            dz = target_pos[2] - robot_pos[2]
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            status = "✅" if distance < 2.0 else "⏳"
            
            # Display
            print(f"{target_name:<8} "
                  f"X:{target_pos[0]:6.1f} Y:{target_pos[1]:6.1f} Z:{target_pos[2]:6.1f} A:{target_pos[3]:5.1f}°  "
                  f"X:{robot_pos[0]:6.1f} Y:{robot_pos[1]:6.1f} Z:{robot_pos[2]:6.1f}  "
                  f"J1:{joint_angles[0]:5.1f}° J2:{joint_angles[1]:5.1f}° J3:{joint_angles[2]:5.1f}° J4:{joint_angles[3]:5.1f}° J5:{joint_angles[4]:5.1f}° J6:{joint_angles[5]:5.1f}°  {status}")
            
            if step < len(MOTION_SEQUENCE):
                time.sleep(1)
        
        print("-" * 120)
        print("\n✅ ALL MOTIONS COMPLETED!")
        print("\n" + "=" * 120)
        print("📍 LIVE MONITORING - Press Ctrl+C to stop".center(120))
        print("=" * 120)
        
        # Live monitoring
        while running and sim.getSimulationState() != sim.simulation_stopped:
            robot_pos = get_robot_pose(sim, simTarget, simBase)
            joint_angles = get_joint_positions(sim, joints)
            
            with position_lock:
                plc_pos = list(current_position)
            
            print("\r" + " " * 120 + "\r", end="")
            print(f"📡 PLC: X:{plc_pos[0]:6.1f} Y:{plc_pos[1]:6.1f} Z:{plc_pos[2]:6.1f} A:{plc_pos[3]:5.1f}°  |  "
                  f"🤖 Sim: X:{robot_pos[0]:6.1f} Y:{robot_pos[1]:6.1f} Z:{robot_pos[2]:6.1f}  |  "
                  f"🔧 J1:{joint_angles[0]:5.1f}° J2:{joint_angles[1]:5.1f}° J3:{joint_angles[2]:5.1f}°", end="")
            
            sim_client.step()
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n⏹️ Stopping...")
    
    running = False
    sim.stopSimulation()
    print("\n👋 Done")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️ Stopped by user")
        running = False