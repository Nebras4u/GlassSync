#!/usr/bin/env python3
"""
Digital Twin - Complete Feedback with PLC Joint Angles
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

# Shared variables between threads
current_position = [0.0, 0.0, 0.0, 0.0]  # X, Y, Z, Angle
plc_act_pos = [0.0, 0.0, 0.0, 0.0]  # PLC actual joint angles (ActPos)
position_lock = threading.Lock()
running = True
motion_completed = False

# Motion sequence positions (X, Y, Z, Angle in degrees)
MOTION_SEQUENCE = [
    {"name": "Pos1", "position": [300.0, 100.0, 300.0, 0.0]},
    {"name": "HmPos", "position": [400.0, 150.0, 350.0, 30.0]},
    {"name": "Pos3", "position": [500.0, 200.0, 400.0, 60.0]},
    {"name": "Pos4", "position": [600.0, 250.0, 450.0, 90.0]},
]

# ============================================================
# 2. OPC UA READER (Continuous)
# ============================================================

class OPCUAReader:
    def __init__(self):
        self.client = None
        self.live_node = None
        self.act_pos_node = None  # Node for ActPos (PLC joint angles)
        self.connected = False
    
    async def connect(self):
        try:
            self.client = Client(SERVER_URL, timeout=5)
            await self.client.connect()
            self.live_node = self.client.get_node('ns=3;s="TO_Data_block_1"."Live_XYZ_Angle"')
            
            # Get ActPos node (PLC actual joint angles)
            try:
                self.act_pos_node = self.client.get_node('ns=3;s="TO_Data_block_1"."ActPos"')
                print("✅ [OPC UA] ActPos node found")
            except Exception as e:
                print(f"⚠️ [OPC UA] ActPos node not found: {e}")
                self.act_pos_node = None
                
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
    
    async def read_act_pos(self):
        """Read ActPos from PLC (actual joint angles)"""
        if not self.connected or not self.act_pos_node:
            return None
        try:
            value = await self.act_pos_node.read_value()
            if value and len(value) >= 4:
                return list(value)
        except:
            pass
        return None
    
    async def disconnect(self):
        if self.client and self.connected:
            await self.client.disconnect()
            self.connected = False
            print("👋 [OPC UA] Disconnected")

def opcua_reader_thread():
    """Thread for reading OPC UA continuously"""
    global current_position, plc_act_pos, running
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    reader = OPCUAReader()
    connected = loop.run_until_complete(reader.connect())
    
    if not connected:
        print("❌ Failed to connect to PLC")
        return
    
    last_position = None
    last_act_pos = None
    
    while running:
        try:
            # Read position
            pos = loop.run_until_complete(reader.read_position())
            if pos and pos != last_position:
                with position_lock:
                    current_position = pos
                last_position = pos
                print(f"📡 PLC Target: X:{pos[0]:.1f} Y:{pos[1]:.1f} Z:{pos[2]:.1f} A:{pos[3]:.1f}°")
            
            # Read ActPos (joint angles)
            act_pos = loop.run_until_complete(reader.read_act_pos())
            if act_pos and act_pos != last_act_pos:
                with position_lock:
                    plc_act_pos = act_pos
                last_act_pos = act_pos
                # Display PLC joint angles
                print(f"🔧 PLC Joints: J1:{act_pos[0]:6.1f}° J2:{act_pos[1]:6.1f}° J3:{act_pos[2]:6.1f}° J4:{act_pos[3]:6.1f}°")
                    
        except Exception as e:
            pass
        time.sleep(0.05)
    
    loop.run_until_complete(reader.disconnect())
    loop.close()

# ============================================================
# 3. COPPELIASIM SETUP
# ============================================================

def setup_coppelia():
    """Setup CoppeliaSim and IK"""
    try:
        sim_client = RemoteAPIClient()
        sim = sim_client.require('sim')
        simIK = sim_client.require('simIK')
        
        sim_client.setStepping(True)
        sim.startSimulation()
        sim.setFloatParam(sim.floatparam_simulation_time_step, 0.01)
        
        # Get objects
        all_objects = sim.getObjectsInTree(sim.handle_scene)
        simBase, simTip, simTarget = None, None, None
        all_joints = []
        
        for obj in all_objects:
            alias = sim.getObjectAlias(obj)
            if alias:
                if 'IRB140' in alias:
                    simBase = obj
                elif 'target' in alias.lower():
                    simTarget = obj
                elif 'tip' in alias.lower():
                    simTip = obj
        
        # Get all joints
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
            
            # Create IK group with pseudo inverse
            ikGroup = simIK.createGroup(ikEnv)
            simIK.setGroupCalculation(ikEnv, ikGroup, simIK.method_pseudo_inverse, 0, 10)
            
            # Add IK element
            ik_el, simToIk, _ = simIK.addElementFromScene(
                ikEnv, ikGroup, simBase, simTip, simTarget, simIK.constraint_pose
            )
            simIK.setElementConstraints(ikEnv, ikGroup, ik_el, simIK.constraint_pose)
            
            # Set joint modes
            for joint in all_joints:
                if joint in simToIk:
                    simIK.setJointMode(ikEnv, simToIk[joint], simIK.jointmode_ik)
            
            print("✅ [CoppeliaSim] IK Ready")
            return sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup, all_joints
        
        print("❌ [CoppeliaSim] IK setup failed")
        return None, None, None, None, None, None, None, None
        
    except Exception as e:
        print(f"❌ CoppeliaSim setup error: {e}")
        return None, None, None, None, None, None, None, None

def get_joint_positions(sim, joints):
    """Get current joint positions as feedback"""
    positions = []
    for joint in joints:
        try:
            positions.append(sim.getJointPosition(joint))
        except:
            positions.append(0.0)
    return positions

def get_robot_pose(sim, simTarget, simBase):
    """Get current robot end-effector pose"""
    try:
        pos = list(sim.getObjectPosition(simTarget, simBase))
        ori = list(sim.getObjectOrientation(simTarget, simBase))
        return pos, ori
    except:
        return [0,0,0], [0,0,0]

def is_ik_successful(result):
    """Check if IK result is successful"""
    if isinstance(result, tuple):
        return result[0] == 1
    return result == 1

def move_to_position(sim, simIK, simBase, simTarget, ikEnv, ikGroup, position, target_name=""):
    """Move robot to a specific position"""
    try:
        x = position[0] / 1000.0  # mm to m
        y = position[1] / 1000.0
        z = position[2] / 1000.0
        angle = math.radians(position[3])
        
        # Update target position
        sim.setObjectPosition(simTarget, simBase, [x, y, z])
        
        # Update target orientation (keep only Z rotation)
        euler = list(sim.getObjectOrientation(simTarget, simBase))
        euler[2] = angle
        sim.setObjectOrientation(simTarget, simBase, euler)
        
        # Apply IK
        result = simIK.handleIkGroup(ikEnv, ikGroup)
        
        # Check if IK was successful
        if is_ik_successful(result):
            return True
        else:
            return False
        
    except Exception as e:
        print(f"   ❌ Move error: {e}")
        return False

# ============================================================
# 4. DISPLAY FUNCTIONS
# ============================================================

def display_complete_status(step_name, target_pos, robot_pos, robot_angles, plc_angles, error, status):
    """Display complete status with all information"""
    print("\n" + "=" * 110)
    print(f"📍 {step_name}")
    print("=" * 110)
    
    # Target position
    print(f"\n🎯 TARGET (PLC Command):")
    print(f"   Position: X:{target_pos[0]:8.1f}mm Y:{target_pos[1]:8.1f}mm Z:{target_pos[2]:8.1f}mm Angle:{target_pos[3]:6.1f}°")
    
    # Robot actual position (feedback)
    print(f"\n🤖 ROBOT FEEDBACK (CoppeliaSim):")
    print(f"   Position: X:{robot_pos[0]*1000:8.1f}mm Y:{robot_pos[1]*1000:8.1f}mm Z:{robot_pos[2]*1000:8.1f}mm")
    
    # Error
    print(f"\n📊 ERROR (Target - Actual):")
    error_x = target_pos[0] - robot_pos[0]*1000
    error_y = target_pos[1] - robot_pos[1]*1000
    error_z = target_pos[2] - robot_pos[2]*1000
    print(f"   ΔX: {error_x:8.1f}mm ΔY: {error_y:8.1f}mm ΔZ: {error_z:8.1f}mm")
    print(f"   Distance: {error:8.1f}mm")
    
    # Joint angles from CoppeliaSim
    print(f"\n🔧 JOINT ANGLES (CoppeliaSim - 6 Axes):")
    if len(robot_angles) >= 6:
        for i in range(6):
            print(f"   J{i+1}: {math.degrees(robot_angles[i]):8.1f}°", end="")
            if i == 2:
                print()
    else:
        for i, angle in enumerate(robot_angles[:3]):
            print(f"   J{i+1}: {math.degrees(angle):8.1f}°", end="")
        print()
    
    # PLC joint angles (ActPos - 4 Axes)
    if plc_angles and len(plc_angles) >= 4:
        print(f"\n🔧 JOINT ANGLES (PLC - 4 Axes):")
        print(f"   Axis1: {plc_angles[0]:8.1f}°   Axis2: {plc_angles[1]:8.1f}°   Axis3: {plc_angles[2]:8.1f}°   Axis4: {plc_angles[3]:8.1f}°")
    
    # Comparison between PLC and CoppeliaSim (first 4 joints)
    if plc_angles and len(plc_angles) >= 4 and len(robot_angles) >= 4:
        print(f"\n📊 COMPARISON (PLC vs CoppeliaSim):")
        for i in range(4):
            plc_deg = plc_angles[i]
            sim_deg = math.degrees(robot_angles[i])
            diff = plc_deg - sim_deg
            print(f"   Axis{i+1}: PLC={plc_deg:8.1f}°  Sim={sim_deg:8.1f}°  Diff={diff:8.1f}°")
    
    # Status
    print(f"\n📌 Status: {status}")
    print("=" * 110)

# ============================================================
# 5. MAIN LOOP WITH MOTION SEQUENCE
# ============================================================

def main():
    global current_position, plc_act_pos, running, motion_completed
    
    print("=" * 110)
    print("🏭 DIGITAL TWIN - Complete Feedback with PLC Joint Angles".center(110))
    print("=" * 110)
    
    # Start OPC UA reader thread
    print("\n🔄 Starting OPC UA reader...")
    opc_thread = threading.Thread(target=opcua_reader_thread, daemon=True)
    opc_thread.start()
    time.sleep(2)
    
    # Setup CoppeliaSim
    print("\n🔄 Setting up CoppeliaSim...")
    result = setup_coppelia()
    if result[0] is None:
        print("❌ Failed to start CoppeliaSim")
        return
    
    sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup, joints = result
    
    print("\n" + "=" * 110)
    print("🔄 RUNNING MOTION SEQUENCE".center(110))
    print("=" * 110)
    
    # Execute motion sequence
    step = 0
    total_steps = len(MOTION_SEQUENCE)
    
    # First, move to a safe home position
    print("\n🏠 Moving to home position...")
    home_pos = [300.0, 0.0, 300.0, 0.0]
    if move_to_position(sim, simIK, simBase, simTarget, ikEnv, ikGroup, home_pos, "Home"):
        print("   ✅ Home position reached")
    else:
        print("   ⚠️ Could not reach home, continuing anyway")
    
    # Step the simulation to apply changes
    for _ in range(10):
        sim_client.step()
        time.sleep(0.02)
    
    try:
        while sim.getSimulationState() != sim.simulation_stopped and running and step < total_steps:
            # Get current target from sequence
            target = MOTION_SEQUENCE[step]
            target_pos = target["position"]
            target_name = target["name"]
            
            # Move to target position
            success = move_to_position(sim, simIK, simBase, simTarget, ikEnv, ikGroup, target_pos, target_name)
            
            # Step simulation to allow robot to move
            for _ in range(30):
                sim_client.step()
                time.sleep(0.02)
            
            # Get feedback: robot pose
            robot_pos, robot_ori = get_robot_pose(sim, simTarget, simBase)
            robot_angles = get_joint_positions(sim, joints)
            
            # Get PLC joint angles (ActPos)
            with position_lock:
                plc_angles = list(plc_act_pos)
            
            # Calculate distance to target (in mm)
            dx = (target_pos[0] - robot_pos[0]*1000)
            dy = (target_pos[1] - robot_pos[1]*1000)
            dz = (target_pos[2] - robot_pos[2]*1000)
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Determine status
            if success and distance < 5.0:
                status = "✅ Target Reached"
            elif success:
                status = f"⏳ Moving... (Distance: {distance:.1f}mm)"
            else:
                status = "⚠️ IK Issue"
            
            # Display complete status
            display_complete_status(
                f"Step {step+1}/{total_steps}: {target_name}",
                target_pos,
                robot_pos,
                robot_angles,
                plc_angles,
                distance,
                status
            )
            
            # Move to next step
            step += 1
            
            # Pause between moves
            if step < total_steps:
                print(f"\n⏳ Waiting 2 seconds before next move...")
                for _ in range(100):
                    sim_client.step()
                    time.sleep(0.02)
        
        if step >= total_steps:
            print("\n" + "=" * 110)
            print("✅ ALL MOTIONS COMPLETED!".center(110))
            print("=" * 110)
            motion_completed = True
            
            # Keep running to show final position with live updates
            print("\n📍 LIVE MONITORING (Press Ctrl+C to stop):")
            print("-" * 110)
            
            while running and sim.getSimulationState() != sim.simulation_stopped:
                robot_pos, robot_ori = get_robot_pose(sim, simTarget, simBase)
                robot_angles = get_joint_positions(sim, joints)
                
                with position_lock:
                    plc_angles = list(plc_act_pos)
                    plc_pos = list(current_position)
                
                # Clear line and display
                print("\r" + " " * 110 + "\r", end="")
                display_str = (f"📍 Pos: X:{robot_pos[0]*1000:6.1f} Y:{robot_pos[1]*1000:6.1f} Z:{robot_pos[2]*1000:6.1f}  |  "
                              f"Sim J1:{math.degrees(robot_angles[0]):5.1f}° J2:{math.degrees(robot_angles[1]):5.1f}° J3:{math.degrees(robot_angles[2]):5.1f}°  |  ")
                if plc_angles and len(plc_angles) >= 4:
                    display_str += f"PLC J1:{plc_angles[0]:5.1f}° J2:{plc_angles[1]:5.1f}° J3:{plc_angles[2]:5.1f}° J4:{plc_angles[3]:5.1f}°"
                print(display_str, end="")
                
                sim_client.step()
                time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n⏹️ Stopping...")
    
    # Cleanup
    running = False
    sim.stopSimulation()
    print("\n👋 Done")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️ Stopped by user")
        running = False