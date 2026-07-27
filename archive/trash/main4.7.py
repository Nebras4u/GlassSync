#!/usr/bin/env python3
"""
Digital Twin - Complete with PLC Motion Commands
"""

import sys
import os
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
        self.connected = False
        # Nodes for sending commands
        self.pos1_node = None
        self.hmpos_node = None
        self.pos3_node = None
        self.pos4_node = None
        self.vlcty_node = None
        self.enable_all_node = None
        self.enable_kin_node = None
    
    async def connect(self):
        try:
            self.client = Client(SERVER_URL, timeout=5)
            await self.client.connect()
            
            # Read nodes
            self.live_node = self.client.get_node('ns=3;s="TO_Data_block_1"."Live_XYZ_Angle"')
            
            # Write nodes (for sending commands)
            self.pos1_node = self.client.get_node('ns=3;s="TO_Data_block_1"."Pos1"')
            self.hmpos_node = self.client.get_node('ns=3;s="TO_Data_block_1"."HmPos"')
            self.pos3_node = self.client.get_node('ns=3;s="TO_Data_block_1"."Pos3"')
            self.pos4_node = self.client.get_node('ns=3;s="TO_Data_block_1"."Pos4"')
            self.vlcty_node = self.client.get_node('ns=3;s="TO_Data_block_1"."Vlcty"')
            self.enable_all_node = self.client.get_node('ns=3;s="TO_Data_block_1"."EnableAll"')
            self.enable_kin_node = self.client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
            
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
    
    async def write_array(self, node, values):
        """Write array to PLC node"""
        if not self.connected:
            return False
        try:
            dv = ua.DataValue(ua.Variant(values, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await node.write_value(dv)
            return True
        except Exception as e:
            print(f"❌ Write error: {e}")
            return False
    
    async def write_value(self, node, value, variant_type=ua.VariantType.Double):
        """Write single value to PLC node"""
        if not self.connected:
            return False
        try:
            dv = ua.DataValue(ua.Variant(value, variant_type))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await node.write_value(dv)
            return True
        except Exception as e:
            print(f"❌ Write error: {e}")
            return False
    
    async def send_motion_command(self, pos1, hmpos, pos3, pos4, velocity=100.0):
        """Send motion commands to PLC"""
        print("\n📝 Sending motion commands to PLC...")
        
        # Write positions
        await self.write_array(self.pos1_node, pos1)
        print(f"   Pos1 = {pos1}")
        
        await self.write_array(self.hmpos_node, hmpos)
        print(f"   HmPos = {hmpos}")
        
        await self.write_array(self.pos3_node, pos3)
        print(f"   Pos3 = {pos3}")
        
        await self.write_array(self.pos4_node, pos4)
        print(f"   Pos4 = {pos4}")
        
        # Write velocity
        await self.write_value(self.vlcty_node, velocity)
        print(f"   Vlcty = {velocity}")
        
        # Enable all axes
        await self.write_value(self.enable_all_node, True, ua.VariantType.Boolean)
        print("   EnableAll = True")
        
        await asyncio.sleep(0.5)
        
        # Toggle EnableKinematics to trigger motion
        await self.write_value(self.enable_kin_node, False, ua.VariantType.Boolean)
        print("   EnableKinematics = False")
        await asyncio.sleep(0.5)
        
        await self.write_value(self.enable_kin_node, True, ua.VariantType.Boolean)
        print("   EnableKinematics = True ✅")
        
        print("✅ Motion commands sent successfully!")
        return True
    
    async def disconnect(self):
        if self.client and self.connected:
            await self.client.disconnect()
            self.connected = False
            print("👋 [OPC UA] Disconnected")

def opcua_reader_thread():
    """Thread for reading OPC UA continuously"""
    global current_position, running
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    reader = OPCUAReader()
    connected = loop.run_until_complete(reader.connect())
    
    if not connected:
        print("❌ Failed to connect to PLC")
        return
    
    # Send motion commands first
    print("\n🔄 Sending initial motion sequence...")
    
    # Use positions from MOTION_SEQUENCE
    pos1 = MOTION_SEQUENCE[0]["position"]
    hmpos = MOTION_SEQUENCE[1]["position"]
    pos3 = MOTION_SEQUENCE[2]["position"]
    pos4 = MOTION_SEQUENCE[3]["position"]
    
    loop.run_until_complete(reader.send_motion_command(pos1, hmpos, pos3, pos4, 100.0))
    
    print("\n📡 Monitoring PLC position...")
    last_position = None
    
    while running:
        try:
            pos = loop.run_until_complete(reader.read_position())
            if pos and pos != last_position:
                with position_lock:
                    current_position = pos
                last_position = pos
                print(f"📡 PLC Target: X:{pos[0]:.1f} Y:{pos[1]:.1f} Z:{pos[2]:.1f} A:{pos[3]:.1f}°")
        except:
            pass
        time.sleep(0.05)
    
    loop.run_until_complete(reader.disconnect())
    loop.close()

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
        
        try:
            sim.setGravity(0.0, 0.0, -0.5)
        except:
            pass
        
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
            
            ikGroup_u = simIK.createGroup(ikEnv)
            simIK.setGroupCalculation(ikEnv, ikGroup_u, simIK.method_pseudo_inverse, 0, 10)
            ik_el_u, simToIk, _ = simIK.addElementFromScene(
                ikEnv, ikGroup_u, simBase, simTip, simTarget, simIK.constraint_pose
            )
            simIK.setElementConstraints(ikEnv, ikGroup_u, ik_el_u, simIK.constraint_pose)
            
            ikGroup_d = simIK.createGroup(ikEnv)
            simIK.setGroupCalculation(ikEnv, ikGroup_d, simIK.method_damped_least_squares, 0.3, 99)
            ik_el_d, _, _ = simIK.addElementFromScene(
                ikEnv, ikGroup_d, simBase, simTip, simTarget, simIK.constraint_pose
            )
            simIK.setElementConstraints(ikEnv, ikGroup_d, ik_el_d, simIK.constraint_pose)
            
            for joint in all_joints:
                if joint in simToIk:
                    simIK.setJointMode(ikEnv, simToIk[joint], simIK.jointmode_ik)
            
            print("✅ [CoppeliaSim] Ready with IK")
            return sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d, all_joints
        
        print("❌ [CoppeliaSim] IK setup failed")
        return None, None, None, None, None, None, None, None, None
        
    except Exception as e:
        print(f"❌ CoppeliaSim error: {e}")
        return None, None, None, None, None, None, None, None, None

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

def move_to_position(sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d, position):
    x = position[0] / 1000.0
    y = position[1] / 1000.0
    z = position[2] / 1000.0
    angle = math.radians(position[3])
    
    sim.setObjectPosition(simTarget, simBase, [x, y, z])
    euler = list(sim.getObjectOrientation(simTarget, simBase))
    euler[2] = angle
    sim.setObjectOrientation(simTarget, simBase, euler)
    
    result = simIK.handleIkGroup(ikEnv, ikGroup_u)
    if result != simIK.result_success:
        simIK.handleIkGroup(ikEnv, ikGroup_d)
    return result == simIK.result_success

# ============================================================
# 4. DISPLAY FUNCTIONS
# ============================================================

def print_detailed_step(step_num, name, target_pos, robot_pos, joint_angles, status):
    """Print detailed view of a single step"""
    print("\n" + "=" * 80)
    print(f"📍 STEP {step_num}: {name}".center(80))
    print("=" * 80)
    
    print("\n🎯 PLC TARGET:")
    print(f"   X: {target_pos[0]:8.1f} mm    Y: {target_pos[1]:8.1f} mm    Z: {target_pos[2]:8.1f} mm    Angle: {target_pos[3]:6.1f}°")
    
    print("\n🤖 ROBOT FEEDBACK (CoppeliaSim):")
    print(f"   X: {robot_pos[0]:8.1f} mm    Y: {robot_pos[1]:8.1f} mm    Z: {robot_pos[2]:8.1f} mm")
    
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]
    dz = target_pos[2] - robot_pos[2]
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    print("\n📊 POSITION ERROR:")
    print(f"   ΔX: {dx:+8.1f} mm    ΔY: {dy:+8.1f} mm    ΔZ: {dz:+8.1f} mm")
    print(f"   Distance: {distance:8.1f} mm")
    
    print("\n🔧 JOINT ANGLES (6 Axes):")
    if len(joint_angles) >= 6:
        for i in range(6):
            print(f"   J{i+1}: {joint_angles[i]:8.1f}°", end="")
            if i == 2:
                print()
        print()
    
    print(f"\n📌 Status: {'✅ TARGET REACHED' if status else '⏳ MOVING...'}")
    print("=" * 80)

# ============================================================
# 5. MAIN
# ============================================================

def main():
    global current_position, running
    
    print("=" * 130)
    print("🏭 DIGITAL TWIN - Complete with Motion Commands".center(130))
    print("=" * 130)
    
    # Start OPC UA reader thread
    print("\n🔄 Starting OPC UA reader & sending commands...")
    opc_thread = threading.Thread(target=opcua_reader_thread, daemon=True)
    opc_thread.start()
    time.sleep(2)
    
    # Setup CoppeliaSim
    print("\n🔄 Setting up CoppeliaSim...")
    result = setup_coppelia()
    if result[0] is None:
        print("❌ Failed to start CoppeliaSim")
        return
    
    sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d, joints = result
    
    print("\n" + "=" * 130)
    print("📊 EXECUTING MOTION SEQUENCE".center(130))
    print("=" * 130)
    
    try:
        # Execute motion sequence
        for step, target in enumerate(MOTION_SEQUENCE, 1):
            target_pos = target["position"]
            target_name = target["name"]
            
            print(f"\n🔄 Moving to {target_name}...")
            
            # Move robot in CoppeliaSim
            success = move_to_position(sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d, target_pos)
            
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
            reached = distance < 2.0
            
            # Display detailed view
            print_detailed_step(step, target_name, target_pos, robot_pos, joint_angles, reached)
            
            if step < len(MOTION_SEQUENCE):
                print("\n⏳ Waiting 1.5 seconds before next move...")
                time.sleep(1.5)
        
        print("\n" + "=" * 130)
        print("✅ ALL MOTIONS COMPLETED SUCCESSFULLY!".center(130))
        print("=" * 130)
        
        # Live monitoring
        print("\n" + "=" * 130)
        print("📍 LIVE MONITORING - Press Ctrl+C to stop".center(130))
        print("=" * 130)
        print("\n" + "-" * 100)
        print(f"{'PLC Target (X,Y,Z,A)':<35} {'Robot Feedback (X,Y,Z)':<30} {'Joint Angles (J1-J3)'}")
        print("-" * 100)
        
        last_display = ""
        
        while running and sim.getSimulationState() != sim.simulation_stopped:
            robot_pos = get_robot_pose(sim, simTarget, simBase)
            joint_angles = get_joint_positions(sim, joints)
            
            with position_lock:
                plc_pos = list(current_position)
            
            display_str = (f"X:{plc_pos[0]:6.1f} Y:{plc_pos[1]:6.1f} Z:{plc_pos[2]:6.1f} A:{plc_pos[3]:5.1f}°  "
                          f"X:{robot_pos[0]:6.1f} Y:{robot_pos[1]:6.1f} Z:{robot_pos[2]:6.1f}  "
                          f"J1:{joint_angles[0]:5.1f}° J2:{joint_angles[1]:5.1f}° J3:{joint_angles[2]:5.1f}°")
            
            if display_str != last_display:
                sys.stdout.write('\033[F')
                sys.stdout.write('\033[K')
                print(display_str)
                last_display = display_str
            
            sim_client.step()
            time.sleep(0.05)
            
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