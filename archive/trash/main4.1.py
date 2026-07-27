#!/usr/bin/env python3
"""
Digital Twin - Motion Sequence with Debug
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
position_lock = threading.Lock()
running = True
motion_completed = False

# Motion sequence positions (X, Y, Z, Angle in degrees)
# Using positions that are within the robot's workspace
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
        
        print("🔍 Finding objects in scene...")
        for obj in all_objects:
            alias = sim.getObjectAlias(obj)
            obj_type = sim.getObjectType(obj)
            
            if alias:
                print(f"   Found: {alias} (Type: {obj_type})")
            
            if alias and 'IRB140' in alias:
                simBase = obj
                print(f"   ✅ Base found: {alias}")
            elif alias and 'target' in alias.lower():
                simTarget = obj
                print(f"   ✅ Target found: {alias}")
            elif alias and 'tip' in alias.lower():
                simTip = obj
                print(f"   ✅ Tip found: {alias}")
        
        # Get all joints
        all_joints = sim.getObjectsInTree(sim.handle_scene, sim.object_joint_type)
        print(f"   Found {len(all_joints)} joints")
        
        if not simBase and all_joints:
            simBase = sim.getObjectParent(all_joints[0])
            print(f"   Using parent of first joint as base")
        
        # Configure joints
        for i, j in enumerate(all_joints):
            sim.setJointMode(j, sim.jointmode_force, 0)
            sim.setJointInterval(j, True, [-3.0, 3.0])
            sim.setJointTargetVelocity(j, 0.0)
            sim.setJointTargetPosition(j, 0.0)
            sim.setJointMaxForce(j, 200.0)
            print(f"   Joint {i+1} configured")
        
        # Setup IK
        if all([simBase, simTip, simTarget]):
            print("🔧 Setting up IK...")
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
                    print(f"   Joint {joint} set to IK mode")
            
            print("✅ [CoppeliaSim] IK Ready")
            return sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup, all_joints
        
        print("❌ [CoppeliaSim] IK setup failed - missing required objects")
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

def move_to_position(sim, simIK, simBase, simTarget, ikEnv, ikGroup, position, target_name=""):
    """Move robot to a specific position with better error handling"""
    try:
        x = position[0] / 1000.0  # mm to m
        y = position[1] / 1000.0
        z = position[2] / 1000.0
        angle = math.radians(position[3])
        
        print(f"   🎯 Moving to {target_name}: X={x*1000:.1f}mm Y={y*1000:.1f}mm Z={z*1000:.1f}mm A={position[3]:.1f}°")
        
        # Update target position
        sim.setObjectPosition(simTarget, simBase, [x, y, z])
        
        # Update target orientation (keep only Z rotation)
        euler = list(sim.getObjectOrientation(simTarget, simBase))
        euler[2] = angle
        sim.setObjectOrientation(simTarget, simBase, euler)
        
        # Apply IK with multiple attempts
        max_attempts = 5
        for attempt in range(max_attempts):
            result = simIK.handleIkGroup(ikEnv, ikGroup)
            if result == simIK.result_success:
                print(f"   ✅ IK successful on attempt {attempt+1}")
                return True
            else:
                print(f"   ⚠️ IK attempt {attempt+1} failed with code: {result}")
                # Try with different method
                if attempt == 2:
                    # Try damped least squares as fallback
                    simIK.setGroupCalculation(ikEnv, ikGroup, simIK.method_damped_least_squares, 0.3, 99)
        
        print(f"   ❌ IK failed after {max_attempts} attempts")
        return False
        
    except Exception as e:
        print(f"   ❌ Move error: {e}")
        return False

# ============================================================
# 4. MAIN LOOP WITH MOTION SEQUENCE
# ============================================================

def main():
    global current_position, running, motion_completed
    
    print("=" * 70)
    print("🏭 DIGITAL TWIN - Motion Sequence".center(70))
    print("=" * 70)
    
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
    
    print("\n" + "=" * 70)
    print("🔄 RUNNING MOTION SEQUENCE".center(70))
    print("=" * 70)
    print("\n📊 Live Data:")
    print("-" * 100)
    print(f"{'Step':<8} {'Target':<30} {'Robot Pose':<30} {'Status'}")
    print("-" * 100)
    
    # Execute motion sequence
    step = 0
    total_steps = len(MOTION_SEQUENCE)
    
    # First, move to a safe home position
    print("\n🏠 Moving to home position...")
    home_pos = [300.0, 0.0, 300.0, 0.0]  # Safe position
    move_to_position(sim, simIK, simBase, simTarget, ikEnv, ikGroup, home_pos, "Home")
    
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
            
            if success:
                # Step simulation to allow robot to move
                for _ in range(20):  # Allow time for movement
                    sim_client.step()
                    time.sleep(0.02)
                
                # Get feedback: robot pose
                robot_pos, robot_ori = get_robot_pose(sim, simTarget, simBase)
                joint_angles = get_joint_positions(sim, joints)
                
                # Calculate distance to target (in mm)
                dx = (target_pos[0] - robot_pos[0]*1000)
                dy = (target_pos[1] - robot_pos[1]*1000)
                dz = (target_pos[2] - robot_pos[2]*1000)
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Display current status
                status = "✅ Done" if distance < 2.0 else f"⏳ Moving... ({distance:.1f}mm)"
                
                print(f"{target_name:<8} "
                      f"X:{target_pos[0]:6.1f} Y:{target_pos[1]:6.1f} Z:{target_pos[2]:6.1f} A:{target_pos[3]:5.1f}°  "
                      f"X:{robot_pos[0]*1000:6.1f} Y:{robot_pos[1]*1000:6.1f} Z:{robot_pos[2]*1000:6.1f}  "
                      f"{status}")
                
                # Move to next step
                step += 1
                
                # Pause between moves
                if step < total_steps:
                    print(f"⏳ Waiting before next move...")
                    for _ in range(25):  # 0.5 seconds
                        sim_client.step()
                        time.sleep(0.02)
            else:
                print(f"❌ Failed to move to {target_name}")
                # Try to continue to next position anyway
                step += 1
        
        if step >= total_steps:
            print("\n" + "=" * 70)
            print("✅ ALL MOTIONS COMPLETED!".center(70))
            print("=" * 70)
            motion_completed = True
            
            # Keep running to show final position
            while running and sim.getSimulationState() != sim.simulation_stopped:
                robot_pos, robot_ori = get_robot_pose(sim, simTarget, simBase)
                joint_angles = get_joint_positions(sim, joints)
                print(f"\r📍 Final: X:{robot_pos[0]*1000:6.1f} Y:{robot_pos[1]*1000:6.1f} Z:{robot_pos[2]*1000:6.1f}  "
                      f"J1:{math.degrees(joint_angles[0]):5.1f}° J2:{math.degrees(joint_angles[1]):5.1f}° J3:{math.degrees(joint_angles[2]):5.1f}°", end="")
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