#!/usr/bin/env python3
"""
Digital Twin - Single Motion with Full Details

This script creates a digital twin simulation where a PLC (Programmable Logic Controller)
controls a robot in CoppeliaSim. It reads real-time position data from the PLC via OPC UA
and visualizes it in a 3D simulation environment.

Key Features:
- OPC UA communication with Siemens PLC (TIA Portal)
- Real-time position tracking from PLC
- CoppeliaSim robot simulation with inverse kinematics
- Detailed position and joint angle feedback display
- Automatic motion sequence execution
"""

# ============================================================
# 1. LIBRARY IMPORTS
# ============================================================

import sys                      # System-specific parameters and functions (command line args, exit)
import os                       # Operating system interface (file paths, environment variables)
import asyncio                  # Asynchronous I/O for handling OPC UA operations without blocking
import math                     # Mathematical functions (trigonometry for angle conversions)
import time                     # Time-related functions (sleep, delays)
import threading                # Thread management for parallel execution (OPC UA reader runs in background)
from asyncua import Client, ua  # OPC UA library - Client for communication with PLC, ua for data types
from coppeliasim_zmqremoteapi_client import RemoteAPIClient  # CoppeliaSim Python API via ZMQ

# ============================================================
# 2. CONFIGURATION SETTINGS
# ============================================================

PLC_IP = "192.168.0.1"          # IP address of the Siemens PLC (change this to match your network)
SERVER_URL = f"opc.tcp://{PLC_IP}:4840"  # OPC UA server URL (standard port 4840 for OPC UA)

# Shared variables between threads (synchronized access using locks)
current_position = [0.0, 0.0, 0.0, 0.0]  # X, Y, Z, Angle (millimeters, degrees)
position_lock = threading.Lock()         # Thread lock to prevent race conditions when accessing current_position
running = True                           # Global flag to control the main loop (set to False to stop)
motion_started = False                   # Flag indicating if PLC motion has been triggered

# Motion sequence positions (X, Y, Z, Angle in degrees)
# These are the target positions that will be sent to the PLC
MOTION_SEQUENCE = [
    {"name": "Pos1", "position": [300.0, 100.0, 300.0, 0.0]},   # First target: X=300mm, Y=100mm, Z=300mm, Angle=0°
    {"name": "HmPos", "position": [400.0, 150.0, 350.0, 30.0]}, # Second target: Home position with 30° rotation
    {"name": "Pos3", "position": [500.0, 200.0, 400.0, 60.0]},  # Third target: 60° rotation
    {"name": "Pos4", "position": [600.0, 250.0, 450.0, 90.0]},  # Fourth target: 90° rotation
]

# ============================================================
# 3. OPC UA READER CLASS - Handles communication with PLC
# ============================================================

class OPCUAReader:
    """
    OPC UA Client for reading and writing data to/from the Siemens PLC.
    
    This class manages:
    - Connection to the OPC UA server
    - Reading Live_XYZ_Angle (current robot position)
    - Writing motion commands (Pos1, HmPos, Pos3, Pos4)
    - Enabling/Disabling axes and kinematics
    """
    
    def __init__(self):
        """Initialize OPC UA reader with default values"""
        self.client = None           # OPC UA client instance (created when connecting)
        self.live_node = None        # Node for reading Live_XYZ_Angle (position feedback)
        self.connected = False       # Connection status flag
        
        # Write nodes (for sending commands to PLC)
        self.pos1_node = None        # Node for writing Pos1 (first target position)
        self.hmpos_node = None       # Node for writing HmPos (home position)
        self.pos3_node = None        # Node for writing Pos3 (third target position)
        self.pos4_node = None        # Node for writing Pos4 (fourth target position)
        self.vlcty_node = None       # Node for writing Velocity (motion speed)
        self.enable_all_node = None  # Node for enabling all axes
        self.enable_kin_node = None  # Node for enabling kinematics
    
    async def connect(self):
        """
        Establish connection to the PLC OPC UA server.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            # Create OPC UA client with 5-second timeout
            self.client = Client(SERVER_URL, timeout=5)
            
            # Connect to the server
            await self.client.connect()
            
            # Initialize read node (Live_XYZ_Angle from PLC data block)
            # ns=3 refers to the namespace, s="..." is the node string identifier
            self.live_node = self.client.get_node('ns=3;s="TO_Data_block_1"."Live_XYZ_Angle"')
            
            # Initialize write nodes (for sending commands to PLC)
            # These nodes correspond to variables in the PLC data block
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
        """
        Read the current robot position from the PLC.
        
        Returns:
            list: [X, Y, Z, Angle] in millimeters and degrees, or None if reading fails
        """
        if not self.connected or not self.live_node:
            return None
        
        try:
            # Read the value from the Live_XYZ_Angle node
            value = await self.live_node.read_value()
            
            # Validate the data (should be an array of 4 doubles)
            if value and len(value) >= 4:
                return list(value)  # Convert to Python list
        except Exception as e:
            # If reading fails, mark connection as broken
            print(f"⚠️ Read error: {e}")
            self.connected = False
        
        return None
    
    async def write_array(self, node, values):
        """
        Write an array (4 values) to a PLC node.
        
        Args:
            node: OPC UA node object
            values: List of 4 floating-point values [X, Y, Z, Angle]
            
        Returns:
            bool: True if write successful, False otherwise
        """
        if not self.connected:
            return False
        
        try:
            # Create OPC UA DataValue with Variant (array of doubles)
            dv = ua.DataValue(ua.Variant(values, ua.VariantType.Double))
            # Clear timestamps to use server time
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            # Write to the PLC
            await node.write_value(dv)
            return True
        except Exception as e:
            print(f"❌ Write error: {e}")
            return False
    
    async def write_value(self, node, value, variant_type=ua.VariantType.Double):
        """
        Write a single value to a PLC node.
        
        Args:
            node: OPC UA node object
            value: Value to write (numeric or boolean)
            variant_type: OPC UA VariantType (Double or Boolean)
            
        Returns:
            bool: True if write successful, False otherwise
        """
        if not self.connected:
            return False
        
        try:
            # Create OPC UA DataValue with appropriate Variant type
            dv = ua.DataValue(ua.Variant(value, variant_type))
            # Clear timestamps
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            # Write to the PLC
            await node.write_value(dv)
            return True
        except Exception as e:
            print(f"❌ Write error: {e}")
            return False
    
    async def send_motion_command(self, pos1, hmpos, pos3, pos4, velocity=100.0):
        """
        Send motion commands to the PLC.
        
        This writes all four target positions, velocity, and toggles
        EnableKinematics to trigger motion execution.
        
        Args:
            pos1: List [X, Y, Z, Angle] for first position
            hmpos: List [X, Y, Z, Angle] for home position
            pos3: List [X, Y, Z, Angle] for third position
            pos4: List [X, Y, Z, Angle] for fourth position
            velocity: Motion speed (default 100.0)
            
        Returns:
            bool: True if all commands sent successfully
        """
        print("\n📝 Sending motion commands to PLC...")
        
        # Write all four target positions to PLC
        await self.write_array(self.pos1_node, pos1)
        print(f"   Pos1 = {pos1}")
        
        await self.write_array(self.hmpos_node, hmpos)
        print(f"   HmPos = {hmpos}")
        
        await self.write_array(self.pos3_node, pos3)
        print(f"   Pos3 = {pos3}")
        
        await self.write_array(self.pos4_node, pos4)
        print(f"   Pos4 = {pos4}")
        
        # Write velocity (motion speed)
        await self.write_value(self.vlcty_node, velocity)
        print(f"   Vlcty = {velocity}")
        
        # Enable all axes (power on)
        await self.write_value(self.enable_all_node, True, ua.VariantType.Boolean)
        print("   EnableAll = True")
        
        # Wait for 0.5 seconds to allow axes to enable
        await asyncio.sleep(0.5)
        
        # Toggle EnableKinematics to trigger motion execution
        # First set to False (disables kinematics)
        await self.write_value(self.enable_kin_node, False, ua.VariantType.Boolean)
        print("   EnableKinematics = False")
        
        # Wait 0.5 seconds
        await asyncio.sleep(0.5)
        
        # Then set to True (re-enables kinematics, triggering motion)
        await self.write_value(self.enable_kin_node, True, ua.VariantType.Boolean)
        print("   EnableKinematics = True ✅")
        
        print("✅ Motion commands sent successfully!")
        return True
    
    async def disconnect(self):
        """Close the OPC UA connection gracefully"""
        if self.client and self.connected:
            await self.client.disconnect()
            self.connected = False
            print("👋 [OPC UA] Disconnected")


def opcua_reader_thread():
    """
    Thread function for continuous OPC UA reading.
    
    This runs in a separate thread to:
    1. Send initial motion commands to PLC
    2. Continuously read Live_XYZ_Angle from PLC
    3. Update the shared current_position variable
    """
    global current_position, running, motion_started
    
    # Create new event loop for this thread (asyncio requires one per thread)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    # Create OPC UA reader instance
    reader = OPCUAReader()
    
    # Connect to PLC
    connected = loop.run_until_complete(reader.connect())
    
    if not connected:
        print("❌ Failed to connect to PLC")
        return
    
    # Extract position arrays from MOTION_SEQUENCE
    pos1 = MOTION_SEQUENCE[0]["position"]
    hmpos = MOTION_SEQUENCE[1]["position"]
    pos3 = MOTION_SEQUENCE[2]["position"]
    pos4 = MOTION_SEQUENCE[3]["position"]
    
    # Send motion commands to PLC
    loop.run_until_complete(reader.send_motion_command(pos1, hmpos, pos3, pos4, 100.0))
    motion_started = True  # Signal that motion has been triggered
    
    # Continuous reading loop
    while running:
        try:
            # Read current position from PLC
            pos = loop.run_until_complete(reader.read_position())
            
            # Update shared variable with thread lock
            if pos:
                with position_lock:
                    current_position = pos
        except Exception as e:
            # Silently handle errors in background thread
            pass
        
        # Wait 20ms before next read (50 Hz update rate)
        time.sleep(0.02)
    
    # Clean up connection when exiting
    loop.run_until_complete(reader.disconnect())
    loop.close()


# ============================================================
# 4. COPPELIASIM SETUP - Robot Simulation Configuration
# ============================================================

def setup_coppelia():
    """
    Initialize and configure CoppeliaSim simulation environment.
    
    This function:
    1. Connects to CoppeliaSim via ZMQ Remote API
    2. Starts the simulation
    3. Finds robot components (base, tip, target, joints)
    4. Configures joints for torque control
    5. Sets up inverse kinematics (IK) groups
    
    Returns:
        tuple: (sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d, all_joints)
        Returns None if setup fails
    """
    try:
        # Create connection to CoppeliaSim (default port 23000)
        sim_client = RemoteAPIClient()
        sim = sim_client.require('sim')      # CoppeliaSim core API
        simIK = sim_client.require('simIK')  # Inverse kinematics API
        
        # Configure simulation stepping and time step
        sim_client.setStepping(True)  # Manual stepping (not continuous)
        sim.startSimulation()          # Start the simulation
        sim.setFloatParam(sim.floatparam_simulation_time_step, 0.01)  # 10ms step
        
        # Set gravity (optional - for physics simulation)
        try:
            sim.setGravity(0.0, 0.0, -0.5)
        except:
            pass  # Ignore if gravity not supported
        
        # Find all objects in the scene
        all_objects = sim.getObjectsInTree(sim.handle_scene)
        simBase = None    # Robot base
        simTip = None     # Robot end-effector (tip)
        simTarget = None  # Target object for IK
        
        # Search for specific objects by alias
        for obj in all_objects:
            alias = sim.getObjectAlias(obj)
            if alias:
                if 'IRB140' in alias:
                    simBase = obj          # Found IRB140 robot base
                elif 'target' in alias.lower():
                    simTarget = obj        # Found target object
                elif 'tip' in alias.lower():
                    simTip = obj           # Found tip (end-effector)
        
        # Get all joints in the scene
        all_joints = sim.getObjectsInTree(sim.handle_scene, sim.object_joint_type)
        
        # If base not found, use parent of first joint
        if not simBase and all_joints:
            simBase = sim.getObjectParent(all_joints[0])
        
        # Configure each joint for torque/force control
        for j in all_joints:
            sim.setJointMode(j, sim.jointmode_force, 0)  # Force/torque mode
            sim.setJointInterval(j, True, [-3.0, 3.0])    # Joint limits ±3 radians
            sim.setJointTargetVelocity(j, 0.0)            # Zero target velocity
            sim.setJointTargetPosition(j, 0.0)            # Zero target position
            sim.setJointMaxForce(j, 200.0)                # Maximum force/torque
        
        # Setup Inverse Kinematics (IK) if all required objects are found
        if all([simBase, simTip, simTarget]):
            # Create IK environment
            ikEnv = simIK.createEnvironment()
            
            # Primary IK group using pseudo-inverse method
            ikGroup_u = simIK.createGroup(ikEnv)
            simIK.setGroupCalculation(ikEnv, ikGroup_u, simIK.method_pseudo_inverse, 0, 10)
            
            # Add IK element connecting base -> tip -> target with pose constraint
            ik_el_u, simToIk, _ = simIK.addElementFromScene(
                ikEnv, ikGroup_u, simBase, simTip, simTarget, simIK.constraint_pose
            )
            simIK.setElementConstraints(ikEnv, ikGroup_u, ik_el_u, simIK.constraint_pose)
            
            # Backup IK group using damped least squares (fallback)
            ikGroup_d = simIK.createGroup(ikEnv)
            simIK.setGroupCalculation(ikEnv, ikGroup_d, simIK.method_damped_least_squares, 0.3, 99)
            ik_el_d, _, _ = simIK.addElementFromScene(
                ikEnv, ikGroup_d, simBase, simTip, simTarget, simIK.constraint_pose
            )
            simIK.setElementConstraints(ikEnv, ikGroup_d, ik_el_d, simIK.constraint_pose)
            
            # Set all joints to IK mode
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
    """
    Read current joint positions from CoppeliaSim.
    
    Args:
        sim: CoppeliaSim API object
        joints: List of joint handles
        
    Returns:
        list: Joint angles in degrees (6 values)
    """
    positions = []
    for joint in joints:
        try:
            # Read joint position in radians, convert to degrees
            positions.append(math.degrees(sim.getJointPosition(joint)))
        except:
            positions.append(0.0)  # Default on error
    return positions


def get_robot_pose(sim, simTarget, simBase):
    """
    Get current robot end-effector position from CoppeliaSim.
    
    Args:
        sim: CoppeliaSim API object
        simTarget: Target object handle
        simBase: Base object handle (reference frame)
        
    Returns:
        list: [X, Y, Z] in millimeters
    """
    try:
        # Read position in meters (CoppeliaSim uses meters)
        pos = list(sim.getObjectPosition(simTarget, simBase))
        # Convert to millimeters for consistency with PLC data
        return [p * 1000 for p in pos]
    except:
        return [0, 0, 0]


def move_to_position(sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d, position):
    """
    Move the robot to a target position using inverse kinematics.
    
    Args:
        sim: CoppeliaSim API object
        simIK: CoppeliaSim IK API object
        simBase: Base object handle
        simTarget: Target object handle
        ikEnv: IK environment handle
        ikGroup_u: Primary IK group (pseudo-inverse)
        ikGroup_d: Backup IK group (damped least squares)
        position: [X, Y, Z, Angle] in millimeters and degrees
        
    Returns:
        bool: True if movement was successful
    """
    # Convert from millimeters to meters (CoppeliaSim uses meters)
    x = position[0] / 1000.0
    y = position[1] / 1000.0
    z = position[2] / 1000.0
    angle = math.radians(position[3])  # Convert degrees to radians
    
    # Update target position in CoppeliaSim
    sim.setObjectPosition(simTarget, simBase, [x, y, z])
    
    # Update target orientation (keep only Z-axis rotation)
    euler = list(sim.getObjectOrientation(simTarget, simBase))
    euler[2] = angle  # Set Z rotation
    sim.setObjectOrientation(simTarget, simBase, euler)
    
    # Apply IK using primary method
    result = simIK.handleIkGroup(ikEnv, ikGroup_u)
    
    # If primary fails, try backup method
    if result != simIK.result_success:
        result = simIK.handleIkGroup(ikEnv, ikGroup_d)
    
    return result == simIK.result_success


# ============================================================
# 5. DISPLAY FUNCTIONS - Beautiful Output Formatting
# ============================================================

def print_detailed_step(step_num, name, target_pos, robot_pos, joint_angles, status):
    """
    Print detailed information about a motion step in a formatted box.
    
    Args:
        step_num: Step number (1-4)
        name: Step name (Pos1, HmPos, Pos3, Pos4)
        target_pos: Target position from PLC [X, Y, Z, Angle]
        robot_pos: Actual robot position from CoppeliaSim [X, Y, Z]
        joint_angles: Joint angles from CoppeliaSim [J1-J6] in degrees
        status: Boolean indicating if target was reached
    """
    # Print header with separator lines
    print("\n" + "=" * 80)
    print(f"📍 STEP {step_num}: {name}".center(80))
    print("=" * 80)
    
    # Display PLC target position
    print("\n🎯 PLC TARGET:")
    print(f"   X: {target_pos[0]:8.1f} mm    Y: {target_pos[1]:8.1f} mm    Z: {target_pos[2]:8.1f} mm    Angle: {target_pos[3]:6.1f}°")
    
    # Display robot feedback from CoppeliaSim
    print("\n🤖 ROBOT FEEDBACK (CoppeliaSim):")
    print(f"   X: {robot_pos[0]:8.1f} mm    Y: {robot_pos[1]:8.1f} mm    Z: {robot_pos[2]:8.1f} mm")
    
    # Calculate and display position error
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]
    dz = target_pos[2] - robot_pos[2]
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    print("\n📊 POSITION ERROR:")
    print(f"   ΔX: {dx:+8.1f} mm    ΔY: {dy:+8.1f} mm    ΔZ: {dz:+8.1f} mm")
    print(f"   Distance: {distance:8.1f} mm")
    
    # Display all 6 joint angles from CoppeliaSim
    print("\n🔧 JOINT ANGLES (6 Axes):")
    if len(joint_angles) >= 6:
        # Print first 3 joints on first line
        for i in range(6):
            print(f"   J{i+1}: {joint_angles[i]:8.1f}°", end="")
            if i == 2:
                print()  # New line after J3
        print()  # Final new line
    
    # Display status with emoji
    print(f"\n📌 Status: {'✅ TARGET REACHED' if status else '⏳ MOVING...'}")
    print("=" * 80)


# ============================================================
# 6. MAIN FUNCTION - Program Entry Point
# ============================================================

def main():
    """
    Main program entry point.
    
    This function orchestrates the entire digital twin workflow:
    1. Starts OPC UA reader thread (sends commands to PLC)
    2. Sets up CoppeliaSim simulation
    3. Continuously follows PLC motion
    4. Displays detailed feedback when positions are reached
    """
    global current_position, running, motion_started
    
    # Print program header
    print("=" * 130)
    print("🏭 DIGITAL TWIN - Single Motion with Full Details".center(130))
    print("=" * 130)
    
    # ============================================================
    # Step 1: Start OPC UA Reader Thread
    # ============================================================
    print("\n🔄 Starting OPC UA...")
    opc_thread = threading.Thread(target=opcua_reader_thread, daemon=True)
    opc_thread.start()
    
    # Wait for motion to start (with timeout)
    print("\n⏳ Waiting for PLC motion to start...")
    timeout = 10
    while not motion_started and timeout > 0:
        time.sleep(0.5)
        timeout -= 0.5
    
    if not motion_started:
        print("❌ Motion did not start!")
        return
    
    # ============================================================
    # Step 2: Setup CoppeliaSim
    # ============================================================
    print("\n🔄 Setting up CoppeliaSim...")
    result = setup_coppelia()
    if result[0] is None:
        print("❌ Failed to start CoppeliaSim")
        return
    
    # Unpack CoppeliaSim objects
    sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d, joints = result
    
    print("\n" + "=" * 130)
    print("📊 FOLLOWING PLC MOTION WITH FULL DETAILS".center(130))
    print("=" * 130)
    
    # ============================================================
    # Step 3: Follow PLC Motion with Full Details
    # ============================================================
    
    # Track which steps have been reached (avoid duplicate displays)
    step_reached = [False] * len(MOTION_SEQUENCE)
    last_position = None
    
    try:
        # Main simulation loop
        while running and sim.getSimulationState() != sim.simulation_stopped:
            # Get current PLC position (thread-safe)
            with position_lock:
                plc_pos = list(current_position)
            
            # Check if PLC has reached any new target position
            for i, target in enumerate(MOTION_SEQUENCE):
                if step_reached[i]:
                    continue  # Already reported this position
                
                target_pos = target["position"]
                
                # Calculate distance from PLC position to target
                dx = target_pos[0] - plc_pos[0]
                dy = target_pos[1] - plc_pos[1]
                dz = target_pos[2] - plc_pos[2]
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # If PLC is within 10mm of target, it has reached the position
                if distance < 10.0:
                    print(f"\n🎯 PLC reached {target['name']}! Moving robot...")
                    
                    # Move robot in CoppeliaSim to the target position
                    success = move_to_position(sim, simIK, simBase, simTarget, 
                                               ikEnv, ikGroup_u, ikGroup_d, target_pos)
                    
                    # Get feedback from CoppeliaSim
                    robot_pos = get_robot_pose(sim, simTarget, simBase)
                    joint_angles = get_joint_positions(sim, joints)
                    
                    # Calculate final position error
                    dx2 = target_pos[0] - robot_pos[0]
                    dy2 = target_pos[1] - robot_pos[1]
                    dz2 = target_pos[2] - robot_pos[2]
                    distance2 = math.sqrt(dx2*dx2 + dy2*dy2 + dz2*dz2)
                    reached = distance2 < 2.0  # Within 2mm
                    
                    # Display detailed information
                    print_detailed_step(i+1, target["name"], target_pos, 
                                      robot_pos, joint_angles, reached)
                    
                    # Mark step as completed
                    step_reached[i] = True
                    
                    # Check if all steps are complete
                    if all(step_reached):
                        print("\n" + "=" * 130)
                        print("✅ ALL MOTIONS COMPLETED!".center(130))
                        print("=" * 130)
                    
                    break  # Only process one step at a time
            
            # Always update robot position in real-time to follow PLC
            if plc_pos and plc_pos != last_position:
                # Convert PLC position (mm) to CoppeliaSim coordinates (m)
                x = plc_pos[0] / 1000.0
                y = plc_pos[1] / 1000.0
                z = plc_pos[2] / 1000.0
                angle = math.radians(plc_pos[3])
                
                # Update target object position and orientation
                sim.setObjectPosition(simTarget, simBase, [x, y, z])
                euler = list(sim.getObjectOrientation(simTarget, simBase))
                euler[2] = angle
                sim.setObjectOrientation(simTarget, simBase, euler)
                
                # Apply inverse kinematics
                result_ik = simIK.handleIkGroup(ikEnv, ikGroup_u)
                if result_ik != simIK.result_success:
                    # Fallback to damped least squares if primary IK fails
                    simIK.handleIkGroup(ikEnv, ikGroup_d)
                
                last_position = list(plc_pos)
            
            # Advance simulation by one step
            sim_client.step()
            time.sleep(0.02)  # 20ms sleep to prevent CPU overload
            
    except KeyboardInterrupt:
        print("\n⏹️ Stopping...")
    
    # ============================================================
    # Step 4: Cleanup and Exit
    # ============================================================
    running = False
    sim.stopSimulation()  # Stop CoppeliaSim simulation
    print("\n👋 Done")


# ============================================================
# 7. PROGRAM ENTRY POINT
# ============================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️ Stopped by user")
        running = False