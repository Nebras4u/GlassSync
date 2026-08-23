#!/usr/bin/env python3
"""
robot_control.py - ABB IRB140 مع Subscription (يعمل)
"""

import sys
import asyncio
import math
import time
import queue
import threading
import numpy as np
from asyncua import Client
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ============================================================
# 1. IK LIBRARY
# ============================================================

try:
    import ikpy
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
    print("✅ IK library loaded")
except ImportError:
    print("📦 Installing ikpy...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "ikpy"])
    import ikpy
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
    print("✅ IK library installed")

# ============================================================
# 2. CONFIGURATION
# ============================================================

PLC_IP = "192.168.0.1"
SERVER_URL = f"opc.tcp://{PLC_IP}:4840"

points_update_queue = queue.Queue()

# ============================================================
# 3. IK CHAIN - IRB140
# ============================================================

def create_robot_chain():
    chain = Chain([
        OriginLink(),
        URDFLink(
            name="joint1",
            origin_translation=[0, 0, 0.352],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            bounds=[-3.14, 3.14]
        ),
        URDFLink(
            name="joint2",
            origin_translation=[0, 0, 0.070],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
            bounds=[-1.57, 1.57]
        ),
        URDFLink(
            name="joint3",
            origin_translation=[0.360, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
            bounds=[-2.5, 1.5]
        ),
        URDFLink(
            name="joint4",
            origin_translation=[0.380, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[1, 0, 0],
            bounds=[-7.0, 7.0]
        ),
        URDFLink(
            name="joint5",
            origin_translation=[0, 0, 0.2],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
            bounds=[-2.5, 2.5]
        ),
        URDFLink(
            name="joint6",
            origin_translation=[0, 0, 0.15],
            origin_orientation=[0, 0, 0],
            rotation=[1, 0, 0],
            bounds=[-7.0, 7.0]
        )
    ])
    return chain

print("\n🔧 Building IK model...")
ik_chain = create_robot_chain()
print(f"✅ IK model built ({len(ik_chain.links)} links)")

# ============================================================
# 4. IK SOLVER
# ============================================================

class IKSolver:
    def __init__(self, chain):
        self.chain = chain
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    def solve(self, target_position_m):
        try:
            target = np.array(target_position_m[:3])
            initial = [0.0] + list(self.current_angles)
            angles = self.chain.inverse_kinematics(
                target,
                initial_position=initial,
                orientation_mode=None
            )
            self.current_angles = angles[1:]
            return np.array(angles[1:])
        except Exception as e:
            print(f"⚠️ IK Error: {e}")
            return None

ik_solver = IKSolver(ik_chain)

# ============================================================
# 5. OPC UA - SUBSCRIPTION
# ============================================================

class SubscriptionHandler:
    def __init__(self):
        self.last_values = None
        
    def datachange_notification(self, node, val, data):
        try:
            if hasattr(val, '__iter__') and not isinstance(val, (str, bytes)):
                actpos = list(val)
            else:
                actpos = [val]
            
            if len(actpos) >= 4:
                x_raw = float(actpos[0])
                y_raw = float(actpos[1])
                z_raw = float(actpos[2])
                angle = float(actpos[3])
                
                current = (x_raw, y_raw, z_raw, angle)
                
                if self.last_values is None or current != self.last_values:
                    self.last_values = current
                    
                    x = x_raw / 1000.0
                    y = y_raw / 1000.0
                    z = z_raw / 1000.0
                    
                    processed_point = {
                        "pos": [x, y, z],
                        "angle": math.radians(angle)
                    }
                    points_update_queue.put(processed_point)
                    
                    print(f"\n📥 ActPos from PLC:")
                    print(f"   Raw: X={x_raw:.3f} mm, Y={y_raw:.3f} mm, Z={z_raw:.3f} mm")
                    print(f"   To Coppelia: X={x:.6f} m, Y={y:.6f} m, Z={z:.6f} m")
                    
        except Exception as e:
            print(f"⚠️ Data handler error: {e}")
    
    def status_change_notification(self, status):
        pass


async def run_opcua_subscription():
    print(f"🔌 [OPC UA] Connecting to PLC at {SERVER_URL}...")
    
    while True:
        client = None
        subscription = None
        
        try:
            client = Client(url=SERVER_URL, timeout=10)
            client.session_timeout = 60000
            client.keep_alive_interval = 5
            
            await client.connect()
            print("⚡ [PLC] OPC UA Connected")
            
            actpos_node = client.get_node('ns=3;s="ABB-IRB-140"."ActPos"')
            
            try:
                current_value = await actpos_node.read_value()
                print(f"📊 Current ActPos: {current_value}")
            except Exception as e:
                print(f"⚠️ Could not read ActPos: {e}")
            
            handler = SubscriptionHandler()
            subscription = await client.create_subscription(500, handler)
            await subscription.subscribe_data_change(actpos_node)
            
            print("✅ [OPC UA] Subscription active (500ms)")
            print("📐 ActPos[1..3] = X, Y, Z (mm) → متر\n")
            
            while True:
                await asyncio.sleep(1)
                
        except asyncio.CancelledError:
            print("\n🛑 OPC UA stopped")
            break
        except Exception as e:
            print(f"\n⚠️ [OPC UA] Error: {e}")
            print("🔄 Reconnecting in 3 seconds...")
            
            if subscription:
                try: await subscription.delete()
                except: pass
            if client:
                try: await client.disconnect()
                except: pass
            
            await asyncio.sleep(3)


def start_opcua_thread():
    asyncio.run(run_opcua_subscription())


# ============================================================
# 6. CoppeliaSim
# ============================================================

def setup_coppelia():
    sim_client = RemoteAPIClient()
    sim = sim_client.require('sim')
    simIK = sim_client.require('simIK')
    
    sim_client.setStepping(True)
    sim.startSimulation()
    sim.setFloatParam(sim.floatparam_simulation_time_step, 0.01)
    
    all_joints = sim.getObjectsInTree(sim.handle_scene, sim.object_joint_type)
    
    robot_joints = []
    for joint in all_joints:
        alias = sim.getObjectAlias(joint)
        if alias and "aux" not in alias.lower():
            sim.setJointMode(joint, sim.jointmode_force, 0)
            sim.setJointInterval(joint, True, [-3.0, 3.0])
            sim.setJointTargetVelocity(joint, 0.0)
            sim.setJointTargetPosition(joint, 0.0)
            sim.setJointMaxForce(joint, 200.0)
            robot_joints.append(joint)
    
    all_objects = sim.getObjectsInTree(sim.handle_scene)
    simBase, simTip, simTarget = None, None, None
    
    for obj in all_objects:
        alias = sim.getObjectAlias(obj)
        if not alias:
            continue
        if 'IRB140' in alias or 'irb140' in alias.lower():
            if simBase is None:
                simBase = obj
        elif 'target' in alias.lower():
            simTarget = obj
        elif 'tip' in alias.lower():
            simTip = obj
    
    if not simBase and robot_joints:
        simBase = sim.getObjectParent(robot_joints[0])
    
    print(f"   Found {len(robot_joints)} joints for robot")
    
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
        
        for joint in robot_joints:
            if joint in simToIk:
                simIK.setJointMode(ikEnv, simToIk[joint], simIK.jointmode_ik)
        
        print("✅ [CoppeliaSim] Ready with IK")
        return sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d
    
    print("⚠️ [CoppeliaSim] Using direct joint control")
    return sim_client, sim, None, None, None, None, None, None


def run_coppelia_engine():
    result = setup_coppelia()
    
    if result[0] is None:
        print("❌ Failed to start CoppeliaSim")
        return
    
    sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d = result
    
    print("🚀 [CoppeliaSim] Running...")
    
    try:
        while sim.getSimulationState() != sim.simulation_stopped:
            latest_point = None
            
            while not points_update_queue.empty():
                latest_point = points_update_queue.get_nowait()
            
            if latest_point is not None:
                pos = latest_point["pos"]
                angle = latest_point["angle"]
                
                if simBase and simTarget:
                    sim.setObjectPosition(simTarget, simBase, pos)
                    
                    euler = list(sim.getObjectOrientation(simTarget, simBase))
                    euler[2] = angle
                    sim.setObjectOrientation(simTarget, simBase, euler)
                    
                    if simIK and ikEnv and ikGroup_u:
                        result = simIK.handleIkGroup(ikEnv, ikGroup_u)
                        if result != simIK.result_success:
                            simIK.handleIkGroup(ikEnv, ikGroup_d)
                    else:
                        print(f"   Direct: X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f}")
                
                sim_client.step()
            else:
                sim_client.step()
                time.sleep(0.001)
                
    except KeyboardInterrupt:
        print("\n⏹️ [CoppeliaSim] Stopping...")
    
    sim.stopSimulation()
    print("👋 [CoppeliaSim] Stopped")


# ============================================================
# 7. Main
# ============================================================

def main():
    print("=" * 70)
    print("🏭 DIGITAL TWIN - IRB140 (Subscription)".center(70))
    print("=" * 70)
    
    print(f"\n📡 OPC UA: {SERVER_URL}")
    print("📐 ActPos[1..3] = X, Y, Z (mm) → متر")
    print("📌 Using Industrial Subscription (500ms)")
    
    print("\n🔄 Starting OPC UA...")
    opc_thread = threading.Thread(target=start_opcua_thread, daemon=True)
    opc_thread.start()
    time.sleep(3)
    
    print("\n🔄 Starting CoppeliaSim...")
    sim_thread = threading.Thread(target=run_coppelia_engine, daemon=True)
    sim_thread.start()
    
    print("\n✅ SYSTEM READY")
    print("📌 غير ActPos في PLC وسيتحرك الروبوت!")
    print("   Press Ctrl+C to stop\n")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n⏹️ Stopped")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️ Stopped")