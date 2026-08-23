#!/usr/bin/env python3
"""
run_manual_plc.py - Manual Control + CoppeliaSim Mirror (All-in-One)
"""

import sys
import os
import asyncio
import math
import time
import queue
import threading
from asyncua import Client, ua
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ============================================================
# Configuration
# ============================================================

PLC_IP = "192.168.0.1"
SERVER_URL = f"opc.tcp://{PLC_IP}:4840"

points_update_queue = queue.Queue()

# Predefined points (in degrees)
POINTS = {
    "1": {"name": "Home", "angles": [0.0, 0.0, 0.0, 0.0]},
    "2": {"name": "Pick", "angles": [30.0, 20.0, -15.0, 10.0]},
    "3": {"name": "Place", "angles": [-20.0, 30.0, 15.0, -10.0]},
    "4": {"name": "Mid", "angles": [45.0, 20.0, 0.0, 0.0]},
    "5": {"name": "Max", "angles": [60.0, 30.0, 20.0, 15.0]},
}


# ============================================================
# OPC UA Subscription Handler
# ============================================================

class OPCUASubscriptionHandler:
    def __init__(self, pos_node):
        self.pos_id = pos_node.nodeid

    def datachange_notification(self, node, val, data):
        try:
            if node.nodeid == self.pos_id:
                incoming_list = list(val) if isinstance(val, list) else val
                if len(incoming_list) >= 4:
                    processed_point = {
                        "pos": [
                            incoming_list[0] / 1000.0,
                            incoming_list[1] / 1000.0,
                            incoming_list[2] / 1000.0
                        ],
                        "angle": math.radians(float(incoming_list[3]))
                    }
                    points_update_queue.put(processed_point)
        except Exception:
            pass

    def status_change_notification(self, status):
        pass


# ============================================================
# OPC UA Write Functions
# ============================================================

async def write_no_timestamp(node, value):
    if isinstance(value, bool):
        dv = ua.DataValue(ua.Variant(value, ua.VariantType.Boolean))
    elif isinstance(value, (int, float)):
        dv = ua.DataValue(ua.Variant(float(value), ua.VariantType.Double))
    else:
        dv = ua.DataValue(ua.Variant(value, ua.VariantType.String))
    dv.SourceTimestamp = None
    dv.ServerTimestamp = None
    await node.write_value(dv)


async def write_array_no_timestamp(node, values):
    variant = ua.Variant(values, ua.VariantType.Double)
    dv = ua.DataValue(variant)
    dv.SourceTimestamp = None
    dv.ServerTimestamp = None
    await node.write_value(dv)


async def read_array_no_timestamp(node, count=4):
    try:
        val = await node.read_value()
        if isinstance(val, (list, tuple)):
            return list(val)[:count]
        return [float(val)] if val is not None else [0.0] * count
    except:
        return [0.0] * count


# ============================================================
# CoppeliaSim Mirror
# ============================================================

def setup_coppelia():
    sim_client = RemoteAPIClient()
    sim = sim_client.require('sim')
    simIK = sim_client.require('simIK')

    sim_client.setStepping(True)
    sim.startSimulation()
    sim.setFloatParam(sim.floatparam_simulation_time_step, 0.01)

    try:
        sim.setGravity(0.0, 0.0, 0.0)
    except:
        pass

    all_objects = sim.getObjectsInTree(sim.handle_scene)
    simBase, simTip, simTarget = None, None, None

    for obj in all_objects:
        alias = sim.getObjectAlias(obj)
        if not alias:
            continue
        if 'IRB140' in alias:
            simBase = obj
        elif 'target' in alias.lower():
            simTarget = obj
        elif 'tip' in alias.lower():
            simTip = obj

    all_joints = sim.getObjectsInTree(sim.handle_scene, sim.object_joint_type)

    if not simBase and all_joints:
        simBase = sim.getObjectParent(all_joints[0])

    for j in all_joints:
        sim.setJointMode(j, sim.jointmode_force, 0)
        sim.setJointInterval(j, True, [-3.0, 3.0])
        sim.setJointTargetVelocity(j, 0.0)
        sim.setJointTargetPosition(j, 0.0)
        sim.setJointMaxForce(j, 200.0)

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
        return sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d

    print("❌ [CoppeliaSim] IK setup failed")
    return None, None, None, None, None, None, None, None


async def run_coppelia_engine():
    """Run CoppeliaSim mirror in a separate task"""
    result = setup_coppelia()

    if result[0] is None:
        print("❌ Failed to start CoppeliaSim")
        return

    sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d = result

    print("🚀 [CoppeliaSim] Running...")

    while True:
        try:
            latest_point = None

            while not points_update_queue.empty():
                latest_point = points_update_queue.get_nowait()

            if latest_point is not None:
                pos = latest_point["pos"]
                angle = latest_point["angle"]

                sim.setObjectPosition(simTarget, simBase, pos)

                euler = list(sim.getObjectOrientation(simTarget, simBase))
                euler[2] = angle
                sim.setObjectOrientation(simTarget, simBase, euler)

                result = simIK.handleIkGroup(ikEnv, ikGroup_u)
                if result != simIK.result_success:
                    simIK.handleIkGroup(ikEnv, ikGroup_d)

                sim_client.step()
            else:
                sim_client.step()

            await asyncio.sleep(0.01)

        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"⚠️ CoppeliaSim error: {e}")
            await asyncio.sleep(0.1)

    sim.stopSimulation()
    print("👋 [CoppeliaSim] Stopped")


# ============================================================
# Main
# ============================================================

async def main():
    print("=" * 70)
    print("🤖 MANUAL CONTROL + COPPELIASIM MIRROR")
    print("=" * 70)

    # Connect to PLC
    print("🔌 Connecting to PLC...")
    async with Client(SERVER_URL, timeout=5) as client:
        print("✅ Connected!")

        # Get nodes
        go2pos_node = client.get_node('ns=3;s="TO_Data_block_1"."Go2Pos"')
        enable_kin_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
        enable_all_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableAll"')
        vlcty_node = client.get_node('ns=3;s="TO_Data_block_1"."Vlcty"')
        live_tcp_node = client.get_node('ns=3;s="TO_Data_block_1"."Live_XYZ_Angle"')
        act_pos_node = client.get_node('ns=3;s="TO_Data_block_1"."ActPos"')

        # Enable system
        print("⚡ Enabling system...")
        await write_no_timestamp(enable_all_node, True)
        await write_no_timestamp(enable_kin_node, True)
        await write_no_timestamp(vlcty_node, 50.0)

        # Subscribe to Live_XYZ_Angle
        handler = OPCUASubscriptionHandler(live_tcp_node)
        sub = await client.create_subscription(500, handler)
        await sub.subscribe_data_change(live_tcp_node)
        print("✅ [OPC UA] Subscription active")

        # Start CoppeliaSim in background
        coppelia_task = asyncio.create_task(run_coppelia_engine())

        # Manual control
        print("\n" + "-" * 60)
        print("🎮 Select a point to move to:")
        for key, point in POINTS.items():
            print(f"  {key}. {point['name']}: {point['angles']}°")
        print("  'status' to show current position")
        print("  'exit' to quit")
        print("-" * 60)

        try:
            while True:
                user_input = input("\nSelect point (1-5): ").strip()
                if user_input.lower() == 'exit':
                    break

                if user_input.lower() == 'status':
                    current = await read_array_no_timestamp(act_pos_node)
                    print(f"📊 Current position: {current}")
                    continue

                if user_input not in POINTS:
                    print("⚠️ Invalid selection. Choose 1-5")
                    continue

                point = POINTS[user_input]
                target = point["angles"]
                print(f"📍 Moving to: {point['name']} → {target}°")

                # Write to Go2Pos
                await write_array_no_timestamp(go2pos_node, target)

                # Toggle EnableKinematics
                await write_no_timestamp(enable_kin_node, False)
                await asyncio.sleep(0.2)
                await write_no_timestamp(enable_kin_node, True)

                # Wait
                await asyncio.sleep(0.5)

                # Show current position
                current = await read_array_no_timestamp(act_pos_node)
                print(f"📊 Current position: {current}")

        except KeyboardInterrupt:
            print("\n⏹️ Interrupted")

        finally:
            coppelia_task.cancel()
            await write_no_timestamp(enable_all_node, False)
            await write_no_timestamp(enable_kin_node, False)

    print("\n👋 Goodbye!")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n⏹️ Stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")