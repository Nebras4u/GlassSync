#!/usr/bin/env python3
"""
run_plc.py - تشغيل متكامل مع PLC (نسخة مصححة)
"""

import sys
import os
import asyncio
import math
import time
import queue
import threading
from datetime import datetime
from asyncua import Client, ua
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ============================================================
# 1. إعدادات OPC UA
# ============================================================

PLC_IP = "192.168.0.1"
SERVER_URL = f"opc.tcp://{PLC_IP}:4840"

points_update_queue = queue.Queue()

current_plc_data = {
    "EnableAll": False,
    "EnableKinematics": False,
    "Vlcty": 0.0,
    "Live_XYZ_Angle": [0.0, 0.0, 0.0, 0.0]
}


class OPCUASubscriptionHandler:
    """معالج اشتراكات OPC UA مع الدوال المطلوبة"""
    
    def __init__(self, enable_node, kin_node, vlcty_node, pos_node):
        self.enable_id = enable_node.nodeid
        self.kin_id = kin_node.nodeid
        self.vlcty_id = vlcty_node.nodeid
        self.pos_id = pos_node.nodeid

    def datachange_notification(self, node, val, data):
        """استقبال التغييرات من PLC"""
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
        except Exception as e:
            print(f"⚠️ Data handler error: {e}")

    # ✅ الدالة المفقودة التي تسبب الخطأ
    def status_change_notification(self, status):
        """معالجة تغيير حالة الاشتراك"""
        print(f"📡 Subscription status: {status}")


async def run_opcua_client():
    """تشغيل عميل OPC UA"""
    print(f"🔌 [OPC UA] Connecting to PLC at {SERVER_URL}...")
    
    while True:
        try:
            async with Client(url=SERVER_URL, timeout=5) as client:
                client.keep_alive_interval = 2
                
                enable_all_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableAll"')
                enable_kin_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
                vlcty_node = client.get_node('ns=3;s="TO_Data_block_1"."Vlcty"')
                live_tcp_node = client.get_node('ns=3;s="TO_Data_block_1"."Live_XYZ_Angle"')
                
                print("⚡ [PLC] Enabling system...")
                
                async def write_bool_no_timestamp(node, value):
                    dv = ua.DataValue(ua.Variant(value, ua.VariantType.Boolean))
                    dv.SourceTimestamp = None
                    dv.ServerTimestamp = None
                    await node.write_value(dv)
                
                await write_bool_no_timestamp(enable_all_node, True)
                await write_bool_no_timestamp(enable_kin_node, True)
                print("✅ [PLC] System enabled")
                
                handler = OPCUASubscriptionHandler(
                    enable_all_node, enable_kin_node, vlcty_node, live_tcp_node
                )
                sub = await client.create_subscription(500, handler)
                
                await sub.subscribe_data_change(enable_all_node)
                await sub.subscribe_data_change(enable_kin_node)
                await sub.subscribe_data_change(vlcty_node)
                await sub.subscribe_data_change(live_tcp_node)
                
                print("✅ [OPC UA] Subscription active")
                
                while True:
                    try:
                        await asyncio.sleep(1)
                    except Exception as e:
                        print(f"⚠️ Connection lost: {e}")
                        break
                    
        except Exception as e:
            print(f"\n⚠️ [OPC UA] Error: {e}")
            print("🔄 Retrying in 5 seconds...")
            await asyncio.sleep(5)


def start_opcua_thread():
    """تشغيل عميل OPC UA في خيط منفصل"""
    asyncio.run(run_opcua_client())


# ============================================================
# 2. CoppeliaSim
# ============================================================

def setup_coppelia():
    """إعداد CoppeliaSim"""
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


def run_coppelia_engine():
    """تشغيل CoppeliaSim"""
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
                time.sleep(0.001)
                
    except KeyboardInterrupt:
        print("\n⏹️ [CoppeliaSim] Stopping...")
    
    sim.stopSimulation()
    print("👋 [CoppeliaSim] Stopped")


# ============================================================
# 3. إرسال أمر من Python
# ============================================================

async def send_go2pos(angles):
    """إرسال Go2Pos + تفعيل EnableKinematics"""
    try:
        async with Client(SERVER_URL, timeout=5) as client:
            go2pos_node = client.get_node('ns=3;s="TO_Data_block_1"."Go2Pos"')
            enable_kin_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
            vlcty_node = client.get_node('ns=3;s="TO_Data_block_1"."Vlcty"')
            
            print(f"📍 {angles}")
            
            # 1. Go2Pos
            variant = ua.Variant(angles, ua.VariantType.Double)
            dv = ua.DataValue(variant)
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await go2pos_node.write_value(dv)
            
            # 2. Vlcty
            dv = ua.DataValue(ua.Variant(50.0, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await vlcty_node.write_value(dv)
            
            # 3. إيقاف
            dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            await asyncio.sleep(0.2)
            
            # 4. تشغيل
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            
            print("   ✅ Done")
            return True
    except Exception as e:
        print(f"❌ {e}")
        return False


# ============================================================
# 4. Main
# ============================================================

def main():
    print("=" * 70)
    print("🏭 DIGITAL TWIN V6.1 - Integrated Runner".center(70))
    print("=" * 70)
    
    opc_thread = threading.Thread(target=start_opcua_thread, daemon=True)
    opc_thread.start()
    
    run_coppelia_engine()
    
    # انتظار 3 ثواني ثم إرسال الأمر
    time.sleep(3)
    asyncio.run(send_go2pos([777.0, 0.0, 130.0, 0.0]))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️ Stopped by user")
