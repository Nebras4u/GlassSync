#!/usr/bin/env python3
"""
run_plc_direct.py - تشغيل مباشر بدون اشتراك OPC UA (مصحح)
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
# 1. إعدادات
# ============================================================

PLC_IP = "192.168.0.1"
SERVER_URL = f"opc.tcp://{PLC_IP}:4840"

# متغيرات مشتركة بين الخيوط
current_position = [0.0, 0.0, 0.0, 0.0]
position_lock = threading.Lock()
running = True

# ============================================================
# 2. OPC UA - قراءة مباشرة (مع asyncio)
# ============================================================

def opcua_reader():
    """خيط منفصل لقراءة البيانات من PLC مباشرة"""
    global current_position, running
    
    # إنشاء حلقة asyncio جديدة لهذا الخيط
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    while running:
        try:
            # تشغيل دالة القراءة بشكل متزامن
            result = loop.run_until_complete(read_plc_position())
            if result:
                with position_lock:
                    current_position = result
        except Exception as e:
            print(f"⚠️ OPC UA error: {e}")
        
        # انتظر 0.05 ثانية (20 مرة في الثانية)
        time.sleep(0.05)
    
    loop.close()

async def read_plc_position():
    """قراءة الموضع من PLC"""
    try:
        client = Client(SERVER_URL, timeout=3)
        await client.connect()
        
        try:
            live_node = client.get_node('ns=3;s="TO_Data_block_1"."Live_XYZ_Angle"')
            value = await live_node.read_value()
            
            if value and len(value) >= 4:
                return list(value)
        finally:
            await client.disconnect()
            
    except Exception as e:
        pass
    
    return None

# ============================================================
# 3. CoppeliaSim
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

def run_coppelia():
    """تشغيل CoppeliaSim مع قراءة مباشرة"""
    global current_position, running
    
    result = setup_coppelia()
    if result[0] is None:
        print("❌ Failed to start CoppeliaSim")
        return
    
    sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d = result
    
    print("🚀 [CoppeliaSim] Running...")
    print("📡 انتظار البيانات من PLC...")
    
    last_position = None
    counter = 0
    no_data_count = 0
    
    try:
        while sim.getSimulationState() != sim.simulation_stopped and running:
            # قراءة أحدث موضع
            with position_lock:
                pos = list(current_position)
            
            # التحقق من وجود بيانات
            if pos == [0.0, 0.0, 0.0, 0.0]:
                no_data_count += 1
                if no_data_count % 50 == 0:  # كل 50 دورة
                    print("⏳ في انتظار البيانات من PLC...")
            else:
                no_data_count = 0
            
            # التحقق من تغير الموضع
            if pos != last_position and pos != [0.0, 0.0, 0.0, 0.0]:
                # تحويل البيانات: PLC تعطي بالميليمتر، CoppeliaSim بالمتر
                x = pos[0] / 1000.0
                y = pos[1] / 1000.0
                z = pos[2] / 1000.0
                angle = math.radians(pos[3])
                
                # تحديث target في CoppeliaSim
                sim.setObjectPosition(simTarget, simBase, [x, y, z])
                
                euler = list(sim.getObjectOrientation(simTarget, simBase))
                euler[2] = angle
                sim.setObjectOrientation(simTarget, simBase, euler)
                
                # تطبيق IK
                result = simIK.handleIkGroup(ikEnv, ikGroup_u)
                if result != simIK.result_success:
                    simIK.handleIkGroup(ikEnv, ikGroup_d)
                
                last_position = list(pos)
                counter += 1
                if counter % 10 == 0:  # كل 10 تحديثات
                    print(f"📍 X:{x:.3f} Y:{y:.3f} Z:{z:.3f} A:{pos[3]:.1f}°")
            
            # خطوة المحاكاة
            sim_client.step()
            time.sleep(0.01)  # 10ms
            
    except KeyboardInterrupt:
        print("\n⏹️ Stopping...")
    
    sim.stopSimulation()
    print("👋 [CoppeliaSim] Stopped")

# ============================================================
# 4. إرسال أمر الحركة إلى PLC
# ============================================================

async def send_motion_command():
    """إرسال أمر الحركة إلى PLC"""
    try:
        async with Client(SERVER_URL, timeout=5) as client:
            print("📝 إرسال أمر الحركة إلى PLC...")
            
            # العقد
            pos1_node = client.get_node('ns=3;s="TO_Data_block_1"."Pos1"')
            hmpos_node = client.get_node('ns=3;s="TO_Data_block_1"."HmPos"')
            pos3_node = client.get_node('ns=3;s="TO_Data_block_1"."Pos3"')
            pos4_node = client.get_node('ns=3;s="TO_Data_block_1"."Pos4"')
            vlcty_node = client.get_node('ns=3;s="TO_Data_block_1"."Vlcty"')
            enable_all_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableAll"')
            enable_kin_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
            
            # نقاط مسار (بالملليمتر)
            pos1 = [150.0, 80.0, 250.0, 10.0]
            hmpos = [250.0, 130.0, 350.0, 55.0]
            pos3 = [350.0, 180.0, 450.0, 100.0]
            pos4 = [450.0, 230.0, 550.0, 145.0]
            
            def write_array(node, values):
                dv = ua.DataValue(ua.Variant(values, ua.VariantType.Double))
                dv.SourceTimestamp = None
                dv.ServerTimestamp = None
                return node.write_value(dv)
            
            await write_array(pos1_node, pos1)
            print(f"   Pos1 = {pos1}")
            
            await write_array(hmpos_node, hmpos)
            print(f"   HmPos = {hmpos}")
            
            await write_array(pos3_node, pos3)
            print(f"   Pos3 = {pos3}")
            
            await write_array(pos4_node, pos4)
            print(f"   Pos4 = {pos4}")
            
            # السرعة
            dv = ua.DataValue(ua.Variant(100.0, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await vlcty_node.write_value(dv)
            print("   Vlcty = 100.0")
            
            # EnableAll
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_all_node.write_value(dv)
            print("   EnableAll = True")
            
            await asyncio.sleep(0.5)
            
            # Rising Edge
            dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            print("   EnableKinematics = False")
            await asyncio.sleep(0.5)
            
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            print("   EnableKinematics = True ✅")
            
            print("✅ تم إرسال أمر الحركة!")
            
    except Exception as e:
        print(f"❌ خطأ: {e}")

# ============================================================
# 5. Main
# ============================================================

def main():
    global running
    
    print("=" * 70)
    print("🏭 DIGITAL TWIN - Direct Control (Fixed)".center(70))
    print("=" * 70)
    
    # ==========================================
    # 1. تشغيل OPC UA Reader في خيط منفصل
    # ==========================================
    print("\n🔄 بدء قراءة OPC UA...")
    opc_thread = threading.Thread(target=opcua_reader, daemon=True)
    opc_thread.start()
    time.sleep(2)
    
    # ==========================================
    # 2. إرسال أمر الحركة
    # ==========================================
    asyncio.run(send_motion_command())
    time.sleep(1)
    
    # ==========================================
    # 3. تشغيل CoppeliaSim
    # ==========================================
    print("\n🔄 بدء CoppeliaSim...")
    run_coppelia()
    
    running = False
    print("\n👋 تم الإنهاء")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n⏹️ Stopped by user")
        running = False