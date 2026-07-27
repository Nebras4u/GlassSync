#!/usr/bin/env python3
"""
run_plc_m99_single.py - استخدام اتصال واحد فقط
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
# إعدادات
# ============================================================

PLC_IP = "192.168.0.1"
SERVER_URL = f"opc.tcp://{PLC_IP}:4840"

points_update_queue = queue.Queue()
opcua_running = True

# ============================================================
# 1. نقاط محددة
# ============================================================

POINTS = {
    "1": {"name": "Home", "angles": [777.0, 0.0, 130.0, 0.0]},
    "2": {"name": "Pos1", "angles": [930.0, 0.0, 370.0, 0.0]},
    "3": {"name": "Pos3", "angles": [925.0, 192.0, 99.0, 0.0]},
    "4": {"name": "Pos4", "angles": [550.0, 500.0, 550.0, 30.0]},
    "5": {"name": "Go2Pos", "angles": [550.0, 440.0, 330.0, 22.0]},
}

# ============================================================
# 2. OPC UA Subscription Handler
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
# 3. العميل الرئيسي - اتصال واحد
# ============================================================

class PLCClient:
    """عميل PLC مع اتصال واحد فقط"""
    
    def __init__(self):
        self.client = None
        self._connected = False
        self._lock = asyncio.Lock()
        
        # العقد المخزنة
        self.go2pos_node = None
        self.enable_kin_node = None
        self.enable_all_node = None
        self.vlcty_node = None
        self.m99_node = None
        self.live_tcp_node = None
        self.act_pos_node = None
    
    async def connect(self):
        """الاتصال بـ PLC مرة واحدة فقط"""
        if self._connected:
            return True
        
        async with self._lock:
            if self._connected:
                return True
            
            try:
                self.client = Client(SERVER_URL, timeout=5)
                await self.client.connect()
                self._connected = True
                
                # تخزين العقد
                self.go2pos_node = self.client.get_node('ns=3;s="TO_Data_block_1"."Go2Pos"')
                self.enable_kin_node = self.client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
                self.enable_all_node = self.client.get_node('ns=3;s="TO_Data_block_1"."EnableAll"')
                self.vlcty_node = self.client.get_node('ns=3;s="TO_Data_block_1"."Vlcty"')
                self.m99_node = self.client.get_node('ns=3;s="M99.4"')
                self.live_tcp_node = self.client.get_node('ns=3;s="TO_Data_block_1"."Live_XYZ_Angle"')
                self.act_pos_node = self.client.get_node('ns=3;s="TO_Data_block_1"."ActPos"')
                
                print("✅ [PLC] متصل")
                return True
                
            except Exception as e:
                print(f"❌ [PLC] فشل الاتصال: {e}")
                return False
    
    async def enable_system(self):
        """تمكين النظام"""
        if not self._connected:
            return False
        
        try:
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await self.enable_all_node.write_value(dv)
            await self.enable_kin_node.write_value(dv)
            print("✅ [PLC] النظام مفعل")
            return True
        except Exception as e:
            print(f"❌ فشل تمكين النظام: {e}")
            return False
    
    async def subscribe_live_position(self):
        """الاشتراك في Live_XYZ_Angle"""
        if not self._connected:
            return False
        
        try:
            handler = OPCUASubscriptionHandler(self.live_tcp_node)
            sub = await self.client.create_subscription(500, handler)
            await sub.subscribe_data_change(self.live_tcp_node)
            print("✅ [OPC UA] اشتراك نشط")
            return True
        except Exception as e:
            print(f"❌ فشل الاشتراك: {e}")
            return False
    
    async def execute_motion(self, angles):
        """تنفيذ الحركة باستخدام M99.4"""
        if not self._connected:
            print("❌ غير متصل بـ PLC")
            return False
        
        try:
            print(f"\n📍 تنفيذ الحركة إلى: {angles}")
            
            # 1. التأكد من EnableKinematics = TRUE
            print("   🔍 التأكد من EnableKinematics = TRUE...")
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await self.enable_kin_node.write_value(dv)
            
            # 2. كتابة Go2Pos
            print("   📝 كتابة Go2Pos...")
            variant = ua.Variant(angles, ua.VariantType.Double)
            dv = ua.DataValue(variant)
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await self.go2pos_node.write_value(dv)
            
            # 3. تعيين السرعة
            print("   📝 تعيين Vlcty = 50.0...")
            dv = ua.DataValue(ua.Variant(50.0, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await self.vlcty_node.write_value(dv)
            
            # 4. قراءة M99.4 الحالية
            current = await self.m99_node.read_value()
            print(f"   📊 M99.4 الحالية: {current}")
            
            # 5. توليد نبضة M99.4 (FALSE → TRUE → FALSE)
            print("   🔄 توليد نبضة M99.4...")
            
            dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await self.m99_node.write_value(dv)
            await asyncio.sleep(0.1)
            print("   ✅ M99.4 = FALSE")
            
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await self.m99_node.write_value(dv)
            await asyncio.sleep(0.1)
            print("   ✅ M99.4 = TRUE (نبضة)")
            
            dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await self.m99_node.write_value(dv)
            print("   ✅ M99.4 = FALSE")
            
            await asyncio.sleep(0.3)
            
            # 6. قراءة ActPos
            act_pos = await self.act_pos_node.read_value()
            if act_pos:
                print(f"   📊 ActPos بعد الحركة: {act_pos}")
            
            print("✅ تم تنفيذ الحركة")
            return True
            
        except Exception as e:
            print(f"❌ خطأ: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    async def read_actpos(self):
        """قراءة ActPos"""
        if not self._connected:
            return None
        try:
            val = await self.act_pos_node.read_value()
            if isinstance(val, (list, tuple)):
                return list(val)
            return [float(val)] if val is not None else [0.0, 0.0, 0.0, 0.0]
        except:
            return None
    
    async def monitor_go2pos(self):
        """مراقبة تغييرات Go2Pos"""
        if not self._connected:
            return
        
        last_go2pos = None
        print("\n🔍 مراقبة تغييرات Go2Pos...")
        
        while True:
            try:
                current = await self.go2pos_node.read_value()
                if isinstance(current, (list, tuple)):
                    current = list(current)
                else:
                    current = [float(current)] if current is not None else [0.0, 0.0, 0.0, 0.0]
                
                if last_go2pos is None:
                    last_go2pos = current
                    print(f"   📌 القيمة الأولية: {current}")
                elif current != last_go2pos:
                    print(f"\n🔄 تغيير في Go2Pos: {last_go2pos} → {current}")
                    await self.execute_motion(current)
                    last_go2pos = current
                
                await asyncio.sleep(0.2)
            except Exception as e:
                print(f"⚠️ خطأ في المراقبة: {e}")
                await asyncio.sleep(1)
    
    async def disconnect(self):
        """قطع الاتصال"""
        if self.client:
            try:
                await self.client.disconnect()
            except:
                pass
        self._connected = False

# ============================================================
# 4. CoppeliaSim
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
            
            print("✅ [CoppeliaSim] جاهز مع IK")
            return sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d
        
        print("❌ [CoppeliaSim] فشل إعداد IK")
        return None, None, None, None, None, None, None, None
        
    except Exception as e:
        print(f"❌ [CoppeliaSim] خطأ: {e}")
        return None, None, None, None, None, None, None, None

def run_coppelia_engine():
    result = setup_coppelia()
    
    if result[0] is None:
        print("❌ فشل بدء CoppeliaSim")
        return
    
    sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d = result
    
    print("🚀 [CoppeliaSim] يعمل...")
    
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
        print("\n⏹️ [CoppeliaSim] إيقاف...")
    
    sim.stopSimulation()
    print("👋 [CoppeliaSim] متوقف")

# ============================================================
# 5. الوظيفة الرئيسية
# ============================================================

async def main():
    print("=" * 70)
    print("🎯 PLC CONTROL - اتصال واحد فقط")
    print("=" * 70)
    
    # إنشاء عميل PLC
    plc = PLCClient()
    
    # الاتصال بـ PLC
    print("🔌 الاتصال بـ PLC...")
    if not await plc.connect():
        print("❌ فشل الاتصال بـ PLC")
        return
    
    # تمكين النظام
    await plc.enable_system()
    
    # الاشتراك في Live_XYZ_Angle
    await plc.subscribe_live_position()
    
    # تشغيل CoppeliaSim
    coppelia_thread = threading.Thread(target=run_coppelia_engine, daemon=True)
    coppelia_thread.start()
    print("✅ [CoppeliaSim] تم بدء الخيط")
    
    await asyncio.sleep(2)
    
    print("\n" + "-" * 60)
    print("🎮 الأوامر:")
    print("  <رقم>  : تنفيذ نقطة محددة (1-5)")
    print("  'm'    : مراقبة تغييرات Go2Pos")
    print("  'status': عرض الموضع الحالي")
    print("  'exit'  : خروج")
    print("-" * 60)
    
    try:
        while True:
            act_pos = await plc.read_actpos()
            if act_pos:
                print(f"\n📍 ActPos: {act_pos}")
            
            user_input = input("\nأمر: ").strip()
            
            if user_input.lower() == 'exit':
                break
            
            if user_input.lower() == 'status':
                continue
            
            if user_input.lower() == 'm':
                print("🔍 بدء المراقبة... (اضغط Ctrl+C للعودة)")
                try:
                    await plc.monitor_go2pos()
                except KeyboardInterrupt:
                    print("\n⏹️ تم إيقاف المراقبة")
                continue
            
            if user_input not in POINTS:
                print("⚠️ اختيار غير صحيح. اختر 1-5 أو 'm'")
                continue
            
            point = POINTS[user_input]
            target = point["angles"]
            
            await plc.execute_motion(target)
                
    except KeyboardInterrupt:
        print("\n⏹️ تم الإيقاف بواسطة المستخدم")
    except Exception as e:
        print(f"\n❌ خطأ: {e}")
        import traceback
        traceback.print_exc()
    
    await plc.disconnect()
    print("\n👋 وداعاً!")

# ============================================================
# 6. التشغيل
# ============================================================

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n⏹️ تم الإيقاف بواسطة المستخدم")
    except Exception as e:
        print(f"\n❌ خطأ: {e}")
        import traceback
        traceback.print_exc()