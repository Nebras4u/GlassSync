#!/usr/bin/env python3
"""
ask_and_move.py - يسألك عن القيمة ثم ينفذها (مثل run_plc.py)
"""

import sys
import os
import asyncio
import time
import threading
from asyncua import Client, ua

# ============================================================
# إعدادات
# ============================================================

PLC_IP = "192.168.0.1"
SERVER_URL = f"opc.tcp://{PLC_IP}:4840"

# ============================================================
# دوال التحريك (نفس run_plc.py)
# ============================================================

async def move_to(angles):
    """نفس طريقة run_plc.py"""
    try:
        async with Client(SERVER_URL, timeout=5) as client:
            go2pos_node = client.get_node('ns=3;s="TO_Data_block_1"."Go2Pos"')
            enable_kin_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
            vlcty_node = client.get_node('ns=3;s="TO_Data_block_1"."Vlcty"')
            
            print(f"\n📍 التحرك إلى: {angles}")
            
            # 1. Go2Pos
            variant = ua.Variant(angles, ua.VariantType.Double)
            dv = ua.DataValue(variant)
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await go2pos_node.write_value(dv)
            print("   ✅ Go2Pos مكتوب")
            
            # 2. Vlcty
            dv = ua.DataValue(ua.Variant(50.0, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await vlcty_node.write_value(dv)
            print("   ✅ Vlcty = 50")
            
            # 3. إيقاف EnableKinematics
            dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            await asyncio.sleep(0.2)
            print("   ✅ EnableKinematics = FALSE")
            
            # 4. تشغيل EnableKinematics
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            print("   ✅ EnableKinematics = TRUE")
            
            print("   ✅ تم تنفيذ الحركة!")
            return True
            
    except Exception as e:
        print(f"   ❌ خطأ: {e}")
        return False

# ============================================================
# تشغيل OPC UA في الخلفية (مثل run_plc.py)
# ============================================================

def start_opcua_background():
    """تشغيل اتصال OPC UA في الخلفية"""
    async def run():
        while True:
            try:
                async with Client(SERVER_URL, timeout=5) as client:
                    print("🔌 [OPC UA] متصل في الخلفية")
                    await asyncio.sleep(1)
                    while True:
                        try:
                            await client.get_endpoints()
                            await asyncio.sleep(1)
                        except:
                            break
            except:
                await asyncio.sleep(5)
    
    asyncio.run(run())

# ============================================================
# الوظيفة الرئيسية - تسأل المستخدم
# ============================================================

async def main():
    print("=" * 70)
    print("🎯 اسأل وأتحرك - أدخل القيم التي تريدها".center(70))
    print("=" * 70)
    
    # تشغيل OPC UA في الخلفية
    thread = threading.Thread(target=start_opcua_background, daemon=True)
    thread.start()
    
    print("\n📝 أدخل زوايا المفاصل الأربعة (J1, J2, J3, J4)")
    print("   مثال: 777, 0, 130, 0")
    print("   أو: 930, 0, 370, 0")
    print("   أو اكتب 'exit' للخروج")
    print("-" * 50)
    
    loop = asyncio.get_event_loop()
    
    while True:
        user_input = await loop.run_in_executor(None, input, "\nأدخل القيم (J1,J2,J3,J4): ")
        
        if user_input.lower() == 'exit':
            break
        
        try:
            # تحويل الإدخال إلى أرقام
            parts = user_input.split(',')
            if len(parts) != 4:
                print("⚠️ يجب إدخال 4 قيم مفصولة بفواصل")
                continue
            
            angles = [float(p.strip()) for p in parts]
            
            # تنفيذ الحركة
            await move_to(angles)
            
        except ValueError:
            print("⚠️ قيم غير صحيحة. استخدم أرقاماً مثل: 777, 0, 130, 0")
        except Exception as e:
            print(f"⚠️ خطأ: {e}")
    
    print("\n👋 وداعاً!")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n⏹️ تم الإيقاف")