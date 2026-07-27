# run_and_monitor.py - تشغيل الحركة ومراقبتها
import asyncio
from asyncua import Client, ua
import time

PLC_IP = "192.168.0.1"
SERVER_URL = f"opc.tcp://{PLC_IP}:4840"

async def run_and_monitor():
    try:
        async with Client(SERVER_URL, timeout=5) as client:
            print("✅ تم الاتصال بالـ PLC")
            
            # ==========================================
            # 1. كتابة نقاط مسار جديدة
            # ==========================================
            print("\n📝 كتابة نقاط المسار...")
            
            pos1_node = client.get_node('ns=3;s="TO_Data_block_1"."Pos1"')
            hmpos_node = client.get_node('ns=3;s="TO_Data_block_1"."HmPos"')
            pos3_node = client.get_node('ns=3;s="TO_Data_block_1"."Pos3"')
            pos4_node = client.get_node('ns=3;s="TO_Data_block_1"."Pos4"')
            vlcty_node = client.get_node('ns=3;s="TO_Data_block_1"."Vlcty"')
            enable_all_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableAll"')
            enable_kin_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
            live_node = client.get_node('ns=3;s="TO_Data_block_1"."Live_XYZ_Angle"')
            
            # نقاط مسار جديدة
            pos1_values = [150.0, 80.0, 250.0, 10.0]
            hmpos_values = [250.0, 130.0, 350.0, 55.0]
            pos3_values = [350.0, 180.0, 450.0, 100.0]
            pos4_values = [450.0, 230.0, 550.0, 145.0]
            
            def write_array(node, values):
                dv = ua.DataValue(ua.Variant(values, ua.VariantType.Double))
                dv.SourceTimestamp = None
                dv.ServerTimestamp = None
                return node.write_value(dv)
            
            await write_array(pos1_node, pos1_values)
            print(f"   Pos1 = {pos1_values}")
            
            await write_array(hmpos_node, hmpos_values)
            print(f"   HmPos = {hmpos_values}")
            
            await write_array(pos3_node, pos3_values)
            print(f"   Pos3 = {pos3_values}")
            
            await write_array(pos4_node, pos4_values)
            print(f"   Pos4 = {pos4_values}")
            
            # السرعة
            dv = ua.DataValue(ua.Variant(50.0, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await vlcty_node.write_value(dv)
            print("   Vlcty = 50.0")
            
            # EnableAll
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_all_node.write_value(dv)
            print("   EnableAll = True")
            
            await asyncio.sleep(0.5)
            
            # ==========================================
            # 2. تشغيل الحركة (Rising Edge)
            # ==========================================
            print("\n🚀 تشغيل الحركة...")
            
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
            
            # ==========================================
            # 3. مراقبة الحركة
            # ==========================================
            print("\n📡 مراقبة المواضع الحية...")
            print("=" * 60)
            print("   X        Y        Z        A")
            print("-" * 60)
            
            robot1_busy = client.get_node('ns=3;s="MC_MVLNERABS_Robot1_DB"."Busy"')
            robot1_error = client.get_node('ns=3;s="MC_MVLNERABS_Robot1_DB"."Error"')
            
            for i in range(20):  # راقب لمدة 10 ثواني (20 * 0.5)
                try:
                    # قراءة الموضع الحي
                    live_pos = await live_node.read_value()
                    if live_pos and len(live_pos) >= 4:
                        print(f"[{i+1:2d}] {live_pos[0]:8.2f} {live_pos[1]:8.2f} {live_pos[2]:8.2f} {live_pos[3]:8.2f}")
                    
                    # قراءة حالة الحركة
                    if i % 4 == 0:  # كل ثانيتين
                        busy = await robot1_busy.read_value()
                        error = await robot1_error.read_value()
                        print(f"     Busy: {busy}, Error: {error}")
                    
                except Exception as e:
                    print(f"[{i+1}] ❌ {e}")
                
                await asyncio.sleep(0.5)
            
            # ==========================================
            # 4. الحالة النهائية
            # ==========================================
            print("\n" + "=" * 60)
            busy_final = await robot1_busy.read_value()
            error_final = await robot1_error.read_value()
            print(f"الحالة النهائية - Busy: {busy_final}, Error: {error_final}")
            
            if not busy_final and not error_final:
                print("✅ الحركة اكتملت بنجاح!")
            elif busy_final:
                print("⏳ الحركة لا تزال قيد التنفيذ...")
            
    except Exception as e:
        print(f"❌ خطأ: {e}")

if __name__ == "__main__":
    asyncio.run(run_and_monitor())