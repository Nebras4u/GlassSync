# reset_errors.py - إعادة تعيين أخطاء المحركات وأوامر الحركة
import asyncio
from asyncua import Client, ua
import time

PLC_IP = "192.168.0.1"
SERVER_URL = f"opc.tcp://{PLC_IP}:4840"

async def reset_errors():
    try:
        async with Client(SERVER_URL, timeout=5) as client:
            print("✅ تم الاتصال بالـ PLC")
            
            # ==========================================
            # 1. إيقاف كل شيء أولاً
            # ==========================================
            print("\n🔄 إيقاف النظام...")
            
            enable_kin_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
            enable_all_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableAll"')
            
            # إيقاف Kinematics
            dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            print("   EnableKinematics = False")
            
            # إيقاف المحركات
            await enable_all_node.write_value(dv)
            print("   EnableAll = False")
            
            await asyncio.sleep(2)
            
            # ==========================================
            # 2. إعادة تعيين أخطاء المحاور
            # ==========================================
            print("\n🔄 إعادة تعيين أخطاء المحاور...")
            
            # محاولة إعادة تعيين كل محور
            for i in range(1, 5):
                try:
                    # محاولة الوصول إلى المحور
                    axis_reset = client.get_node(f'ns=3;s="PositioningAxis_{i}"."Reset"')
                    dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
                    dv.SourceTimestamp = None
                    dv.ServerTimestamp = None
                    await axis_reset.write_value(dv)
                    print(f"   Axis {i} Reset = True")
                    await asyncio.sleep(0.1)
                    
                    dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
                    dv.SourceTimestamp = None
                    dv.ServerTimestamp = None
                    await axis_reset.write_value(dv)
                    print(f"   Axis {i} Reset = False")
                except:
                    print(f"   ⚠️ Cannot reset Axis {i} (maybe no OPC UA access)")
            
            await asyncio.sleep(1)
            
            # ==========================================
            # 3. إعادة تعيين أوامر MC_MVLNERABS
            # ==========================================
            print("\n🔄 إعادة تعيين أوامر الحركة...")
            
            for i in range(1, 5):
                try:
                    # محاولة إعادة تعيين كل أمر حركة
                    mc_reset = client.get_node(f'ns=3;s="MC_MVLNERABS_Robot{i}_DB"."Reset"')
                    dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
                    dv.SourceTimestamp = None
                    dv.ServerTimestamp = None
                    await mc_reset.write_value(dv)
                    print(f"   Robot {i} Reset = True")
                    await asyncio.sleep(0.1)
                    
                    dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
                    dv.SourceTimestamp = None
                    dv.ServerTimestamp = None
                    await mc_reset.write_value(dv)
                    print(f"   Robot {i} Reset = False")
                except:
                    print(f"   ⚠️ Cannot reset Robot {i}")
            
            await asyncio.sleep(1)
            
            # ==========================================
            # 4. تشغيل النظام مرة أخرى
            # ==========================================
            print("\n🔄 تشغيل النظام...")
            
            # تشغيل المحركات
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_all_node.write_value(dv)
            print("   EnableAll = True")
            
            await asyncio.sleep(1)
            
            # ==========================================
            # 5. كتابة نقاط المسار مرة أخرى
            # ==========================================
            print("\n📝 كتابة نقاط المسار...")
            
            pos1_node = client.get_node('ns=3;s="TO_Data_block_1"."Pos1"')
            hmpos_node = client.get_node('ns=3;s="TO_Data_block_1"."HmPos"')
            pos3_node = client.get_node('ns=3;s="TO_Data_block_1"."Pos3"')
            pos4_node = client.get_node('ns=3;s="TO_Data_block_1"."Pos4"')
            vlcty_node = client.get_node('ns=3;s="TO_Data_block_1"."Vlcty"')
            
            # نقاط مسار مختلفة وجديدة
            pos1_values = [150.0, 80.0, 250.0, 10.0]
            hmpos_values = [250.0, 130.0, 350.0, 55.0]
            pos3_values = [350.0, 180.0, 450.0, 100.0]
            pos4_values = [450.0, 230.0, 550.0, 145.0]
            
            dv = ua.DataValue(ua.Variant(pos1_values, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await pos1_node.write_value(dv)
            print(f"   Pos1 = {pos1_values}")
            
            dv = ua.DataValue(ua.Variant(hmpos_values, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await hmpos_node.write_value(dv)
            print(f"   HmPos = {hmpos_values}")
            
            dv = ua.DataValue(ua.Variant(pos3_values, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await pos3_node.write_value(dv)
            print(f"   Pos3 = {pos3_values}")
            
            dv = ua.DataValue(ua.Variant(pos4_values, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await pos4_node.write_value(dv)
            print(f"   Pos4 = {pos4_values}")
            
            # السرعة
            dv = ua.DataValue(ua.Variant(50.0, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await vlcty_node.write_value(dv)
            print("   Vlcty = 50.0")
            
            await asyncio.sleep(1)
            
            # ==========================================
            # 6. تشغيل الحركة (Rising Edge)
            # ==========================================
            print("\n🚀 تشغيل الحركة...")
            
            # تأكد من False
            dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            print("   EnableKinematics = False")
            await asyncio.sleep(1)
            
            # ثم True
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            print("   EnableKinematics = True ✅")
            
            print("\n✅ تم إرسال جميع الأوامر!")
            print("انتظر 3 ثواني وتحقق من الحركة...")
            
            await asyncio.sleep(3)
            
            # ==========================================
            # 7. التحقق من الأخطاء مرة أخرى
            # ==========================================
            print("\n📊 التحقق من الحالة...")
            
            robot1_error = client.get_node('ns=3;s="MC_MVLNERABS_Robot1_DB"."Error"')
            robot1_busy = client.get_node('ns=3;s="MC_MVLNERABS_Robot1_DB"."Busy"')
            
            error_val = await robot1_error.read_value()
            busy_val = await robot1_busy.read_value()
            
            print(f"Robot1 Error: {error_val}")
            print(f"Robot1 Busy: {busy_val}")
            
            if error_val:
                print("\n⚠️ لا يزال هناك خطأ!")
                print("تحتاج إلى إعادة تعيين الأخطاء يدوياً في PLC")
                print("أو التحقق من تكوين Kinematics والمحاور")
            else:
                print("\n✅ يبدو أن الأخطاء تم إصلاحها!")
                
    except Exception as e:
        print(f"❌ خطأ: {e}")

if __name__ == "__main__":
    asyncio.run(reset_errors())