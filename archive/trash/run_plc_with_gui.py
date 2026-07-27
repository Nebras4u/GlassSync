# send_command.py - أرسل أوامر إلى PLC باستخدام run_plc.py

import asyncio
from asyncua import Client, ua

SERVER_URL = "opc.tcp://192.168.0.1:4840"

async def send_go2pos(angles):
    """إرسال أمر إلى Go2Pos (نفس طريقة run_plc.py)"""
    try:
        async with Client(SERVER_URL, timeout=5) as client:
            go2pos_node = client.get_node('ns=3;s="TO_Data_block_1"."Go2Pos"')
            enable_kin_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
            vlcty_node = client.get_node('ns=3;s="TO_Data_block_1"."Vlcty"')
            
            # كتابة الزوايا
            variant = ua.Variant(angles, ua.VariantType.Double)
            dv = ua.DataValue(variant)
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await go2pos_node.write_value(dv)
            
            # تعيين السرعة
            dv = ua.DataValue(ua.Variant(50.0, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await vlcty_node.write_value(dv)
            
            # إيقاف وتشغيل Kinematics
            dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            await asyncio.sleep(0.2)
            
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            
            return True
    except Exception as e:
        print(f"❌ خطأ: {e}")
        return False

# استخدام الأمر
if __name__ == "__main__":
    angles = [777.0, 0.0, 130.0, 0.0]  # Home position
    
    success = asyncio.run(send_go2pos(angles))
    if success:
        print("✅ تم إرسال الأمر")
    else:
        print("❌ فشل إرسال الأمر")