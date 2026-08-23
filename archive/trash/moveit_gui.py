#!/usr/bin/env python3
"""
moveit_gui.py - MOVEIT Mode (Manual Position Control)
Controls robot by writing to Go2Pos with predefined waypoints
"""

import sys
import os
import asyncio
import math
import time
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from asyncua import Client, ua
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ============================================================
# Configuration
# ============================================================

PLC_IP = "192.168.0.1"
SERVER_URL = f"opc.tcp://{PLC_IP}:4840"

# Queue for CoppeliaSim updates
points_update_queue = queue.Queue()

# ============================================================
# ACTUAL WAYPOINTS FROM YOUR PROJECT
# ============================================================
# Values are in mm and degrees (from your DB)

WAYPOINTS = {
    "pos1": {
        "name": "Pos1 (SetPos)",
        "description": "First waypoint - SetPos",
        "angles": [930.0, 0.0, 370.0, 0.0]
    },
    "hmpos": {
        "name": "HmPos (Home)",
        "description": "Second waypoint - Home Position",
        "angles": [777.0, 0.0, 130.0, 0.0]
    },
    "pos3": {
        "name": "Pos3 (Third)",
        "description": "Third waypoint",
        "angles": [925.0, 192.0, 99.0, 0.0]
    },
    "pos4": {
        "name": "Pos4 (Fourth)",
        "description": "Fourth waypoint - Stop",
        "angles": [550.0, 500.0, 550.0, 30.0]
    },
    "go2pos": {
        "name": "Go2Pos (Manual)",
        "description": "Manual target position",
        "angles": [550.0, 440.0, 330.0, 22.0]
    }
}

CUSTOM_POINTS = {
    "custom1": {"name": "Custom 1", "angles": [300.0, 200.0, 400.0, 30.0]},
    "custom2": {"name": "Custom 2", "angles": [100.0, 50.0, 200.0, 10.0]},
    "custom3": {"name": "Custom 3", "angles": [400.0, 250.0, 450.0, 40.0]},
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
# OPC UA Client (Stable)
# ============================================================

opcua_running = True

async def run_opcua_client_stable():
    global opcua_running
    print(f"🔌 [OPC UA] Connecting to PLC at {SERVER_URL}...")
    
    while opcua_running:
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
                
                handler = OPCUASubscriptionHandler(live_tcp_node)
                sub = await client.create_subscription(500, handler)
                await sub.subscribe_data_change(live_tcp_node)
                
                print("✅ [OPC UA] Subscription active")
                
                while opcua_running:
                    try:
                        await client.get_endpoints()
                        await asyncio.sleep(1)
                    except Exception as e:
                        print(f"⚠️ Connection lost: {e}")
                        break
                    
        except Exception as e:
            if opcua_running:
                print(f"\n⚠️ [OPC UA] Error: {e}")
                print("🔄 Retrying in 3 seconds...")
                await asyncio.sleep(3)


def start_opcua_thread():
    asyncio.run(run_opcua_client_stable())


# ============================================================
# Write to PLC Functions
# ============================================================

async def write_go2pos_and_execute(angles):
    try:
        async with Client(SERVER_URL, timeout=5) as client:
            go2pos_node = client.get_node('ns=3;s="TO_Data_block_1"."Go2Pos"')
            enable_kin_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
            vlcty_node = client.get_node('ns=3;s="TO_Data_block_1"."Vlcty"')
            
            variant = ua.Variant(angles, ua.VariantType.Double)
            dv = ua.DataValue(variant)
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await go2pos_node.write_value(dv)
            
            dv = ua.DataValue(ua.Variant(50.0, ua.VariantType.Double))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await vlcty_node.write_value(dv)
            
            dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            await asyncio.sleep(0.2)
            
            dv = ua.DataValue(ua.Variant(True, ua.VariantType.Boolean))
            dv.SourceTimestamp = None
            dv.ServerTimestamp = None
            await enable_kin_node.write_value(dv)
            
            return True, "Success"
    except Exception as e:
        return False, str(e)


def write_go2pos_sync(angles):
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        success, msg = loop.run_until_complete(write_go2pos_and_execute(angles))
        loop.close()
        return success, msg
    except Exception as e:
        return False, str(e)


# ============================================================
# CoppeliaSim (from run_plc.py)
# ============================================================

def setup_coppelia():
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
# MOVEIT GUI Class
# ============================================================

class MoveItGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("🤖 MOVEIT Mode - Manual Position Control")
        self.root.geometry("800x700")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.status_var = tk.StringVar(value="🟡 Initializing...")
        self.position_var = tk.StringVar(value="Position: --")
        self.plc_status_var = tk.StringVar(value="🔌 Disconnected")
        self.coppelia_status_var = tk.StringVar(value="🔌 Disconnected")

        self.create_widgets()
        self.start_background_threads()
        self.update_status()

    def create_widgets(self):
        title = tk.Label(self.root, text="MOVEIT Mode - Manual Position Control", font=("Arial", 18, "bold"))
        title.pack(pady=10)

        # Status Frame
        status_frame = ttk.LabelFrame(self.root, text="System Status", padding=10)
        status_frame.pack(fill=tk.X, padx=20, pady=5)

        for label, var in [
            ("PLC:", self.plc_status_var),
            ("CoppeliaSim:", self.coppelia_status_var),
            ("System:", self.status_var),
            ("Position:", self.position_var),
        ]:
            row = tk.Frame(status_frame)
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=label, width=15, anchor=tk.W).pack(side=tk.LEFT)
            tk.Label(row, textvariable=var, anchor=tk.W).pack(side=tk.LEFT)

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=20, pady=10)

        # Main Waypoints (from SCL)
        main_frame = ttk.LabelFrame(self.root, text="Main Waypoints (Pos1, HmPos, Pos3, Pos4)", padding=10)
        main_frame.pack(fill=tk.X, padx=20, pady=5)

        main_grid = ttk.Frame(main_frame)
        main_grid.pack()

        main_waypoints = [
            ("📍 Pos1 (SetPos)", "pos1", "#4CAF50"),
            ("🏠 HmPos (Home)", "hmpos", "#2196F3"),
            ("📍 Pos3 (Third)", "pos3", "#FF9800"),
            ("⏹ Pos4 (Stop)", "pos4", "#f44336"),
        ]

        for i, (label, key, color) in enumerate(main_waypoints):
            btn = tk.Button(
                main_grid,
                text=label,
                command=lambda k=key: self.move_to_point(k),
                width=18,
                height=2,
                bg=color,
                fg="white",
                font=("Arial", 10, "bold")
            )
            btn.grid(row=i//2, column=i%2, padx=5, pady=5)

        # Go2Pos
        go2_frame = ttk.LabelFrame(self.root, text="Go2Pos (Manual Target)", padding=10)
        go2_frame.pack(fill=tk.X, padx=20, pady=5)

        go2_grid = ttk.Frame(go2_frame)
        go2_grid.pack()

        tk.Button(
            go2_grid,
            text="🎯 Go2Pos (Manual)",
            command=lambda: self.move_to_point("go2pos"),
            width=20,
            height=2,
            bg="#9C27B0",
            fg="white",
            font=("Arial", 10, "bold")
        ).pack(pady=5)

        # Custom Points
        custom_frame = ttk.LabelFrame(self.root, text="Custom Points", padding=10)
        custom_frame.pack(fill=tk.X, padx=20, pady=5)

        custom_grid = ttk.Frame(custom_frame)
        custom_grid.pack()

        custom_waypoints = [
            ("📐 Custom 1", "custom1", "#607D8B"),
            ("📐 Custom 2", "custom2", "#795548"),
            ("📐 Custom 3", "custom3", "#009688"),
        ]

        for i, (label, key, color) in enumerate(custom_waypoints):
            btn = tk.Button(
                custom_grid,
                text=label,
                command=lambda k=key: self.move_to_point(k),
                width=15,
                height=2,
                bg=color,
                fg="white",
                font=("Arial", 10)
            )
            btn.grid(row=i//3, column=i%3, padx=5, pady=5)

        # Controls
        controls_frame = ttk.LabelFrame(self.root, text="Controls", padding=10)
        controls_frame.pack(fill=tk.X, padx=20, pady=5)

        controls_grid = ttk.Frame(controls_frame)
        controls_grid.pack()

        for text, cmd in [
            ("📊 Read Status", self.read_status),
            ("🔄 Reset System", self.reset_system),
            ("🚨 Emergency Stop", self.emergency_stop),
        ]:
            ttk.Button(controls_grid, text=text, command=cmd).pack(side=tk.LEFT, padx=5)

        # Info
        info_frame = ttk.LabelFrame(self.root, text="Waypoint Information", padding=10)
        info_frame.pack(fill=tk.X, padx=20, pady=5)

        self.info_text = tk.Text(info_frame, height=6, font=("Courier", 9), state=tk.DISABLED)
        self.info_text.pack(fill=tk.BOTH, expand=True)

        # Log
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

        self.log_text = tk.Text(log_frame, height=10, font=("Courier", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(self.log_text)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.log_text.yview)

    def start_background_threads(self):
        self.log("🔄 Starting background threads...")
        
        try:
            self.opc_thread = threading.Thread(target=start_opcua_thread, daemon=True)
            self.opc_thread.start()
            self.log("✅ OPC UA thread started")
            self.plc_status_var.set("🟢 Connected")
        except Exception as e:
            self.log(f"❌ OPC UA error: {e}")
            self.plc_status_var.set("🔴 Error")

        try:
            self.coppelia_thread = threading.Thread(target=run_coppelia_engine, daemon=True)
            self.coppelia_thread.start()
            self.log("✅ CoppeliaSim thread started")
            self.coppelia_status_var.set("🟢 Connected")
        except Exception as e:
            self.log(f"❌ CoppeliaSim error: {e}")
            self.coppelia_status_var.set("🔴 Error")

        self.status_var.set("🟢 System Ready")
        self.log("✅ System started")
        self.show_waypoint_info()

    def show_waypoint_info(self):
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete(1.0, tk.END)
        
        info = "📋 Waypoint Values (from PLC DB):\n"
        info += "-" * 50 + "\n"
        
        all_points = {**WAYPOINTS, **CUSTOM_POINTS}
        for key, point in all_points.items():
            info += f"  {point['name']:15} : {point['angles']}\n"
            if 'description' in point:
                info += f"    {point['description']}\n"
        
        self.info_text.insert(1.0, info)
        self.info_text.config(state=tk.DISABLED)

    def update_status(self):
        if not hasattr(self, 'root'):
            return

        try:
            if not points_update_queue.empty():
                latest_point = None
                while not points_update_queue.empty():
                    latest_point = points_update_queue.get_nowait()
                if latest_point:
                    pos = latest_point["pos"]
                    angle = latest_point["angle"]
                    self.position_var.set(f"Position: X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f}, A={math.degrees(angle):.1f}°")
        except:
            pass

        self.root.after(1000, self.update_status)

    def move_to_point(self, key):
        all_points = {**WAYPOINTS, **CUSTOM_POINTS}
        if key not in all_points:
            return

        point = all_points[key]
        angles = point["angles"]
        name = point["name"]

        self.log(f"📍 Moving to {name}: {angles}°")
        self.status_var.set(f"🟡 Moving to {name}...")

        thread = threading.Thread(target=self._move_thread, args=(angles, name), daemon=True)
        thread.start()

    def _move_thread(self, angles, name):
        success, msg = write_go2pos_sync(angles)
        if success:
            self.root.after(0, lambda: self.log(f"✅ Moved to {name}"))
            self.root.after(0, lambda: self.status_var.set("🟢 System Ready"))
        else:
            self.root.after(0, lambda: self.log(f"❌ Move error: {msg}"))
            self.root.after(0, lambda: self.status_var.set("🔴 Error"))

    def read_status(self):
        self.log("📊 Reading status...")
        self.status_var.set("🟡 Reading...")
        
        thread = threading.Thread(target=self._read_thread, daemon=True)
        thread.start()

    def _read_thread(self):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._async_read())
        except Exception as e:
            self.root.after(0, lambda: self.log(f"❌ Read error: {e}"))

    async def _async_read(self):
        try:
            async with Client(SERVER_URL, timeout=5) as client:
                act_pos_node = client.get_node('ns=3;s="TO_Data_block_1"."ActPos"')
                val = await act_pos_node.read_value()
                if isinstance(val, (list, tuple)):
                    self.root.after(0, lambda: self.log(f"📊 Position: {list(val)}"))
                    self.root.after(0, lambda: self.position_var.set(f"Position: {list(val)}°"))
                self.root.after(0, lambda: self.status_var.set("🟢 System Ready"))
        except Exception as e:
            self.root.after(0, lambda: self.log(f"❌ Read error: {e}"))
            self.root.after(0, lambda: self.status_var.set("🔴 Error"))

    def reset_system(self):
        self.log("🔄 Resetting system...")
        self.status_var.set("🟡 Resetting...")
        thread = threading.Thread(target=self._move_thread, args=(WAYPOINTS["hmpos"]["angles"], "Home"), daemon=True)
        thread.start()

    def emergency_stop(self):
        if messagebox.askyesno("Emergency Stop", "Are you sure?"):
            self.log("🚨 EMERGENCY STOP!")
            self.status_var.set("🔴 EMERGENCY STOP")
            
            thread = threading.Thread(target=self._emergency_thread, daemon=True)
            thread.start()

    def _emergency_thread(self):
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._async_emergency())
        except Exception as e:
            self.root.after(0, lambda: self.log(f"❌ Emergency error: {e}"))

    async def _async_emergency(self):
        try:
            async with Client(SERVER_URL, timeout=5) as client:
                enable_all_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableAll"')
                enable_kin_node = client.get_node('ns=3;s="TO_Data_block_1"."EnableKinematics"')
                
                dv = ua.DataValue(ua.Variant(False, ua.VariantType.Boolean))
                dv.SourceTimestamp = None
                dv.ServerTimestamp = None
                await enable_all_node.write_value(dv)
                await enable_kin_node.write_value(dv)
                
                self.root.after(0, lambda: self.log("✅ Emergency stop activated"))
        except Exception as e:
            self.root.after(0, lambda: self.log(f"❌ Emergency error: {e}"))

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)

    def on_close(self):
        if messagebox.askokcancel("Close", "Do you want to close the application?"):
            global opcua_running
            opcua_running = False
            self.log("👋 Shutting down...")
            time.sleep(1)
            self.root.destroy()

    def run(self):
        self.root.mainloop()


# ============================================================
# Main
# ============================================================

def main():
    gui = MoveItGUI()
    gui.run()


if __name__ == "__main__":
    main()