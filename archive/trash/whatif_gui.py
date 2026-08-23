#!/usr/bin/env python3
"""
whatif_gui.py - What-If Mode (Circular Path Simulation)
Using the same successful connection method as run_plc.py
"""

import sys
import os
import asyncio
import math
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ============================================================
# CoppeliaSim Connection (SAME as run_plc.py - WORKING)
# ============================================================

def setup_coppelia():
    """EXACT copy from run_plc.py - WORKING"""
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
            
            print("✅ [CoppeliaSim] Ready with IK")
            return sim_client, sim, simIK, simBase, simTarget, ikEnv, ikGroup_u, ikGroup_d
        
        print("❌ [CoppeliaSim] IK setup failed")
        return None, None, None, None, None, None, None, None
    except Exception as e:
        print(f"❌ CoppeliaSim error: {e}")
        return None, None, None, None, None, None, None, None


# ============================================================
# What-If GUI
# ============================================================

class WhatIfGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("🤖 What-If Mode - Circular Path Simulation")
        self.root.geometry("650x700")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # CoppeliaSim
        self.sim_client = None
        self.sim = None
        self.simIK = None
        self.simBase = None
        self.simTarget = None
        self.ikEnv = None
        self.ikGroup_u = None
        self.ikGroup_d = None
        self.connected = False
        self.sim_running = False
        self.sim_thread = None

        # Path parameters
        self.radius = 0.3
        self.height = 0.5
        self.speed = 0.5
        self.direction = 1
        self._time = 0.0
        self._path_running = False

        # Status
        self.status_var = tk.StringVar(value="🟡 Initializing...")
        self.position_var = tk.StringVar(value="Position: --")
        self.coppelia_status_var = tk.StringVar(value="🔌 Disconnected")

        self.create_widgets()
        self.connect_coppelia()

    def create_widgets(self):
        title = tk.Label(self.root, text="What-If Mode - Circular Path", font=("Arial", 18, "bold"))
        title.pack(pady=10)

        # Status
        status_frame = ttk.LabelFrame(self.root, text="Status", padding=10)
        status_frame.pack(fill=tk.X, padx=20, pady=5)

        for label, var in [
            ("CoppeliaSim:", self.coppelia_status_var),
            ("System:", self.status_var),
            ("Position:", self.position_var),
        ]:
            row = tk.Frame(status_frame)
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=label, width=15, anchor=tk.W).pack(side=tk.LEFT)
            tk.Label(row, textvariable=var, anchor=tk.W).pack(side=tk.LEFT)

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=20, pady=10)

        # Path Parameters
        params_frame = ttk.LabelFrame(self.root, text="Circular Path Parameters", padding=10)
        params_frame.pack(fill=tk.X, padx=20, pady=5)

        # Radius
        radius_frame = ttk.Frame(params_frame)
        radius_frame.pack(fill=tk.X, pady=2)
        tk.Label(radius_frame, text="Radius (m):", width=15).pack(side=tk.LEFT)
        self.radius_var = tk.DoubleVar(value=0.3)
        ttk.Scale(radius_frame, from_=0.05, to=0.8, variable=self.radius_var, orient=tk.HORIZONTAL, length=200).pack(side=tk.LEFT, padx=5)
        tk.Label(radius_frame, textvariable=self.radius_var, width=6).pack(side=tk.LEFT)

        # Height
        height_frame = ttk.Frame(params_frame)
        height_frame.pack(fill=tk.X, pady=2)
        tk.Label(height_frame, text="Height (m):", width=15).pack(side=tk.LEFT)
        self.height_var = tk.DoubleVar(value=0.5)
        ttk.Scale(height_frame, from_=0.1, to=1.0, variable=self.height_var, orient=tk.HORIZONTAL, length=200).pack(side=tk.LEFT, padx=5)
        tk.Label(height_frame, textvariable=self.height_var, width=6).pack(side=tk.LEFT)

        # Speed
        speed_frame = ttk.Frame(params_frame)
        speed_frame.pack(fill=tk.X, pady=2)
        tk.Label(speed_frame, text="Speed:", width=15).pack(side=tk.LEFT)
        self.speed_var = tk.DoubleVar(value=0.5)
        ttk.Scale(speed_frame, from_=0.1, to=2.0, variable=self.speed_var, orient=tk.HORIZONTAL, length=200).pack(side=tk.LEFT, padx=5)
        tk.Label(speed_frame, textvariable=self.speed_var, width=6).pack(side=tk.LEFT)

        # Direction
        dir_frame = ttk.Frame(params_frame)
        dir_frame.pack(fill=tk.X, pady=2)
        tk.Label(dir_frame, text="Direction:", width=15).pack(side=tk.LEFT)
        self.direction_var = tk.StringVar(value="CW")
        ttk.Combobox(dir_frame, textvariable=self.direction_var, values=["CW", "CCW"], width=8, state="readonly").pack(side=tk.LEFT, padx=5)

        # Apply button
        ttk.Button(params_frame, text="Apply Parameters", command=self.apply_parameters).pack(pady=5)

        ttk.Separator(self.root, orient='horizontal').pack(fill='x', padx=20, pady=10)

        # Controls
        controls_frame = ttk.LabelFrame(self.root, text="Simulation Controls", padding=10)
        controls_frame.pack(fill=tk.X, padx=20, pady=5)

        controls_grid = ttk.Frame(controls_frame)
        controls_grid.pack()

        ttk.Button(controls_grid, text="▶ Start", command=self.start_simulation).pack(side=tk.LEFT, padx=5)
        ttk.Button(controls_grid, text="⏸ Pause", command=self.pause_simulation).pack(side=tk.LEFT, padx=5)
        ttk.Button(controls_grid, text="⏹ Stop", command=self.stop_simulation).pack(side=tk.LEFT, padx=5)
        ttk.Button(controls_grid, text="🏠 Home", command=self.go_home).pack(side=tk.LEFT, padx=5)
        ttk.Button(controls_grid, text="📊 Status", command=self.show_status).pack(side=tk.LEFT, padx=5)

        # Log
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

        self.log_text = tk.Text(log_frame, height=10, font=("Courier", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(self.log_text)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.log_text.yview)

    def connect_coppelia(self):
        """Connect using the SAME method as run_plc.py (WORKING)"""
        self.log("🔄 Connecting to CoppeliaSim...")
        self.status_var.set("🟡 Connecting...")

        try:
            result = setup_coppelia()
            if result[0] is not None:
                self.sim_client, self.sim, self.simIK, self.simBase, self.simTarget, self.ikEnv, self.ikGroup_u, self.ikGroup_d = result
                self.connected = True
                self.coppelia_status_var.set("🟢 Connected")
                self.status_var.set("🟢 Ready")
                self.log("✅ CoppeliaSim connected!")
            else:
                self.log("❌ Connection failed - Is CoppeliaSim running?")
                self.coppelia_status_var.set("🔴 Error")
                self.status_var.set("🔴 Error")
        except Exception as e:
            self.log(f"❌ Error: {e}")
            self.coppelia_status_var.set("🔴 Error")

    def apply_parameters(self):
        self.radius = self.radius_var.get()
        self.height = self.height_var.get()
        self.speed = self.speed_var.get()
        self.direction = 1 if self.direction_var.get() == "CW" else -1
        self.log(f"⚙️ Parameters: R={self.radius}, H={self.height}, V={self.speed}")

    def get_next_position(self):
        """Get next position on circular path"""
        if not self._path_running:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        dt = 0.02 * self.speed
        self._time += dt * self.speed
        angle = self._time * self.direction

        x = self.radius * math.cos(angle)
        y = self.radius * math.sin(angle)
        z = self.height

        # Convert Cartesian to joint angles (simplified)
        theta1 = math.atan2(y, x) if (x != 0 or y != 0) else 0.0
        r = math.sqrt(x**2 + y**2)
        theta2 = math.atan2(z - 0.1, r + 0.1) - 0.3
        theta3 = math.atan2(z - 0.2, r - 0.1) - 0.2
        theta4 = theta1 * 0.3

        # Convert to degrees
        return [
            theta1 * 180.0 / math.pi,
            theta2 * 180.0 / math.pi,
            theta3 * 180.0 / math.pi,
            theta4 * 180.0 / math.pi,
            0.0,
            0.0
        ]

    def move_to_angles(self, angles_deg):
        """Move robot using the SAME method as run_plc.py"""
        if not self.connected:
            return

        angles_rad = [a * math.pi / 180.0 for a in angles_deg]

        try:
            # Use IK to move to position
            # Convert angles to position (simplified)
            theta1, theta2, theta3, theta4 = angles_rad[:4]
            
            x = 0.3 * math.cos(theta1) + 0.2 * math.cos(theta1 + theta2)
            y = 0.3 * math.sin(theta1) + 0.2 * math.sin(theta1 + theta2)
            z = 0.2 + 0.2 * math.sin(theta2) + 0.1 * math.sin(theta2 + theta3)
            
            # Move target
            self.sim.setObjectPosition(self.simTarget, self.simBase, [x, y, z])
            
            # Solve IK
            result = self.simIK.handleIkGroup(self.ikEnv, self.ikGroup_u)
            if result != self.simIK.result_success:
                self.simIK.handleIkGroup(self.ikEnv, self.ikGroup_d)
            
            self.sim_client.step()
            
        except Exception as e:
            print(f"❌ Move error: {e}")

    def start_simulation(self):
        if not self.connected:
            self.log("❌ Not connected to CoppeliaSim")
            return

        self._path_running = True
        self._time = 0.0
        self.sim_running = True
        self.status_var.set("🟢 Running")
        self.log("▶️ Simulation started")

        if self.sim_thread is None or not self.sim_thread.is_alive():
            self.sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
            self.sim_thread.start()

    def _sim_loop(self):
        """Simulation loop"""
        while self.sim_running:
            if self._path_running and self.connected:
                angles = self.get_next_position()
                self.move_to_angles(angles)
                self.root.after(0, lambda: self.position_var.set(f"Position: {[round(a, 1) for a in angles[:4]]}"))
            time.sleep(0.02)

    def pause_simulation(self):
        if self._path_running:
            self._path_running = False
            self.status_var.set("⏸ Paused")
            self.log("⏸ Paused")
        else:
            self._path_running = True
            self.status_var.set("🟢 Running")
            self.log("▶️ Resumed")

    def stop_simulation(self):
        self.sim_running = False
        self._path_running = False
        self.status_var.set("⏹ Stopped")
        self.log("⏹ Stopped")

    def go_home(self):
        if self.connected:
            self.move_to_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.position_var.set("Position: Home")
            self.log("🏠 Home")

    def show_status(self):
        self.log(f"📊 Radius={self.radius}, Height={self.height}, Speed={self.speed}")

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)

    def on_close(self):
        if messagebox.askokcancel("Close", "Close What-If mode?"):
            self.sim_running = False
            if self.sim_client:
                try:
                    self.sim.stopSimulation()
                except:
                    pass
            self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    gui = WhatIfGUI()
    gui.run()


if __name__ == "__main__":
    main()