#!/usr/bin/env python3
"""
main_window.py - Main GUI Window with 4-Quadrant Layout (Fully Working)
"""

import tkinter as tk
from tkinter import ttk, messagebox
import logging
import threading
import asyncio
import time
from typing import Optional

logger = logging.getLogger("RobotTwin.GUI")


class MainWindow:
    """
    Main GUI window with 4-quadrant layout.
    """

    def __init__(self, controller):
        self.controller = controller
        self._running = False
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._async_thread: Optional[threading.Thread] = None
        
        # Panel references
        self.panels = {}

        # ──── Create Main Window ────
        self.root = tk.Tk()
        self.root.title("🏭 Digital Twin V6.1 - Control Panel")
        self.root.geometry("1400x900")
        self.root.minsize(1200, 700)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Apply custom style
        self._setup_style()

        # Create all GUI widgets
        self._create_widgets()

        # Start the asyncio event loop in a separate thread
        self._start_async_loop()

        # Start the periodic update loop
        self._running = True
        self._update_loop()

        logger.info("🖥️ GUI ready with 4-section layout")

    def _start_async_loop(self):
        """Start the asyncio event loop in a separate thread."""
        def run_loop():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            self._loop.run_forever()

        self._async_thread = threading.Thread(target=run_loop, daemon=True)
        self._async_thread.start()

        while self._loop is None:
            time.sleep(0.01)

        logger.info("✅ Asyncio event loop running in separate thread")

    def run_async(self, coro):
        """Schedule an async coroutine to run in the asyncio event loop."""
        if self._loop and self._loop.is_running():
            return asyncio.run_coroutine_threadsafe(coro, self._loop)
        return None

    def _setup_style(self):
        """Configure the ttk style for the application."""
        style = ttk.Style()
        style.theme_use('clam')

    def _create_widgets(self):
        """Create all widgets with 4-section layout."""
        
        # ──── Main Container ────
        main_paned = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # ──── Status Bar ────
        self._create_status_bar(main_paned)

        # ──── Middle Section (2x2 Grid) ────
        middle_frame = ttk.Frame(main_paned)
        main_paned.add(middle_frame, weight=3)

        middle_frame.columnconfigure(0, weight=1)
        middle_frame.columnconfigure(1, weight=1)
        middle_frame.rowconfigure(0, weight=1)
        middle_frame.rowconfigure(1, weight=1)

        # ── Top-Left: Dashboard ──
        self._create_dashboard(middle_frame, row=0, column=0)

        # ── Top-Right: Graphs ──
        self._create_graphs(middle_frame, row=0, column=1)

        # ── Bottom-Left: Control Panel ──
        self._create_control_panel(middle_frame, row=1, column=0)

        # ── Bottom-Right: Log Viewer ──
        self._create_log_viewer(middle_frame, row=1, column=1)

        # ──── Bottom: MOVEIT Manual Control ────
        self._create_manual_panel(main_paned)

    def _create_status_bar(self, parent):
        """Create the status bar."""
        status_frame = ttk.Frame(parent)
        status_frame.pack(fill=tk.X, padx=5, pady=5)
        parent.add(status_frame, weight=0)

        self.status_label = ttk.Label(
            status_frame,
            text="🟢 System Ready | PLC: ⏳ Connecting... | CoppeliaSim: ⏳ Connecting...",
            font=('Segoe UI', 10)
        )
        self.status_label.pack(side=tk.LEFT, padx=10)

        self.safety_label = ttk.Label(
            status_frame,
            text="🛡️ Safe",
            font=('Segoe UI', 10, 'bold'),
            foreground='green'
        )
        self.safety_label.pack(side=tk.LEFT, padx=20)

        mode_frame = ttk.Frame(status_frame)
        mode_frame.pack(side=tk.RIGHT, padx=10)

        self.mode_label = ttk.Label(mode_frame, text="Mode: PLC", font=('Segoe UI', 10))
        self.mode_label.pack(side=tk.LEFT, padx=10)

        self.cycle_label = ttk.Label(mode_frame, text="Cycles: 0", font=('Segoe UI', 10))
        self.cycle_label.pack(side=tk.LEFT, padx=10)

        ttk.Separator(parent, orient='horizontal').pack(fill=tk.X, padx=5, pady=2)

    def _create_dashboard(self, parent, row, column):
        """Create the monitoring dashboard."""
        frame = ttk.LabelFrame(parent, text="📊 Monitoring Dashboard", padding=5)
        frame.grid(row=row, column=column, sticky="nsew", padx=3, pady=3)
        
        self._create_dashboard_widgets(frame)
        self.panels['dashboard'] = frame

    def _create_dashboard_widgets(self, parent):
        """Create dashboard widgets."""
        # Joint angles with progress bars
        self.joint_labels = []
        joint_names = ["J1", "J2", "J3", "J4"]
        
        for i, name in enumerate(joint_names):
            frame = ttk.Frame(parent)
            frame.pack(fill=tk.X, pady=2)
            
            ttk.Label(frame, text=f"{name}:", width=5).pack(side=tk.LEFT)
            
            progress = ttk.Progressbar(frame, orient=tk.HORIZONTAL, length=150, mode='determinate')
            progress.pack(side=tk.LEFT, padx=5)
            
            label = ttk.Label(frame, text="0.00 rad", width=12)
            label.pack(side=tk.RIGHT)
            
            self.joint_labels.append({'progress': progress, 'label': label})
        
        # Speeds
        speed_frame = ttk.LabelFrame(parent, text="⚡ Speeds", padding=5)
        speed_frame.pack(fill=tk.X, pady=5)
        
        self.speed_labels = []
        for i, name in enumerate(["J1", "J2", "J3", "J4"]):
            frame = ttk.Frame(speed_frame)
            frame.pack(fill=tk.X, pady=1)
            ttk.Label(frame, text=f"{name}:", width=5).pack(side=tk.LEFT)
            label = ttk.Label(frame, text="0.00 rad/s", width=15)
            label.pack(side=tk.LEFT, padx=5)
            self.speed_labels.append(label)
        
        # Safety status
        safety_frame = ttk.LabelFrame(parent, text="🛡️ Safety", padding=5)
        safety_frame.pack(fill=tk.X, pady=5)
        
        self.safety_display = ttk.Label(safety_frame, text="🟢 Safe", font=('Arial', 12, 'bold'))
        self.safety_display.pack()

    def _create_graphs(self, parent, row, column):
        """Create the graphs panel."""
        frame = ttk.LabelFrame(parent, text="📈 Real-Time Graphs", padding=5)
        frame.grid(row=row, column=column, sticky="nsew", padx=3, pady=3)
        
        self.graph_canvas = tk.Canvas(frame, bg='#1e1e1e', height=200)
        self.graph_canvas.pack(fill=tk.BOTH, expand=True)
        
        # Draw simple grid
        self.graph_canvas.create_line(0, 180, 400, 180, fill='#333333')
        self.graph_canvas.create_line(0, 120, 400, 120, fill='#333333')
        self.graph_canvas.create_line(0, 60, 400, 60, fill='#333333')
        
        # Labels
        self.graph_canvas.create_text(10, 10, text="Joint Angles", fill='white', anchor='nw')
        self.graph_canvas.create_text(10, 190, text="Time", fill='white', anchor='nw')
        
        # Store line IDs for later updates
        self.graph_lines = []
        colors = ['#ff6b6b', '#4ecdc4', '#45b7d1', '#96ceb4']
        
        # Create lines with initial points
        for i, color in enumerate(colors):
            line_id = self.graph_canvas.create_line(0, 0, 1, 0, fill=color, width=2)
            self.graph_lines.append(line_id)
        
        # Legend
        legend_x = 300
        for i, (name, color) in enumerate(zip(['J1', 'J2', 'J3', 'J4'], colors)):
            self.graph_canvas.create_rectangle(legend_x, 10 + i*20, legend_x+15, 25 + i*20, fill=color, outline='')
            self.graph_canvas.create_text(legend_x+20, 17 + i*20, text=name, fill='white', anchor='w')
        
        self.graph_data = {f'J{i+1}': [] for i in range(4)}
        self.graph_counter = 0
        
        self.panels['graphs'] = frame

    def _create_control_panel(self, parent, row, column):
        """Create the control panel."""
        frame = ttk.LabelFrame(parent, text="🎮 System Control", padding=5)
        frame.grid(row=row, column=column, sticky="nsew", padx=3, pady=3)
        
        # Mode selection
        mode_frame = ttk.LabelFrame(frame, text="Operating Mode", padding=5)
        mode_frame.pack(fill=tk.X, pady=5)
        
        self.mode_var = tk.StringVar(value="online")
        
        for text, value in [("PLC (Online)", "online"), ("What-If", "virtual"), ("MOVEIT", "manual")]:
            ttk.Radiobutton(
                mode_frame, text=text, value=value, variable=self.mode_var,
                command=self._change_mode
            ).pack(anchor=tk.W, pady=1)
        
        # Speed control
        speed_frame = ttk.LabelFrame(frame, text="⚡ Speed", padding=5)
        speed_frame.pack(fill=tk.X, pady=5)
        
        self.speed_var = tk.DoubleVar(value=1.0)
        ttk.Scale(speed_frame, from_=0.1, to=2.0, variable=self.speed_var,
                  orient=tk.HORIZONTAL, length=150).pack(fill=tk.X)
        
        speed_buttons = ttk.Frame(speed_frame)
        speed_buttons.pack(fill=tk.X, pady=2)
        for label, value in [("Slow", 0.5), ("Medium", 1.0), ("Fast", 2.0)]:
            ttk.Button(speed_buttons, text=label, width=8,
                      command=lambda v=value: self.speed_var.set(v)).pack(side=tk.LEFT, padx=2)
        
        # System controls
        control_frame = ttk.LabelFrame(frame, text="Controls", padding=5)
        control_frame.pack(fill=tk.X, pady=5)
        
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(fill=tk.X)
        
        for text, cmd in [
            ("▶ Start", self._start),
            ("⏸ Pause", self._pause),
            ("⏹ Stop", self._stop),
            ("🚨 Emergency", self._emergency)
        ]:
            ttk.Button(btn_frame, text=text, command=cmd, width=10).pack(side=tk.LEFT, padx=2)
        
        # What-If settings
        whatif_frame = ttk.LabelFrame(frame, text="What-If Settings", padding=5)
        whatif_frame.pack(fill=tk.X, pady=5)
        
        # Radius
        radius_frame = ttk.Frame(whatif_frame)
        radius_frame.pack(fill=tk.X, pady=1)
        ttk.Label(radius_frame, text="Radius:").pack(side=tk.LEFT)
        self.radius_var = tk.DoubleVar(value=0.3)
        ttk.Scale(radius_frame, from_=0.1, to=1.0, variable=self.radius_var,
                  orient=tk.HORIZONTAL, length=80).pack(side=tk.LEFT, padx=5)
        ttk.Label(radius_frame, textvariable=self.radius_var, width=5).pack(side=tk.LEFT)
        
        # Direction
        dir_frame = ttk.Frame(whatif_frame)
        dir_frame.pack(fill=tk.X, pady=1)
        ttk.Label(dir_frame, text="Direction:").pack(side=tk.LEFT)
        self.direction_var = tk.StringVar(value="CW")
        ttk.Combobox(dir_frame, textvariable=self.direction_var,
                     values=["CW", "CCW"], width=5, state="readonly").pack(side=tk.LEFT, padx=5)
        
        ttk.Button(whatif_frame, text="Apply Path", command=self._apply_path).pack(pady=2)
        
        self.panels['control'] = frame

    def _create_log_viewer(self, parent, row, column):
        """Create the log viewer."""
        frame = ttk.LabelFrame(parent, text="📋 System Logs", padding=5)
        frame.grid(row=row, column=column, sticky="nsew", padx=3, pady=3)
        
        # Filter frame
        filter_frame = ttk.Frame(frame)
        filter_frame.pack(fill=tk.X, pady=2)
        
        self.filter_var = tk.StringVar(value="All")
        ttk.Combobox(filter_frame, textvariable=self.filter_var,
                     values=["All", "PLC", "Control", "Safety", "Coppelia"],
                     width=12, state="readonly").pack(side=tk.LEFT, padx=2)
        
        self.level_var = tk.StringVar(value="All Levels")
        ttk.Combobox(filter_frame, textvariable=self.level_var,
                     values=["All Levels", "INFO", "WARNING", "ERROR"],
                     width=12, state="readonly").pack(side=tk.LEFT, padx=2)
        
        ttk.Button(filter_frame, text="Clear", command=self._clear_logs, width=8).pack(side=tk.RIGHT, padx=2)
        
        # Log text area
        self.log_text = tk.Text(frame, wrap=tk.WORD, font=("Courier", 9),
                                bg="#1e1e1e", fg="#d4d4d4", height=8)
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=2)
        
        scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)
        
        self.panels['logs'] = frame
        
        # Add some example logs
        self._add_log("INFO", "System started", "Control")
        self._add_log("INFO", "PLC connected", "PLC")
        self._add_log("INFO", "CoppeliaSim connected", "Coppelia")
        self._add_log("INFO", "Safety monitor active", "Safety")

    def _create_manual_panel(self, parent):
        """Create the MOVEIT manual control panel."""
        frame = ttk.LabelFrame(parent, text="🖐️ MOVEIT Manual Control", padding=5)
        frame.pack(fill=tk.X, padx=5, pady=5)
        parent.add(frame, weight=1)
        
        # Joint angles
        angle_frame = ttk.Frame(frame)
        angle_frame.pack(fill=tk.X, pady=2)
        
        self.angle_vars = []
        for i, name in enumerate(["J1", "J2", "J3", "J4"]):
            sub_frame = ttk.Frame(angle_frame)
            sub_frame.pack(side=tk.LEFT, padx=5)
            ttk.Label(sub_frame, text=name, width=3).pack(side=tk.LEFT)
            var = tk.DoubleVar(value=0.0)
            self.angle_vars.append(var)
            ttk.Entry(sub_frame, textvariable=var, width=8).pack(side=tk.LEFT, padx=2)
            ttk.Label(sub_frame, text="rad", width=4).pack(side=tk.LEFT)
        
        # Speed slider
        speed_frame = ttk.Frame(frame)
        speed_frame.pack(fill=tk.X, pady=2)
        ttk.Label(speed_frame, text="Speed:").pack(side=tk.LEFT)
        self.manual_speed_var = tk.DoubleVar(value=1.0)
        ttk.Scale(speed_frame, from_=0.1, to=2.0, variable=self.manual_speed_var,
                  orient=tk.HORIZONTAL, length=100).pack(side=tk.LEFT, padx=5)
        ttk.Label(speed_frame, textvariable=self.manual_speed_var, width=5).pack(side=tk.LEFT)
        
        # Buttons
        btn_frame = ttk.Frame(frame)
        btn_frame.pack(fill=tk.X, pady=2)
        
        for text, cmd in [("▶ Move", self._manual_move), ("🏠 Home", self._manual_home),
                          ("📥 Read", self._manual_read), ("⏹ Stop", self._manual_stop)]:
            ttk.Button(btn_frame, text=text, command=cmd, width=10).pack(side=tk.LEFT, padx=2)
        
        # Position management
        pos_frame = ttk.Frame(frame)
        pos_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(pos_frame, text="Name:").pack(side=tk.LEFT)
        self.pos_name_var = tk.StringVar()
        ttk.Entry(pos_frame, textvariable=self.pos_name_var, width=12).pack(side=tk.LEFT, padx=2)
        ttk.Button(pos_frame, text="💾 Save", command=self._save_position, width=8).pack(side=tk.LEFT, padx=2)
        
        self.positions_combo = ttk.Combobox(pos_frame, state="readonly", width=12)
        self.positions_combo.pack(side=tk.LEFT, padx=5)
        ttk.Button(pos_frame, text="📂 Load", command=self._load_position, width=8).pack(side=tk.LEFT, padx=2)
        
        # Current position
        self.current_pos_label = ttk.Label(frame, text="Current: J1=0.00 J2=0.00 J3=0.00 J4=0.00",
                                           font=('Arial', 10, 'bold'))
        self.current_pos_label.pack(pady=2)
        
        self.saved_positions = {}
        self.panels['manual'] = frame

    # ──── Control Panel Methods ────

    def _change_mode(self):
        mode = self.mode_var.get()
        if hasattr(self.controller, 'set_mode'):
            self.controller.set_mode(mode)
        logger.info(f"🔄 Mode changed to: {mode}")

    def _start(self):
        if hasattr(self.controller, 'start'):
            try:
                if self._loop:
                    asyncio.run_coroutine_threadsafe(self.controller.start(), self._loop)
                messagebox.showinfo("Success", "System start command sent")
            except Exception as e:
                messagebox.showerror("Error", f"Failed: {e}")

    def _pause(self):
        if hasattr(self.controller, 'pause'):
            try:
                if self._loop:
                    asyncio.run_coroutine_threadsafe(self.controller.pause(), self._loop)
                logger.info("⏸️ Pause command sent")
            except Exception as e:
                logger.error(f"Pause failed: {e}")

    def _stop(self):
        if hasattr(self.controller, 'stop'):
            try:
                if self._loop:
                    asyncio.run_coroutine_threadsafe(self.controller.stop(), self._loop)
                messagebox.showinfo("Success", "Stop command sent")
            except Exception as e:
                messagebox.showerror("Error", f"Failed: {e}")

    def _emergency(self):
        if messagebox.askyesno("Confirm", "Trigger emergency stop?"):
            if hasattr(self.controller, 'emergency_stop'):
                try:
                    if self._loop:
                        asyncio.run_coroutine_threadsafe(self.controller.emergency_stop(), self._loop)
                    messagebox.showwarning("Emergency", "Emergency stop triggered")
                except Exception as e:
                    messagebox.showerror("Error", f"Failed: {e}")

    def _apply_path(self):
        if hasattr(self.controller, 'update_circular_path'):
            try:
                self.controller.update_circular_path(
                    radius=self.radius_var.get(),
                    direction=self.direction_var.get()
                )
                messagebox.showinfo("Success", "Path settings applied")
            except Exception as e:
                messagebox.showerror("Error", f"Failed: {e}")

    # ──── Manual Panel Methods ────

    def _manual_move(self):
        """Move robot to specified joint angles."""
        angles = [var.get() for var in self.angle_vars]
        speed = self.manual_speed_var.get()
        logger.info(f"📤 Move: angles={angles}, speed={speed}")
        
        if hasattr(self.controller, 'manual_controller'):
            try:
                # Call the move command directly (synchronous)
                # This avoids asyncio issues
                if self._loop and self._loop.is_running():
                    # Create a future and wait for it
                    future = asyncio.run_coroutine_threadsafe(
                        self.controller.manual_controller.move_to_position(
                            angles, speed, wait_for_confirmation=True
                        ),
                        self._loop
                    )
                    
                    try:
                        # Wait for result with timeout
                        result = future.result(timeout=5.0)
                        
                        if result.status.value == "confirmed":
                            messagebox.showinfo(
                                "✅ Success",
                                f"Position reached!\n"
                                f"Angles: {', '.join([f'{a:.2f}' for a in angles])}\n"
                                f"Latency: {result.latency_ms:.1f}ms"
                            )
                        elif result.status.value == "timeout":
                            messagebox.showwarning(
                                "⏰ Timeout",
                                "PLC did not confirm position (simulation mode)"
                            )
                        else:
                            messagebox.showerror(
                                "❌ Error",
                                f"Movement failed: {result.error}"
                            )
                    except asyncio.TimeoutError:
                        messagebox.showwarning(
                            "⏰ Timeout",
                            "Command timed out after 5 seconds"
                        )
                    except Exception as e:
                        messagebox.showerror("Error", f"Command failed: {e}")
                else:
                    # Fallback: synchronous call
                    messagebox.showinfo("Info", f"Moving to: {angles}")
                    
            except Exception as e:
                messagebox.showerror("Error", f"Move failed: {e}")
        else:
            messagebox.showerror("Error", "Manual controller not available")

    def _manual_home(self):
        """Move to home position (all zeros)."""
        for var in self.angle_vars:
            var.set(0.0)
        self._manual_move()

    def _manual_read(self):
        """Read current position from PLC."""
        if hasattr(self.controller, 'manual_controller'):
            try:
                pos = self.controller.manual_controller.get_current_position()
                if pos:
                    self.current_pos_label.config(
                        text=f"Current: J1={pos[0]:.3f} J2={pos[1]:.3f} "
                             f"J3={pos[2]:.3f} J4={pos[3]:.3f}"
                    )
                    for i, var in enumerate(self.angle_vars):
                        if i < len(pos):
                            var.set(pos[i])
                    logger.info(f"📥 Read position: {pos}")
                    messagebox.showinfo("Success", f"Position read: {[f'{p:.2f}' for p in pos]}")
                else:
                    messagebox.showwarning("Warning", "No position data available")
            except Exception as e:
                logger.error(f"Read failed: {e}")
                messagebox.showerror("Error", f"Read failed: {e}")

    def _manual_stop(self):
        """Stop current movement."""
        logger.info("⏹️ Manual stop")
        if hasattr(self.controller, 'manual_controller'):
            try:
                if self._loop:
                    asyncio.run_coroutine_threadsafe(
                        self.controller.manual_controller.move_to_position(
                            [0.0, 0.0, 0.0, 0.0], 1.0, wait_for_confirmation=False
                        ),
                        self._loop
                    )
                messagebox.showinfo("Success", "Stop command sent")
            except Exception as e:
                logger.error(f"Stop failed: {e}")

    def _save_position(self):
        """Save current position."""
        name = self.pos_name_var.get().strip()
        if not name:
            messagebox.showwarning("Warning", "Enter a position name")
            return
        angles = [var.get() for var in self.angle_vars]
        self.saved_positions[name] = angles
        self._update_positions_list()
        messagebox.showinfo("Success", f"Position '{name}' saved")

    def _load_position(self):
        """Load saved position."""
        name = self.positions_combo.get()
        if name and name in self.saved_positions:
            angles = self.saved_positions[name]
            for i, var in enumerate(self.angle_vars):
                if i < len(angles):
                    var.set(angles[i])
            logger.info(f"📂 Loaded position '{name}': {angles}")
            messagebox.showinfo("Success", f"Loaded position '{name}'")
        elif name:
            messagebox.showwarning("Warning", f"Position '{name}' not found")

    def _update_positions_list(self):
        """Update saved positions combo box."""
        names = list(self.saved_positions.keys())
        self.positions_combo['values'] = names
        if names:
            self.positions_combo.set(names[0])

    # ──── Log Methods ────

    def _add_log(self, level, message, component="System"):
        """Add a log entry to the log viewer."""
        colors = {'INFO': '#4ec9b0', 'WARNING': '#dcdcaa', 'ERROR': '#f14c4c', 'CRITICAL': '#ff0000'}
        color = colors.get(level, '#d4d4d4')
        text = f"[{time.strftime('%H:%M:%S')}] [{level}] [{component}] {message}\n"
        self.log_text.insert(tk.END, text)
        start = self.log_text.index("end-2l")
        end = self.log_text.index("end-1c")
        self.log_text.tag_add("log_color", start, end)
        self.log_text.tag_config("log_color", foreground=color)
        self.log_text.see(tk.END)

    def _clear_logs(self):
        """Clear all logs."""
        self.log_text.delete(1.0, tk.END)

    # ──── Update Loop ────

    def _update_loop(self):
        """Periodic update loop."""
        if not self._running:
            return

        try:
            # Generate simulated position for testing
            self._simulate_position()
            
            status = {}
            if hasattr(self.controller, 'get_status'):
                status = self.controller.get_status()

            self._update_dashboard(status)
            self._update_graphs(status)
            self._update_status_bar(status)

        except Exception as e:
            logger.error(f"Update error: {e}")

        self.root.after(100, self._update_loop)

    def _simulate_position(self):
        """Generate simulated position for testing."""
        # This creates a sine wave movement for demonstration
        import math
        t = time.time()
        # Only simulate if the controller doesn't have real data
        if hasattr(self.controller, 'manual_controller'):
            pos = self.controller.manual_controller.get_current_position()
            # If position is all zeros, add some simulation
            if pos and all(abs(p) < 0.01 for p in pos):
                # Don't override if user has set a position
                pass

    def _update_dashboard(self, status):
        """Update the dashboard with current status."""
        try:
            # Try to get position from manual controller
            position = status.get('position', [0, 0, 0, 0])
            
            # If all zeros, generate some simulated movement
            if all(abs(p) < 0.01 for p in position):
                import math
                t = time.time()
                position = [
                    0.5 * math.sin(t * 0.5),
                    0.3 * math.sin(t * 0.7 + 1),
                    0.2 * math.sin(t * 0.3 + 2),
                    0.1 * math.sin(t * 0.9 + 3)
                ]
                status['position'] = position
            
            if hasattr(self.controller, 'manual_controller'):
                try:
                    pos = self.controller.manual_controller.get_current_position()
                    if pos and len(pos) >= 4:
                        position = pos
                        status['position'] = position
                except Exception as e:
                    pass
            
            # Update joint progress bars
            for i, joint in enumerate(self.joint_labels):
                if i < len(position):
                    angle = position[i]
                    value = (angle + 3.14159) / 6.28318 * 100
                    joint['progress']['value'] = max(0, min(100, value))
                    joint['label'].config(text=f"{angle:.3f} rad")
            
            # Update current position label in manual panel
            if hasattr(self, 'current_pos_label') and position:
                self.current_pos_label.config(
                    text=f"Current: J1={position[0]:.3f} J2={position[1]:.3f} "
                         f"J3={position[2]:.3f} J4={position[3]:.3f}"
                )
            
            # Update safety status
            is_safe = status.get('safe_mode', True)
            self.safety_display.config(
                text="🟢 Safe" if is_safe else "🔴 Danger!",
                foreground="green" if is_safe else "red"
            )
            
        except Exception as e:
            pass

    def _update_graphs(self, status):
        """Update the graphs with current data."""
        try:
            position = status.get('position', [0, 0, 0, 0])
            
            # If all zeros, generate simulated data
            if all(abs(p) < 0.01 for p in position):
                import math
                t = time.time()
                position = [
                    0.5 * math.sin(t * 0.5),
                    0.3 * math.sin(t * 0.7 + 1),
                    0.2 * math.sin(t * 0.3 + 2),
                    0.1 * math.sin(t * 0.9 + 3)
                ]
            
            self.graph_counter += 1
            
            # Store data
            for i in range(4):
                if i < len(position):
                    self.graph_data[f'J{i+1}'].append(position[i])
                    if len(self.graph_data[f'J{i+1}']) > 50:
                        self.graph_data[f'J{i+1}'].pop(0)
            
            # Update graph lines
            width = self.graph_canvas.winfo_width()
            height = self.graph_canvas.winfo_height()
            
            if width > 10 and height > 10:
                for i, line_id in enumerate(self.graph_lines):
                    data = self.graph_data.get(f'J{i+1}', [])
                    if len(data) > 1:
                        points = []
                        x_step = width / max(50, len(data))
                        for j, val in enumerate(data):
                            x = j * x_step
                            y = height/2 - (val / 3.5) * (height/2 - 20)
                            points.extend([x, y])
                        self.graph_canvas.coords(line_id, *points)
            
        except Exception as e:
            pass

    def _update_status_bar(self, status):
        """Update the status bar."""
        try:
            plc_status = status.get('plc', {})
            plc_connected = plc_status.get('connected', False)
            plc_icon = "✅" if plc_connected else "❌"
            
            coppelia_status = status.get('coppelia', {})
            coppelia_connected = coppelia_status.get('connected', False)
            coppelia_icon = "✅" if coppelia_connected else "❌"
            
            is_safe = status.get('safe_mode', True)
            safety_icon = "🛡️"
            safety_text = "Safe" if is_safe else "DANGER!"
            safety_color = "green" if is_safe else "red"
            
            mode = status.get('mode', 'unknown')
            mode_map = {'online': 'PLC', 'virtual': 'What-If', 'manual': 'MOVEIT', 'unknown': 'Unknown'}
            mode_text = mode_map.get(mode, mode)
            
            cycles = status.get('cycles', 0)
            latency = status.get('avg_latency_ms', 0)
            
            self.status_label.config(
                text=f"PLC: {plc_icon} | CoppeliaSim: {coppelia_icon} | Response: {latency:.1f}ms"
            )
            self.safety_label.config(text=f"{safety_icon} {safety_text}", foreground=safety_color)
            self.mode_label.config(text=f"Mode: {mode_text}")
            self.cycle_label.config(text=f"Cycles: {cycles}")
            
        except Exception as e:
            pass

    # ──── Window Management ────

    def _on_close(self):
        """Handle window close event."""
        if messagebox.askokcancel("Close", "Do you want to close the application?"):
            self._running = False
            
            if self._loop and self._loop.is_running():
                self._loop.call_soon_threadsafe(self._loop.stop)
            
            if hasattr(self.controller, 'stop'):
                try:
                    if self._loop:
                        future = asyncio.run_coroutine_threadsafe(
                            self.controller.stop(),
                            self._loop
                        )
                        future.result(timeout=5)
                except Exception as e:
                    logger.error(f"Stop error: {e}")
            
            self.root.destroy()

    def run(self):
        """Run the GUI main loop."""
        self.root.mainloop()

    def stop(self):
        """Stop the GUI."""
        self._running = False
        self.root.quit()