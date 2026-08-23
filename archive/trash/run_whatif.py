#!/usr/bin/env python3
"""
run_whatif.py - What-If Mode (All Axes Motion)

This module provides a What-If simulation mode that moves all 6 axes
of the robot in a coordinated motion pattern.

Key Concepts:
- All-Axes Motion: Moves all 6 axes simultaneously
- Coordinated Motion: Each axis moves with a different pattern
- Continuous Motion: Runs indefinitely until stopped
- Real-Time Logging: Shows current positions periodically
- Physics-Enabled: Uses force mode with PID control

The What-If mode is useful for:
    - Demonstrating robot capabilities
    - Testing motion patterns
    - Visualizing complex movements
    - Training and education
"""

import sys
import os
import asyncio
import math
import time

# --- Windows Event Loop Fix ---
if sys.platform == 'win32':
    try:
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    except:
        pass

# --- Project Setup ---
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from utils.logging_config import setup_logging
setup_logging()

import logging
logger = logging.getLogger("RobotTwin.WhatIf")


class CoppeliaDirect:
    """
    Direct CoppeliaSim controller for What-If mode.

    This class provides:
        1. Physics-Enabled Motion: Force mode with PID control
        2. All-Axes Support: Controls all 6 axes
        3. Joint Limits: Validates angles against safe limits
        4. IK Support: For accurate positioning
        5. Disconnection: Clean shutdown

    The controller supports 6 axes (A1-A6) with configurable limits.
    """

    def __init__(self):
        """Initialize the direct CoppeliaSim controller."""
        self.sim_client = None
        self.sim = None
        self.simIK = None
        self.simBase = None
        self.simTip = None
        self.simTarget = None
        self.all_joints = []
        self.tool_joint = None
        self.simToIk = {}
        self.ikEnv = None
        self.ikGroup_u = None
        self.ikGroup_d = None
        self._connected = False

        # Joint limits for 6 axes (radians)
        self.joint_limits = [
            (-2.8, 2.8),   # A1: Base rotation
            (-2.0, 2.0),   # A2: Shoulder
            (-2.0, 2.0),   # A3: Elbow
            (-3.0, 3.0),   # A4: Wrist 1
            (-3.0, 3.0),   # A5: Wrist 2
            (-3.0, 3.0),   # A6: Wrist 3
        ]

    def connect(self):
        """
        Connect to CoppeliaSim with physics enabled.

        Returns:
            bool: True if connection was successful
        """
        try:
            from coppeliasim_zmqremoteapi_client import RemoteAPIClient

            self.sim_client = RemoteAPIClient()
            self.sim = self.sim_client.require('sim')
            self.simIK = self.sim_client.require('simIK')

            # --- Simulation Setup ---
            self.sim_client.setStepping(True)
            self.sim.startSimulation()
            self.sim.setFloatParam(self.sim.floatparam_simulation_time_step, 0.01)

            # --- Robot Discovery ---
            all_objs = self.sim.getObjectsInTree(self.sim.handle_scene)
            self.simBase, self.simTip, self.simTarget = None, None, None

            for obj in all_objs:
                alias = self.sim.getObjectAlias(obj)
                if not alias:
                    continue
                if 'IRB140' in alias or 'robot' in alias.lower():
                    self.simBase = obj
                elif 'target' in alias.lower():
                    self.simTarget = obj
                elif 'tip' in alias.lower():
                    self.simTip = obj

            self.all_joints = self.sim.getObjectsInTree(self.sim.handle_scene, self.sim.object_joint_type)

            if not self.simBase and self.all_joints:
                self.simBase = self.sim.getObjectParent(self.all_joints[0])

            self.tool_joint = self.all_joints[-1] if self.all_joints else None

            # --- Joint Configuration (Force Mode) ---
            for j in self.all_joints:
                self.sim.setJointMode(j, self.sim.jointmode_force, 0)
                self.sim.setJointInterval(j, True, [-3.0, 3.0])
                self.sim.setJointTargetVelocity(j, 0.0)
                self.sim.setJointTargetPosition(j, 0.0)
                self.sim.setJointMaxForce(j, 50.0)

            # --- IK Setup ---
            if all([self.simBase, self.simTip, self.simTarget]):
                self.ikEnv = self.simIK.createEnvironment()

                self.ikGroup_u = self.simIK.createGroup(self.ikEnv)
                self.simIK.setGroupCalculation(
                    self.ikEnv, self.ikGroup_u,
                    self.simIK.method_pseudo_inverse, 0, 10
                )

                ik_el_u, self.simToIk, _ = self.simIK.addElementFromScene(
                    self.ikEnv, self.ikGroup_u,
                    self.simBase, self.simTip, self.simTarget,
                    self.simIK.constraint_pose
                )
                self.simIK.setElementConstraints(
                    self.ikEnv, self.ikGroup_u,
                    ik_el_u, self.simIK.constraint_position
                )

                self.ikGroup_d = self.simIK.createGroup(self.ikEnv)
                self.simIK.setGroupCalculation(
                    self.ikEnv, self.ikGroup_d,
                    self.simIK.method_damped_least_squares, 0.3, 99
                )

                ik_el_d, _, _ = self.simIK.addElementFromScene(
                    self.ikEnv, self.ikGroup_d,
                    self.simBase, self.simTip, self.simTarget,
                    self.simIK.constraint_pose
                )
                self.simIK.setElementConstraints(
                    self.ikEnv, self.ikGroup_d,
                    ik_el_d, self.simIK.constraint_position
                )

            self._connected = True
            logger.info("✅ Connected to CoppeliaSim")
            return True

        except Exception as e:
            logger.error(f"❌ Connection failed: {e}")
            return False

    def move_to_angles(self, angles):
        """
        Move the robot to the specified joint angles.

        Args:
            angles (List[float]): List of joint angles in radians
        """
        if not self._connected:
            return

        try:
            # Clamp angles to safe limits
            clamped_angles = []
            for i, angle in enumerate(angles):
                if i < len(self.joint_limits):
                    lo, hi = self.joint_limits[i]
                    clamped_angles.append(max(lo, min(hi, angle)))
                else:
                    clamped_angles.append(angle)

            # Apply angles to joints
            for i, j in enumerate(self.all_joints[:6]):
                if i < len(clamped_angles):
                    self.sim.setJointTargetPosition(j, clamped_angles[i])
                    self.sim.setJointTargetVelocity(j, 0.0)

            self.sim_client.step()

        except Exception as e:
            logger.error(f"❌ Movement failed: {e}")

    def disconnect(self):
        """Disconnect from CoppeliaSim."""
        if self.sim_client:
            try:
                self.sim.stopSimulation()
            except:
                pass
        self._connected = False


async def main():
    """Main entry point for What-If mode."""
    logger.info("=" * 70)
    logger.info("🔄 What-If Mode - All Axes Motion")
    logger.info("=" * 70)

    coppelia = CoppeliaDirect()
    if not coppelia.connect():
        logger.error("❌ Failed to connect to CoppeliaSim")
        return

    logger.info("✅ CoppeliaSim ready")
    logger.info("🔄 Running motion... Press Ctrl+C to stop")

    try:
        t = 0

        while True:
            t += 0.02

            # ============================================================
            # All 6 Axes Motion Pattern
            # ============================================================
            # Each axis moves with a different pattern for visual variety

            a1 = t * 0.3                         # Continuous rotation (base)
            a2 = 0.3 + 0.2 * math.sin(t * 0.3)   # Elevation (shoulder)
            a3 = -0.1 + 0.15 * math.sin(t * 0.4) # Oscillation (elbow)
            a4 = 0.2 * math.sin(t * 0.5)         # Oscillation (wrist 1)
            a5 = 0.15 * math.cos(t * 0.6)        # Oscillation (wrist 2)
            a6 = 0.0                              # Fixed (tool)

            angles = [a1, a2, a3, a4, a5, a6]

            # Log every 50 cycles
            if int(t * 10) % 50 == 0:
                logger.info(
                    f"📍 A1={a1:.2f}, A2={a2:.2f}, A3={a3:.2f}, "
                    f"A4={a4:.2f}, A5={a5:.2f}, A6={a6:.2f}"
                )

            coppelia.move_to_angles(angles)
            await asyncio.sleep(0.02)

    except KeyboardInterrupt:
        logger.info("⏹️ Stopping...")

    coppelia.disconnect()
    logger.info("👋 Goodbye!")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("⏹️ Stopped by user")