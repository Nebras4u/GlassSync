#!/usr/bin/env python3
"""
run_direct.py - Direct CoppeliaSim Runner with Physics

This module provides a direct runner for CoppeliaSim with physics enabled,
allowing the robot to move without collapsing. It includes a ZMQ server
for receiving commands and a test sequence for verification.

Key Concepts:
- Physics Simulation: Force mode with PID control for stable motion
- Joint Configuration: Force mode with max force for gravity compensation
- IK Setup: Inverse kinematics for accurate positioning
- ZMQ Server: Receives commands for what-if and manual modes
- Test Sequence: Demonstrates basic movement patterns

The direct runner is useful for testing CoppeliaSim integration
without the full Digital Twin system.
"""

import sys
import os
import asyncio
import math
import time
import json

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
logger = logging.getLogger("RobotTwin.Direct")

import zmq
import zmq.asyncio


class CoppeliaDirect:
    """
    Direct CoppeliaSim controller with physics.

    This class provides:
        1. Physics-Enabled Motion: Force mode with PID control
        2. Joint Configuration: Force mode with max force
        3. IK Setup: For accurate positioning
        4. Movement Methods: Direct and smooth movement
        5. Disconnection: Clean shutdown

    The physics configuration prevents the robot from collapsing
    due to gravity by using force mode with sufficient torque.
    """

    def __init__(self):
        """Initialize the direct CoppeliaSim controller."""
        self.sim_client = None
        self.sim = None
        self.simIK = None

        # Robot parts
        self.simBase = None
        self.simTip = None
        self.simTarget = None
        self.all_joints = []
        self.tool_joint = None
        self.simToIk = {}

        # IK environment
        self.ikEnv = None
        self.ikGroup_u = None
        self.ikGroup_d = None

        # State
        self._connected = False
        self._running = False

        # Joint limits (radians)
        self.joint_limits = [
            (-2.8, 2.8),   # Joint 1: Base rotation
            (-2.0, 2.0),   # Joint 2: Shoulder
            (-2.0, 2.0),   # Joint 3: Elbow
            (-3.0, 3.0),   # Joint 4: Wrist
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

            # --- Joint Configuration (Force Mode with PID) ---
            for j in self.all_joints:
                self.sim.setJointMode(j, self.sim.jointmode_force, 0)
                self.sim.setJointInterval(j, True, [-3.0, 3.0])
                self.sim.setJointTargetVelocity(j, 0.0)
                self.sim.setJointTargetPosition(j, 0.0)
                # Max force (torque) to compensate for gravity
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
            logger.info("✅ Connected to CoppeliaSim (physics enabled)")
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

            # Apply angles with PID control
            for i, j in enumerate(self.all_joints[:4]):
                if i < len(clamped_angles):
                    self.sim.setJointTargetPosition(j, clamped_angles[i])
                    self.sim.setJointTargetVelocity(j, 0.0)

            # Lock redundant axes
            if len(self.all_joints) >= 6:
                self.sim.setJointTargetPosition(self.all_joints[3], 0.0)
                self.sim.setJointTargetPosition(self.all_joints[4], 0.0)

            # Apply tool angle
            if self.tool_joint and len(clamped_angles) >= 4:
                self.sim.setJointTargetPosition(self.tool_joint, clamped_angles[3])

            # Step simulation
            self.sim_client.step()

        except Exception as e:
            logger.error(f"❌ Movement failed: {e}")

    def move_smooth(self, target_angles, steps=20):
        """
        Smoothly move to target angles.

        Args:
            target_angles (List[float]): Target joint angles
            steps (int): Number of interpolation steps
        """
        if not self._connected:
            return

        # Get current angles
        current_angles = []
        for i, j in enumerate(self.all_joints[:4]):
            current_angles.append(self.sim.getJointPosition(j))
        while len(current_angles) < 4:
            current_angles.append(0.0)

        # Interpolate and move
        for i in range(steps):
            t = i / (steps - 1)
            inter_angles = [
                current_angles[0] + (target_angles[0] - current_angles[0]) * t,
                current_angles[1] + (target_angles[1] - current_angles[1]) * t,
                current_angles[2] + (target_angles[2] - current_angles[2]) * t,
                current_angles[3] + (target_angles[3] - current_angles[3]) * t,
            ]
            self.move_to_angles(inter_angles)
            time.sleep(0.03)

    def disconnect(self):
        """Disconnect from CoppeliaSim."""
        if self.sim_client:
            try:
                self.sim.stopSimulation()
            except:
                pass
        self._connected = False


class ZMQServer:
    """
    ZMQ server for receiving commands.

    This server listens for commands on the 'what_if' and 'manual' topics
    and executes them on the CoppeliaSim controller.
    """

    def __init__(self, coppelia):
        """
        Initialize the ZMQ server.

        Args:
            coppelia (CoppeliaDirect): The CoppeliaSim controller
        """
        self.coppelia = coppelia
        self._running = False
        self.ctx = zmq.asyncio.Context()
        self.socket = self.ctx.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5556")
        self.socket.setsockopt(zmq.SUBSCRIBE, b"what_if")
        self.socket.setsockopt(zmq.SUBSCRIBE, b"manual")

    async def run(self):
        """
        Run the ZMQ server.

        This method listens for commands and executes them.
        """
        self._running = True
        logger.info("📡 ZMQ Server ready")

        while self._running:
            try:
                msg = await asyncio.wait_for(
                    self.socket.recv_multipart(),
                    timeout=0.1
                )
                topic, payload = msg[0], msg[1]
                data = json.loads(payload.decode())

                if topic == b"what_if":
                    angles = data.get('joint_angles', [0, 0, 0, 0])
                    self.coppelia.move_to_angles(angles)

                elif topic == b"manual":
                    angles = data.get('joint_angles', [0, 0, 0, 0])
                    self.coppelia.move_smooth(angles)

            except asyncio.TimeoutError:
                continue
            except Exception as e:
                logger.error(f"❌ ZMQ error: {e}")

        self.socket.close()
        self.ctx.term()


async def main():
    """Main entry point."""
    logger.info("=" * 70)
    logger.info("🚀 Direct CoppeliaSim Runner")
    logger.info("=" * 70)

    coppelia = CoppeliaDirect()
    if not coppelia.connect():
        logger.error("❌ Failed to connect to CoppeliaSim")
        return

    logger.info("✅ CoppeliaSim ready (physics enabled)")

    # Start ZMQ server
    server = ZMQServer(coppelia)
    server_task = asyncio.create_task(server.run())

    # Test sequence
    logger.info("🔄 Running test sequence...")

    test_angles = [
        [0.0, 0.0, 0.0, 0.0],
        [0.5, 0.3, -0.2, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [-0.5, 0.3, 0.2, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ]

    for angles in test_angles:
        logger.info(f"📤 Moving to: {angles}")
        coppelia.move_smooth(angles, steps=20)
        await asyncio.sleep(0.5)

    logger.info("✅ Test complete")
    logger.info("📡 Waiting for ZMQ commands...")

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.info("⏹️ Stopping...")

    # Cleanup
    server._running = False
    server_task.cancel()
    coppelia.disconnect()
    logger.info("👋 Goodbye!")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("⏹️ Stopped by user")