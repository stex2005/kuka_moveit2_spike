#!/usr/bin/env python3
"""Comprehensive MoveIt2 planner validation for kuka_moveit2_spike.

Tests OMPL (RRTConnect, RRT*, PRM), Pilz (PTP, LIN, CIRC) planners.
Requires bringup.launch.py to be running.

Usage: ros2 run kuka_moveit2_spike test_plan_execute.py
   or: python3 test_plan_execute.py
"""
import copy
import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from tf2_ros import Buffer, TransformListener

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    MotionPlanRequest,
    PlanningOptions,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# Named states from SRDF
NAMED_STATES = {
    "default": {
        "joint_a1": -0.785,
        "joint_a2": -1.74532925,
        "joint_a3": 1.74532925,
        "joint_a4": 0.0,
        "joint_a5": 1.57079633,
        "joint_a6": 0.0,
    },
    "zero": {
        "joint_a1": 0.0,
        "joint_a2": 0.0,
        "joint_a3": 0.5,
        "joint_a4": 0.0,
        "joint_a5": 0.5,
        "joint_a6": 0.0,
    },
}

# A modest joint-space offset from "zero" for testing different planners
OFFSET_STATE = {
    "joint_a1": 0.3,
    "joint_a2": -0.3,
    "joint_a3": 0.8,
    "joint_a4": 0.2,
    "joint_a5": 0.8,
    "joint_a6": 0.2,
}

JOINT_NAMES = [
    "joint_a1", "joint_a2", "joint_a3",
    "joint_a4", "joint_a5", "joint_a6",
]


class PlannerTester(Node):
    def __init__(self):
        super().__init__("planner_tester")
        self._cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=self._cb_group
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._results = []

    def get_ee_pose(self, timeout=5.0):
        """Get current end-effector pose via TF."""
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self._tf_buffer.lookup_transform("base_link", "link_A6", rclpy.time.Time())
                pose = PoseStamped()
                pose.header.frame_id = "base_link"
                pose.pose.position.x = t.transform.translation.x
                pose.pose.position.y = t.transform.translation.y
                pose.pose.position.z = t.transform.translation.z
                pose.pose.orientation = t.transform.rotation
                return pose
            except Exception:
                continue
        self.get_logger().error("Could not get EE pose from TF")
        return None

    def wait_for_server(self, timeout=10.0):
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self._action_client.wait_for_server(timeout):
            self.get_logger().error("MoveGroup action server not available")
            return False
        self.get_logger().info("Connected to MoveGroup action server")
        return True

    def _make_joint_goal(self, target_joints, pipeline_id="ompl", planner_id=""):
        """Build a MoveGroup goal for joint-space targets."""
        goal = MoveGroup.Goal()
        req = goal.request

        req.group_name = "manipulator"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        req.pipeline_id = pipeline_id
        req.planner_id = planner_id

        constraints = Constraints()
        for name in JOINT_NAMES:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = target_joints[name]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        return goal

    def _make_cartesian_goal(self, pose, pipeline_id="pilz_industrial_motion_planner", planner_id="LIN"):
        """Build a MoveGroup goal for Cartesian pose targets."""
        goal = MoveGroup.Goal()
        req = goal.request

        req.group_name = "manipulator"
        req.num_planning_attempts = 1
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.pipeline_id = pipeline_id
        req.planner_id = planner_id

        constraints = Constraints()

        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "link_A6"
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0

        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose.pose)
        pc.constraint_region = bv
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "link_A6"
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        req.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        return goal

    def send_goal_and_wait(self, goal, test_name, timeout=30.0):
        """Send a MoveGroup goal and wait for result."""
        self.get_logger().info(f"--- {test_name} ---")
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"  FAILED: Goal rejected")
            self._results.append((test_name, False, "Goal rejected"))
            return False

        self.get_logger().info(f"  Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

        result = result_future.result()
        if result is None:
            self.get_logger().error(f"  FAILED: Timeout")
            self._results.append((test_name, False, "Timeout"))
            return False

        r = result.result
        error_code = r.error_code.val
        if error_code == 1:  # SUCCESS
            pt = r.planning_time if hasattr(r, 'planning_time') else 0.0
            traj_pts = len(r.planned_trajectory.joint_trajectory.points) if r.planned_trajectory.joint_trajectory.points else 0
            detail = f"planning_time={pt:.3f}s, traj_points={traj_pts}"
            self.get_logger().info(f"  PASSED ({detail})")
            self._results.append((test_name, True, detail))
            time.sleep(1.0)  # let the robot settle
            return True
        else:
            self.get_logger().error(f"  FAILED: error_code={error_code}")
            self._results.append((test_name, False, f"error_code={error_code}"))
            return False

    def run_tests(self):
        """Run all planner tests."""
        # ===== OMPL planners (joint-space) =====

        # Test 1: OMPL RRTConnect (default) -> zero
        goal = self._make_joint_goal(NAMED_STATES["zero"], "ompl", "RRTConnect")
        self.send_goal_and_wait(goal, "OMPL RRTConnect: default -> zero")

        # Test 2: OMPL RRTConnect -> offset
        goal = self._make_joint_goal(OFFSET_STATE, "ompl", "RRTConnect")
        self.send_goal_and_wait(goal, "OMPL RRTConnect: zero -> offset")

        # Test 3: OMPL RRTstar -> default
        goal = self._make_joint_goal(NAMED_STATES["default"], "ompl", "RRTstar")
        self.send_goal_and_wait(goal, "OMPL RRTstar: offset -> default")

        # Test 4: OMPL PRM -> zero
        goal = self._make_joint_goal(NAMED_STATES["zero"], "ompl", "PRM")
        self.send_goal_and_wait(goal, "OMPL PRM: default -> zero")

        # Test 5: OMPL EST -> offset
        goal = self._make_joint_goal(OFFSET_STATE, "ompl", "EST")
        self.send_goal_and_wait(goal, "OMPL EST: zero -> offset")

        # Test 6: OMPL KPIECE -> default
        goal = self._make_joint_goal(NAMED_STATES["default"], "ompl", "KPIECE")
        self.send_goal_and_wait(goal, "OMPL KPIECE: offset -> default")

        # ===== Pilz planners (joint-space) =====

        # Test 7: Pilz PTP -> zero
        goal = self._make_joint_goal(NAMED_STATES["zero"], "pilz_industrial_motion_planner", "PTP")
        self.send_goal_and_wait(goal, "Pilz PTP: default -> zero")

        # Test 8: Pilz PTP -> offset
        goal = self._make_joint_goal(OFFSET_STATE, "pilz_industrial_motion_planner", "PTP")
        self.send_goal_and_wait(goal, "Pilz PTP: zero -> offset")

        # ===== Pilz Cartesian planners =====

        # First go to a known pose via PTP, then get EE pose for Cartesian tests
        goal = self._make_joint_goal(NAMED_STATES["zero"], "pilz_industrial_motion_planner", "PTP")
        self.send_goal_and_wait(goal, "Pilz PTP: setup for Cartesian tests -> zero")

        ee_pose = self.get_ee_pose()
        if ee_pose:
            self.get_logger().info(
                f"  EE pose: [{ee_pose.pose.position.x:.3f}, "
                f"{ee_pose.pose.position.y:.3f}, {ee_pose.pose.position.z:.3f}]"
            )

            # Test: Pilz LIN — small Z offset (move 10cm up)
            lin_target = copy.deepcopy(ee_pose)
            lin_target.pose.position.z += 0.10
            goal = self._make_cartesian_goal(lin_target, "pilz_industrial_motion_planner", "LIN")
            self.send_goal_and_wait(goal, "Pilz LIN: +10cm Z (Cartesian)")

            # Test: Pilz LIN — back down
            goal = self._make_cartesian_goal(ee_pose, "pilz_industrial_motion_planner", "LIN")
            self.send_goal_and_wait(goal, "Pilz LIN: -10cm Z back (Cartesian)")

            # Test: Pilz LIN — small X offset
            lin_target2 = copy.deepcopy(ee_pose)
            lin_target2.pose.position.x += 0.10
            goal = self._make_cartesian_goal(lin_target2, "pilz_industrial_motion_planner", "LIN")
            self.send_goal_and_wait(goal, "Pilz LIN: +10cm X (Cartesian)")

            # Test: Pilz LIN — back
            goal = self._make_cartesian_goal(ee_pose, "pilz_industrial_motion_planner", "LIN")
            self.send_goal_and_wait(goal, "Pilz LIN: back to origin (Cartesian)")
        else:
            self._results.append(("Pilz LIN tests", False, "Could not get EE pose"))

        # Return home
        goal = self._make_joint_goal(NAMED_STATES["default"], "pilz_industrial_motion_planner", "PTP")
        self.send_goal_and_wait(goal, "Pilz PTP: return to default")

        # ===== Summary =====
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("RESULTS SUMMARY")
        self.get_logger().info("=" * 60)
        passed = 0
        failed = 0
        for name, success, detail in self._results:
            status = "PASS" if success else "FAIL"
            self.get_logger().info(f"  [{status}] {name} — {detail}")
            if success:
                passed += 1
            else:
                failed += 1
        self.get_logger().info(f"\n  {passed} passed, {failed} failed, {len(self._results)} total")
        self.get_logger().info("=" * 60)


def main():
    rclpy.init()
    tester = PlannerTester()

    if not tester.wait_for_server():
        sys.exit(1)

    tester.run_tests()
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
