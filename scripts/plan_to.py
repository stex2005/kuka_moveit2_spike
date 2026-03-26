#!/usr/bin/env python3
"""Plan to a named state or joint target without executing.

Usage:
  python3 plan_to.py zero
  python3 plan_to.py default
  python3 plan_to.py --joints 0.0 0.0 0.5 0.0 0.5 0.0
  python3 plan_to.py zero --pipeline pilz_industrial_motion_planner --planner PTP
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, DisplayTrajectory
from sensor_msgs.msg import JointState

NAMED_STATES = {
    "default": [-0.785, -1.74532925, 1.74532925, 0.0, 1.57079633, 0.0],
    "zero": [0.0, 0.0, 0.5, 0.0, 0.5, 0.0],
}

JOINT_NAMES = [
    "joint_a1",
    "joint_a2",
    "joint_a3",
    "joint_a4",
    "joint_a5",
    "joint_a6",
]


def main():
    parser = argparse.ArgumentParser(description="Plan to a target")
    parser.add_argument("target", nargs="?", help="Named state (zero, default)")
    parser.add_argument(
        "--joints", nargs=6, type=float, help="6 joint values in radians"
    )
    parser.add_argument(
        "--pipeline", default="ompl", help="Planning pipeline (default: ompl)"
    )
    parser.add_argument(
        "--planner", default="", help="Planner ID (e.g. RRTConnect, PTP)"
    )
    args = parser.parse_args()

    if args.joints:
        joint_values = args.joints
    elif args.target and args.target in NAMED_STATES:
        joint_values = NAMED_STATES[args.target]
    else:
        print(
            f"Usage: plan_to.py <{'|'.join(NAMED_STATES)}> or --joints j1 j2 j3 j4 j5 j6"
        )
        sys.exit(1)

    rclpy.init()
    node = Node("plan_to")
    client = ActionClient(node, MoveGroup, "/move_action")
    display_pub = node.create_publisher(DisplayTrajectory, "/display_planned_path", 10)

    # Get current joint state for trajectory_start
    current_js = [None]

    def js_cb(msg):
        current_js[0] = msg

    _js_sub = node.create_subscription(JointState, "/joint_states", js_cb, 10)
    # Spin until we get a joint state
    import time as _time

    t0 = _time.time()
    while current_js[0] is None and (_time.time() - t0) < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("Waiting for MoveGroup action server...")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("MoveGroup not available")
        sys.exit(1)

    goal = MoveGroup.Goal()
    goal.request.group_name = "manipulator"
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 10.0
    goal.request.max_velocity_scaling_factor = 0.5
    goal.request.max_acceleration_scaling_factor = 0.5
    goal.request.pipeline_id = args.pipeline
    goal.request.planner_id = args.planner

    constraints = Constraints()
    for name, val in zip(JOINT_NAMES, joint_values):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = val
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    goal.request.goal_constraints.append(constraints)

    goal.planning_options.plan_only = True

    target_str = args.target if args.target else str(joint_values)
    node.get_logger().info(
        f"Planning to {target_str} with {args.pipeline}/{args.planner or 'default'}..."
    )

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    goal_handle = future.result()
    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error("Goal rejected")
        sys.exit(1)

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)

    result = result_future.result()
    if result is None:
        node.get_logger().error("Timeout waiting for result")
        sys.exit(1)

    r = result.result
    if r.error_code.val == 1:
        traj = r.planned_trajectory.joint_trajectory
        pts = len(traj.points)
        dur = (
            traj.points[-1].time_from_start.sec
            + traj.points[-1].time_from_start.nanosec * 1e-9
            if pts > 0
            else 0.0
        )
        node.get_logger().info(f"Plan OK: {pts} points, duration={dur:.3f}s")

        # Publish to RViz with start state
        display = DisplayTrajectory()
        display.trajectory.append(r.planned_trajectory)
        if current_js[0]:

            display.trajectory_start.joint_state = current_js[0]
        display_pub.publish(display)
        node.get_logger().info(
            "Published trajectory to /display_planned_path (visible in RViz)"
        )
        node.get_logger().info("Use execute_to.py to execute")

        # Keep alive so RViz receives the latched message
        _time.sleep(2.0)
    else:
        node.get_logger().error(f"Planning failed: error_code={r.error_code.val}")
        sys.exit(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
