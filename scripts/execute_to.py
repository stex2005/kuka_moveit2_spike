#!/usr/bin/env python3
"""Plan and execute to a named state or joint target.

Usage:
  python3 execute_to.py zero
  python3 execute_to.py default
  python3 execute_to.py --joints 0.0 0.0 0.5 0.0 0.5 0.0
  python3 execute_to.py zero --pipeline pilz_industrial_motion_planner --planner PTP
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

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
    parser = argparse.ArgumentParser(description="Plan and execute to a target")
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
    parser.add_argument(
        "--velocity", type=float, default=0.5, help="Max velocity scaling (0-1)"
    )
    parser.add_argument(
        "--acceleration", type=float, default=0.5, help="Max acceleration scaling (0-1)"
    )
    args = parser.parse_args()

    if args.joints:
        joint_values = args.joints
    elif args.target and args.target in NAMED_STATES:
        joint_values = NAMED_STATES[args.target]
    else:
        print(
            f"Usage: execute_to.py <{'|'.join(NAMED_STATES)}> or --joints j1 j2 j3 j4 j5 j6"
        )
        sys.exit(1)

    rclpy.init()
    node = Node("execute_to")
    client = ActionClient(node, MoveGroup, "/move_action")

    node.get_logger().info("Waiting for MoveGroup action server...")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("MoveGroup not available")
        sys.exit(1)

    goal = MoveGroup.Goal()
    goal.request.group_name = "manipulator"
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 10.0
    goal.request.max_velocity_scaling_factor = args.velocity
    goal.request.max_acceleration_scaling_factor = args.acceleration
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

    goal.planning_options.plan_only = False
    goal.planning_options.replan = False

    target_str = args.target if args.target else str(joint_values)
    node.get_logger().info(
        f"Planning + executing to {target_str} "
        f"with {args.pipeline}/{args.planner or 'default'} "
        f"(vel={args.velocity}, acc={args.acceleration})..."
    )

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    goal_handle = future.result()
    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error("Goal rejected")
        sys.exit(1)

    node.get_logger().info("Goal accepted, executing...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)

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
        node.get_logger().info(f"Done: {pts} points, duration={dur:.3f}s")
    else:
        node.get_logger().error(f"Failed: error_code={r.error_code.val}")
        sys.exit(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
