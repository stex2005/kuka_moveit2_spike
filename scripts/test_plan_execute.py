#!/usr/bin/env python3
"""Minimal MoveIt2 plan+execute validation for kuka_moveit2_spike.

Requires bringup.launch.py to be running.
Usage: python3 test_plan_execute.py
"""
import sys
import rclpy
from rclpy.logging import get_logger

logger = get_logger("test_plan_execute")


def main():
    rclpy.init()

    try:
        from moveit_py.moveit_py import MoveItPy
    except ImportError:
        logger.error("moveit_py not available — install ros-humble-moveit-py")
        sys.exit(1)

    logger.info("Initializing MoveItPy...")
    moveit = MoveItPy(node_name="test_plan_execute")
    arm = moveit.get_planning_component("manipulator")

    # ---- Test 1: OMPL plan to "zero" named state ----
    logger.info("=== Test 1: OMPL plan to 'zero' state ===")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="zero")
    plan_result = arm.plan()
    if plan_result:
        logger.info("OMPL plan succeeded — executing...")
        moveit.execute(plan_result.trajectory, controllers=[])
        logger.info("Test 1 PASSED")
    else:
        logger.error("Test 1 FAILED — OMPL planning failed")

    # ---- Test 2: OMPL plan back to "default" ----
    logger.info("=== Test 2: OMPL plan to 'default' state ===")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="default")
    plan_result = arm.plan()
    if plan_result:
        logger.info("OMPL plan succeeded — executing...")
        moveit.execute(plan_result.trajectory, controllers=[])
        logger.info("Test 2 PASSED")
    else:
        logger.error("Test 2 FAILED — OMPL planning failed")

    # ---- Test 3: Pilz PTP plan to "zero" ----
    logger.info("=== Test 3: Pilz PTP plan to 'zero' state ===")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="zero")
    plan_result = arm.plan(pipeline_id="pilz_industrial_motion_planner", planner_id="PTP")
    if plan_result:
        logger.info("Pilz PTP plan succeeded — executing...")
        moveit.execute(plan_result.trajectory, controllers=[])
        logger.info("Test 3 PASSED")
    else:
        logger.error("Test 3 FAILED — Pilz PTP planning failed")

    logger.info("=== All tests complete ===")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
