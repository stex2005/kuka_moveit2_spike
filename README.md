# kuka_moveit2_spike

Feasibility spike: KUKA kr_70_r2100 arm with MoveIt2 and ros2_control on ROS2 Humble.

## Quick Start

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select kuka_moveit2_spike --symlink-install
source install/setup.bash

# Launch with RViz
ros2 launch kuka_moveit2_spike bringup.launch.py gui:=true

# Launch headless
ros2 launch kuka_moveit2_spike bringup.launch.py

# CLI tools
python3 scripts/plan_to.py zero
python3 scripts/plan_to.py default --pipeline pilz_industrial_motion_planner --planner PTP
python3 scripts/execute_to.py zero
python3 scripts/execute_to.py --joints 0.3 -0.3 0.8 0.2 0.8 0.2 --velocity 0.3

# Run planner test suite
python3 scripts/test_plan_execute.py
```

## Dependencies

```bash
sudo apt install \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-pilz-industrial-motion-planner \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro
```

## Planner Test Results

| Planner | Type | Status |
|---------|------|--------|
| OMPL RRTConnect | Joint-space | PASS |
| OMPL RRTstar | Joint-space | PASS |
| OMPL PRM | Joint-space | PASS |
| OMPL EST | Joint-space | PASS |
| OMPL KPIECE | Joint-space | PASS |
| Pilz PTP | Joint-space | PASS |
| Pilz LIN | Cartesian | PASS (Z, Y axes) |
| Pilz LIN | Joint-space | FAIL (expected — LIN requires Cartesian goals) |

## Lessons Learned

### YAML type strictness

MoveIt2 on Humble crashes with `InvalidParameterTypeException` if YAML values are integers where doubles are expected. **Every numeric value in config files must have a decimal point.**

Bad:
```yaml
max_acceleration: 10
min_position: -6
max_trans_vel: 1
```

Good:
```yaml
max_acceleration: 10.0
min_position: -6.0
max_trans_vel: 1.0
```

This applies to `joint_limits.yaml`, `pilz_cartesian_limits.yaml`, and any YAML loaded as ROS2 parameters.

### File naming conventions

- `cartesian_limits.yaml` is deprecated — must be `pilz_cartesian_limits.yaml`
- `MoveItConfigsBuilder.planning_pipelines()` looks for `{pipeline_name}_planning.yaml`. For Pilz, the file must be named `pilz_industrial_motion_planner_planning.yaml`, not `pilz_planning.yaml`.

### MoveItSimpleControllerManager config

The controller manager params must be **flat** (not nested under `moveit_simple_controller_manager:`):

```yaml
# This works:
controller_names:
  - joint_trajectory_controller

joint_trajectory_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints: [...]
```

```yaml
# This does NOT work (returns 0 controllers):
moveit_simple_controller_manager:
  controller_names:
    - joint_trajectory_controller
  joint_trajectory_controller:
    ...
```

The `moveit_controller_manager` and `moveit_manage_controllers` params must also be in this file for `MoveItConfigsBuilder.trajectory_execution()` to load them.

### OMPL request adapters

The ROS1 `ompl_planning.yaml` does not include `request_adapters`. Without the `AddTimeOptimalParameterization` adapter, trajectories have **zero timestamps** and `joint_trajectory_controller` rejects them with:

> Time between points 0 and 1 is not strictly increasing

Add to `ompl_planning.yaml`:
```yaml
planning_plugin: ompl_interface/OMPLPlanner
request_adapters: >-
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planner_request_adapters/ResolveConstraintFrames
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints
```

### KDL root link inertia warning

KDL does not support inertia on the root link. Fix by adding a dummy `world` link as the URDF root:

```xml
<link name="world"/>
<joint name="world_to_base" type="fixed">
  <parent link="world"/>
  <child link="base_link"/>
</joint>
```

And in the SRDF:
```xml
<virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_link"/>
```

### Pilz planner constraints

- **PTP** works with joint-space goals
- **LIN** requires Cartesian pose goals (position + orientation constraints). Joint-space goals return `FAILURE` (-1).
- **CIRC** requires Cartesian goals with a center or interim point constraint
- Pilz package name for apt: `ros-humble-pilz-industrial-motion-planner` (not under `moveit-planners-*`)

### RViz execution lag — MotionPlanning plugin is the bottleneck

During trajectory execution, RViz drops to ~1 FPS. Initial suspicion was the heavy STL meshes (~7.5MB total), but **disabling mesh visuals did not help**. The actual bottleneck is the **MotionPlanning plugin's planning scene monitor**, which performs continuous collision checking during execution.

**Fix:** Use a standalone **RobotModel** display for smooth real-time visualization. Only enable the MotionPlanning display when you need interactive planning (drag goals, plan button). CLI scripts (`plan_to.py`, `execute_to.py`) work without the MotionPlanning display.

With RobotModel only: smooth execution at full frame rate.
With MotionPlanning enabled: ~1 FPS during execution.

### MoveGroup action result

`planning_time` in `MoveGroup.Result` is always 0.0 when `plan_only=False` on Humble. Use trajectory duration from the last waypoint's `time_from_start` instead.

### Shared memory transport errors

FastRTPS shared memory errors (`Failed init_port`) can cause communication lag:
```
RTPS_TRANSPORT_SHM Error: Failed init_port fastrtps_port7411
```
Fix: `sudo rm -rf /dev/shm/fastrtps_*` and relaunch.

## What's Not Covered

- **kuka_drivers** (EKI/RSI hardware interfaces) — not evaluated yet
- **Physical robot** — all testing on fake hardware (`mock_components/GenericSystem`)
- **Pilz CIRC** — not tested (needs center/interim point constraints)
- **Mesh decimation** — STL files are raw from kuka_experimental

## Package Structure

```
kuka_moveit2_spike/
├── CMakeLists.txt
├── package.xml
├── .gitignore
├── urdf/
│   ├── kr_70_r2100.urdf.xacro          # Arm-only URDF with world link
│   └── kr_70_r2100.ros2_control.xacro   # Fake/real hardware toggle
├── srdf/
│   └── kr_70_r2100.srdf.xacro          # Group, named states, collision pairs
├── meshes/                               # 8 STL files from kuka_experimental
├── config/
│   ├── joint_limits.yaml
│   ├── pilz_cartesian_limits.yaml
│   ├── kinematics.yaml
│   ├── ompl_planning.yaml
│   ├── pilz_industrial_motion_planner_planning.yaml
│   ├── moveit_controllers.yaml
│   ├── ros2_controllers.yaml
│   └── moveit.rviz
├── launch/
│   └── bringup.launch.py               # gui:=true for RViz
└── scripts/
    ├── plan_to.py                       # Plan only (visualize in RViz)
    ├── execute_to.py                    # Plan + execute
    └── test_plan_execute.py             # Planner test suite
```
