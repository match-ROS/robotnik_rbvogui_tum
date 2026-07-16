# RB-VOGUI Topic Contract

The generic AM demo should connect to the Robotnik simulation through launch arguments,
not hardcoded topic names.

## Candidate Inputs

Base pose:

- Preferred AM topic: `/robot_pose`
- Type: `geometry_msgs/msg/PoseStamped`
- Source in the validated launch: Gazebo
  `/world/robotnik_simple/dynamic_pose/info`, bridged to ROS and filtered in
  `robotnik_rbvogui_tum`.
- Frame ID: `map`

TCP/nozzle pose:

- Topic: `/current_tcp_pose`
- Type: `geometry_msgs/msg/PoseStamped`
- Source: ROS TF lookup from `map` to `robot_arm_nozzle_tip`
- Frame ID: `map`

## Documented Outputs

Base command:

- Preferred first command topic: `/robot/robotnik_base_control/cmd_vel_unstamped`
- Type: `geometry_msgs/msg/Twist`
- Reason: generic base followers can publish x, y, and yaw velocity without
  platform-specific stamping.

Fallback command topic:

- Topic: `/robot/robotnik_base_control/cmd_vel`
- Type: `geometry_msgs/msg/TwistStamped`
- Use only if the active controller requires stamped messages.

The unstamped command topic is runtime-verified with
`rbvogui_ur_standard_control.launch.py`. That launch keeps Robotnik's documented topic
name as the public platform command input, then converts it to standard steering and
wheel joint-group controller commands inside `robotnik_rbvogui_tum`.

Robotnik's released Jazzy base controller still crashes Gazebo on its first update.
Use the local standard-controller launch unless a compatible Robotnik controller
binary or source checkout is available.

## Generic Follower Parameters

The later `base_trajectory_follower` package should expose:

- `path_topic`
- `robot_pose_topic`
- `cmd_vel_topic`
- `cmd_vel_stamped`
- `base_frame`
- `world_frame`
- `lookahead_distance`
- `xy_goal_tolerance`
- `yaw_goal_tolerance`
- `max_vx`
- `max_vy`
- `max_wz`
- `stale_pose_timeout`
