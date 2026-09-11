# Manipulator controller — ROS 2 Humble

This script uses `rclpy` and the MoveIt 2 `moveit_msgs/action/MoveGroup`
action. Launch your robot's Humble MoveIt configuration, joint-state publisher,
and trajectory controllers before running it. The default action is
`/move_action`, with planning groups `arm` and `gripper`.

On your Ubuntu ROS 2 Humble machine, install dependencies if needed:

```bash
sudo apt install ros-humble-rclpy ros-humble-action-msgs ros-humble-moveit-msgs ros-humble-moveit
source /opt/ros/humble/setup.bash
source /path/to/your/robot_ws/install/setup.bash
```

The default arm joint names are `joint1`, `joint2`, `joint3`, and `joint4`,
and the default gripper joint is `gripper_left_joint`. With these names, run:

```bash
python3 manipulator_controller.py
```

If your installed robot configuration uses different names, override them with
ROS parameters. The arm names must match the order of the four indexed joints
in the original script. Use the independently controlled gripper joint, not a
mimic joint. For example, to override just the gripper name:

```bash
python3 manipulator_controller.py --ros-args \
  -p gripper_joint_name:=gripper_joint
```

Running the script moves the arm to zero and then performs the original pick-up
sequence. Empty or invalid arm-name lists and empty gripper names cause an error
before motion is requested. The
original arm targets and gripper positions (`0.019` open, `-0.01` closed) are
preserved; they must be valid for your robot's configured joint limits.

Additional ROS parameters:

- `arm_joint_names`: four joint names in script order; defaults to
  `[joint1, joint2, joint3, joint4]`.
- `gripper_joint_name`: controlled gripper joint; defaults to `gripper_left_joint`.
- `procedure`: `pick_up` (default), `drop_off`, `init_pose`, `reach_front`,
  `reach_back`, `open_gripper`, or `close_gripper`. Every invocation first moves
  the arm to the initial zero pose.
- `arm_group`, `gripper_group`: MoveIt planning group names.
- `move_action`: MoveGroup action name; default `/move_action`.
- `use_sim_time`: set to `true` for a simulator publishing `/clock`.
- `server_timeout`: seconds to wait for the action server, default `10.0`.
- `planning_time`: allowed planning seconds per request, default `5.0`.
- `execution_timeout`: seconds to wait for an accepted goal's result, default
  `120.0`. On timeout the script requests cancellation and exits with an error.

Both groups use velocity and acceleration scaling of `0.1`. Motion failures
stop the sequence. MoveIt must monitor current joint states and have execution
configured for both groups. A successful cancellation request is not confirmation
that execution has already stopped. Goal acceptance waits for the server response.

Importing the module does not move the robot. For programmatic use, initialize
`rclpy`, construct `ManipulatorController`, call `configure()`, then invoke the
motion methods from the main thread. These blocking methods spin the node and
must not be called from ROS callbacks or alongside another executor spinning it.

Validation in the editing environment is limited to syntax and mocked action
responses; ROS 2 and a robot/simulator are not installed here.
