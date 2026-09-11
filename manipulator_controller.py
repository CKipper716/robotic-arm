#!/usr/bin/env python3
"""ROS 2 Humble controller using MoveIt 2's MoveGroup action.

Start the robot's MoveIt move_group and trajectory controllers first.
Joint names default to the documented OpenMANIPULATOR-X model and can be
overridden using ROS parameters. See README.md for an example.
Methods block while spinning this node; call them from the main thread,
not from a ROS callback or an executor that already spins this node.
"""

import sys

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.node import Node


class ManipulatorController(Node):
    def __init__(self):
        super().__init__('manipulator_controller')
        self.declare_parameter(
            'arm_joint_names', ['joint1', 'joint2', 'joint3', 'joint4'])
        self.declare_parameter('gripper_joint_name', 'gripper_left_joint')
        self.declare_parameter('arm_group', 'arm')
        self.declare_parameter('gripper_group', 'gripper')
        self.declare_parameter('move_action', '/move_action')
        self.declare_parameter('procedure', 'pick_up')
        self.declare_parameter('server_timeout', 10.0)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('execution_timeout', 120.0)
        self._active_goal = None
        self._move_client = None

    def configure(self):
        self.arm_joint_names = self.get_parameter('arm_joint_names').value
        self.gripper_joint_name = self.get_parameter('gripper_joint_name').value
        if (len(self.arm_joint_names) != 4
                or any(not name.strip() for name in self.arm_joint_names)
                or len(set(self.arm_joint_names)) != 4):
            raise ValueError('Set arm_joint_names to four distinct joint names '
                             'in the original script order.')
        if not self.gripper_joint_name.strip():
            raise ValueError('Set gripper_joint_name to the controlled gripper joint.')
        for name in ('server_timeout', 'planning_time', 'execution_timeout'):
            value = self.get_parameter(name).value
            if not 0.0 < value < float('inf'):
                raise ValueError(f'{name} must be finite and positive.')
        for name in ('arm_group', 'gripper_group', 'move_action'):
            if not self.get_parameter(name).value.strip():
                raise ValueError(f'{name} must not be empty.')
        self._move_client = ActionClient(
            self, MoveGroup, self.get_parameter('move_action').value)
        if not self._move_client.wait_for_server(
                timeout_sec=self.get_parameter('server_timeout').value):
            raise RuntimeError('MoveIt MoveGroup action is unavailable. '
                               'Start move_group and check move_action.')

    def _wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('MoveIt request timed out or ROS shut down.')
        return future.result()

    def cancel_motion(self):
        """Request cancellation of an accepted goal before exiting."""
        if self._active_goal is not None and rclpy.ok():
            future = self._active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if not future.done() or not future.result().goals_canceling:
                self.get_logger().warning('MoveIt did not confirm cancellation.')
            self._active_goal = None

    def _move(self, group, joint_names, positions):
        if self._move_client is None:
            raise RuntimeError('Call configure() before requesting motion.')
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = group
        request.num_planning_attempts = 5
        request.allowed_planning_time = self.get_parameter('planning_time').value
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        # Empty diff asks MoveIt to use its monitored current robot state.
        request.start_state.is_diff = True
        constraints = Constraints()
        for name, position in zip(joint_names, positions):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(position),
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        # Wait for acceptance before awaiting execution so cancellation can use
        # the accepted goal handle. No second plan/go request is necessary.
        self._active_goal = self._wait(
            self._move_client.send_goal_async(goal), timeout=None)
        if not self._active_goal.accepted:
            self._active_goal = None
            raise RuntimeError(f'MoveIt rejected the {group} goal.')
        result = self._wait(
            self._active_goal.get_result_async(),
            self.get_parameter('execution_timeout').value)
        self._active_goal = None
        if (result.status != GoalStatus.STATUS_SUCCEEDED
                or result.result.error_code.val != MoveItErrorCodes.SUCCESS):
            raise RuntimeError(
                f'{group} motion failed: action status {result.status}, '
                f'MoveIt error {result.result.error_code.val}.')

    def _move_arm(self, positions):
        self._move(self.get_parameter('arm_group').value,
                   self.arm_joint_names, positions)

    def init_pose(self):
        self._move_arm([0.0, 0.0, 0.0, 0.0])
        self.get_logger().info('Manipulator pose initialized')

    def open_gripper(self):
        self._move(self.get_parameter('gripper_group').value,
                   [self.gripper_joint_name], [0.019])
        self.get_logger().info('Gripper opened')

    def close_gripper(self):
        self._move(self.get_parameter('gripper_group').value,
                   [self.gripper_joint_name], [-0.01])
        self.get_logger().info('Gripper closed')

    def reach_front(self):
        self._move_arm([0.0, 0.9, 0.0, -0.7])

    def reach_back(self):
        self._move_arm([0.0, -1.0, 0.8, 0.0])

    def _pause(self, seconds):
        # Honor use_sim_time, continuing to process /clock during pauses.
        deadline = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rclpy.ok():
            raise RuntimeError('ROS shut down during the procedure.')

    def pick_up(self):
        self.open_gripper()
        self._pause(1)
        self.reach_front()
        self._pause(3)
        self.close_gripper()
        self._pause(1)
        self.init_pose()
        self._pause(2)
        self.get_logger().info('Pick-up procedure completed.')

    def drop_off(self):
        self.init_pose()
        self._pause(1)
        self.reach_front()
        self._pause(3)
        self.open_gripper()
        self._pause(1)
        self.init_pose()
        self._pause(3)
        self.close_gripper()
        self._pause(2)
        self.get_logger().info('Drop-off procedure completed.')


# Preserve the original class name for callers importing this script.
manipulator_controller = ManipulatorController


def main(args=None):
    rclpy.init(args=args)
    controller = None
    exit_code = 0
    try:
        controller = ManipulatorController()
        procedure = controller.get_parameter('procedure').value
        if procedure not in ('pick_up', 'drop_off', 'init_pose', 'reach_front',
                             'reach_back', 'open_gripper', 'close_gripper'):
            raise ValueError(f'Unknown procedure: {procedure}')
        controller.configure()
        controller.init_pose()
        if procedure != 'init_pose':
            getattr(controller, procedure)()
    except KeyboardInterrupt:
        exit_code = 130
    except Exception as exc:
        if controller is not None:
            controller.get_logger().error(str(exc))
        else:
            print(str(exc), file=sys.stderr)
        exit_code = 1
    finally:
        if controller is not None:
            try:
                controller.cancel_motion()
            finally:
                controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
