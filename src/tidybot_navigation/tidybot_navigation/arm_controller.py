#!/usr/bin/env python3
"""
TidyBot Arm Controller
======================
Controls the 3-DOF right arm via the joint_pose_trajectory Gazebo plugin
and the vacuum gripper for pick-and-place.

Publishes JointTrajectory messages to /set_joint_trajectory with
frame_id='base_footprint' (the root link after fixed-joint merging).

Joint origin pre-rotations (URDF): shoulder -0.5 rad, elbow -2.5 rad about Y.
Effective angle = joint_value + offset, so:
  - Negative shoulder values swing the arm forward
  - Elbow ~2.5 unfolds the forearm from its tucked position

Arm poses (joint angles in rad — shoulder / elbow / wrist):
  REST  :  0.0 /  0.0 / 0.0   — arm tucked against torso
  REACH : -0.2 /  2.5 / 0.3   — arm extends forward and down, gripper near ground
  CARRY :  0.2 /  0.3 / 0.0   — arm tucked up, carrying an object
  DROP  :  0.0 /  1.5 / 0.0   — arm extended forward toward box opening
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    'shoulder_right_joint',
    'elbow_right_joint',
    'wrist_right_joint',
]

# (shoulder, elbow, wrist) in radians
POSES = {
    'REST' : [0.0,   0.0,  0.0],
    'REACH': [-0.2,  2.5,  0.3],
    'CARRY': [0.2,   0.3,  0.0],
    'DROP' : [0.0,   1.5,  0.0],
}

FRAME_ID = 'base_footprint'
MOVE_DURATION_SEC = 2  # seconds for arm to reach target


class ArmController(Node):

    def __init__(self):
        super().__init__('tidybot_arm_controller')

        self.current_pose = 'REST'

        # Trajectory publisher for arm position control
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/set_joint_trajectory', 10)

        # Vacuum gripper switch service
        self.vacuum_cli = self.create_client(
            SetBool, '/vacuum/switch')

        # Command listener
        self.log_sub = self.create_subscription(
            String, 'task_log', self._log_cb, 10)

        # Send REST pose after a short delay so the trajectory plugin is ready
        # This actively holds the arm in the tucked position from the start,
        # preventing gravity from pulling it down before any command is sent.
        self._startup_timer = self.create_timer(1.0, self._startup_hold)

        self.get_logger().info('Arm controller ready (trajectory mode)')

    def _startup_hold(self):
        """Send REST pose once on startup to engage the PID hold."""
        self._startup_timer.cancel()
        self.get_logger().info('Sending initial REST hold trajectory')
        self._set_pose('REST')

    def _log_cb(self, msg: String):
        text = msg.data
        if text.startswith('PICK:'):
            self.get_logger().info('Arm -> REACH (picking)')
            self._set_pose('REACH')
        elif text == 'ARM:CARRY':
            self.get_logger().info('Arm -> CARRY')
            self._set_pose('CARRY')
        elif text == 'ARM:GRAB':
            self.get_logger().info('Vacuum gripper ON')
            self._set_vacuum(True)
        elif text == 'ARM:RELEASE':
            self.get_logger().info('Vacuum gripper OFF')
            self._set_vacuum(False)
        elif text == 'DROP_ALL':
            self.get_logger().info('Arm -> DROP')
            self._set_pose('DROP')
        elif text == 'ARM:REST':
            self.get_logger().info('Arm -> REST')
            self._set_pose('REST')

    def _set_pose(self, pose_name: str):
        self.current_pose = pose_name
        angles = POSES[pose_name]

        traj = JointTrajectory()
        traj.header = Header()
        traj.header.frame_id = FRAME_ID
        traj.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(a) for a in angles]
        point.time_from_start = Duration(sec=MOVE_DURATION_SEC, nanosec=0)
        traj.points = [point]

        self.traj_pub.publish(traj)
        self.get_logger().info(f'Sent trajectory: {pose_name} {angles}')

    def _set_vacuum(self, on: bool):
        """Call the vacuum gripper switch service."""
        if not self.vacuum_cli.service_is_ready():
            self.get_logger().warn('Vacuum switch service not ready!')
            return
        req = SetBool.Request()
        req.data = on
        future = self.vacuum_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Vacuum {"ON" if on else "OFF"}: {f.result()}'))


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
