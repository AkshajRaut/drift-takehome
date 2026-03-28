#!/usr/bin/env python3
"""
Magnetic Gripper Node
=====================
Simulates a magnetic gripper using continuous position-locking.

When activated, finds the nearest pickup object and locks its position
to the hand link every tick using Gazebo's set_entity_state service.
The object stays in the world the entire time — no delete/respawn.
On release, the position lock stops and the object drops with gravity.

Uses /gazebo/model_states for robot + object world positions,
and /joint_states for forward kinematics to compute hand position.

Services:
  /gripper/attach  (std_srvs/SetBool)  — data=true: grab nearest, data=false: release

Topics published:
  /gripper/holding (std_msgs/String)    — name of held object, or ""
"""

import math
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates, EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

PICKUP_OBJECTS = [
    'cube1', 'cube2', 'cube3',
    'cube4', 'cube5', 'cube6',
]

ATTACH_RANGE = 1.0
LOCK_RATE_HZ = 30.0

# Robot geometry (from URDF)
WHEEL_RADIUS = 0.10
BASE_HEIGHT = 0.15
TORSO_HEIGHT = 0.55
UA_LEN = 0.25
FA_LEN = 0.22
HAND_LEN = 0.12
SHOULDER_OFFSET_Y = -0.19
SHOULDER_OFFSET_X = 0.02
SHOULDER_OFFSET_Z = TORSO_HEIGHT * 0.65
SHOULDER_PRE = -0.5
ELBOW_PRE = -2.5


class GripperNode(Node):

    def __init__(self):
        super().__init__('gripper_node')

        self.held_object = None
        self._obj_positions = {}
        self._robot_pos = None
        self._robot_yaw = 0.0
        self._joints = {}

        self._set_state = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')

        self.create_subscription(
            ModelStates, '/gazebo/model_states', self._model_cb, 1)
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 1)

        self.create_service(SetBool, '/gripper/attach', self._attach_cb)

        self.holding_pub = self.create_publisher(String, '/gripper/holding', 10)
        self.create_timer(0.1, self._pub_state)
        self._lock_timer = self.create_timer(1.0 / LOCK_RATE_HZ, self._lock_tick)

        self.get_logger().info('Magnetic gripper ready')

    def _model_cb(self, msg: ModelStates):
        for i, name in enumerate(msg.name):
            p = msg.pose[i].position
            q = msg.pose[i].orientation
            if name in PICKUP_OBJECTS:
                self._obj_positions[name] = (p.x, p.y, p.z)
            elif name == 'tidybot':
                self._robot_pos = (p.x, p.y, p.z)
                self._robot_yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            self._joints[name] = msg.position[i]

    def _get_hand_world_pos(self):
        if self._robot_pos is None:
            return None

        rx, ry, rz = self._robot_pos
        yaw = self._robot_yaw

        s = self._joints.get('shoulder_right_joint', 0.0)
        e = self._joints.get('elbow_right_joint', 0.0)
        w = self._joints.get('wrist_right_joint', 0.0)

        sa = s + SHOULDER_PRE
        ea = sa + (e + ELBOW_PRE)
        wa = ea + w

        mx = SHOULDER_OFFSET_X
        mz = WHEEL_RADIUS + BASE_HEIGHT / 2.0 + SHOULDER_OFFSET_Z

        tip_x = mx - UA_LEN * math.sin(sa)
        tip_z = mz - UA_LEN * math.cos(sa)
        tip_x += -FA_LEN * math.sin(ea)
        tip_z += -FA_LEN * math.cos(ea)
        tip_x += -HAND_LEN * math.sin(wa)
        tip_z += -HAND_LEN * math.cos(wa)

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        world_x = rx + cos_y * tip_x - sin_y * SHOULDER_OFFSET_Y
        world_y = ry + sin_y * tip_x + cos_y * SHOULDER_OFFSET_Y
        world_z = rz + tip_z

        return (world_x, world_y, world_z)

    def _pub_state(self):
        msg = String()
        msg.data = self.held_object or ''
        self.holding_pub.publish(msg)

    def _attach_cb(self, request, response):
        if request.data:
            hand = self._get_hand_world_pos()
            if hand is None:
                response.success = False
                response.message = 'Position data not ready'
                return response

            hx, hy, hz = hand
            self.get_logger().info(f'Hand at ({hx:.3f}, {hy:.3f}, {hz:.3f})')

            best_name, best_dist = None, ATTACH_RANGE
            for name, pos in self._obj_positions.items():
                d = math.sqrt((pos[0] - hx)**2 +
                              (pos[1] - hy)**2 +
                              (pos[2] - hz)**2)
                if d < best_dist:
                    best_dist = d
                    best_name = name

            if best_name:
                self.held_object = best_name
                response.success = True
                response.message = f'Grabbed {best_name} (d={best_dist:.2f}m)'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f'No object within {ATTACH_RANGE}m'
                self.get_logger().warn(response.message)
        else:
            if self.held_object:
                self.get_logger().info(f'Released {self.held_object}')
                self.held_object = None
            response.success = True
            response.message = 'Released'
        return response

    def _lock_tick(self):
        if self.held_object is None or not self._set_state.service_is_ready():
            return
        if self._robot_pos is None:
            return

        rx, ry, rz = self._robot_pos
        yaw = self._robot_yaw

        # Place cube 0.5m in front of robot, at ground level + small offset
        # This keeps it visible and outside the collision cylinder (r=0.35m)
        cx = rx + 0.50 * math.cos(yaw)
        cy = ry + 0.50 * math.sin(yaw)
        cz = 0.30  # on top of base, visible but outside collision

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.held_object
        req.state.pose = Pose(
            position=Point(x=cx, y=cy, z=cz),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        req.state.twist = Twist()
        req.state.reference_frame = ''
        self._set_state.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
