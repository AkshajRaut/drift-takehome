#!/usr/bin/env python3
"""
Simple gripper node — attach/detach objects to the robot's hand.

Grab: deletes the object from the world (it disappears).
Release: respawns the object at the hand's current position.

No physics conflicts, no teleporting, no glitches.

Services:
  /gripper/attach  (std_srvs/SetBool)  — data=true: grab nearest, data=false: release

Topics:
  /gripper/holding (std_msgs/String)    — name of held object, or ""
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import SetBool
from std_msgs.msg import String
from gazebo_msgs.srv import GetEntityState, SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Point, Quaternion


PICKUP_OBJECTS = [
    'cube1', 'cube2', 'cube3',
    'cube4', 'cube5', 'cube6',
]

# SDF template for respawning a pickup object
OBJ_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
<model name="{name}">
  <link name="link">
    <inertial>
      <mass>0.2</mass>
      <inertia>
        <ixx>0.0005</ixx><ixy>0</ixy><ixz>0</ixz>
        <iyy>0.0005</iyy><iyz>0</iyz><izz>0.0005</izz>
      </inertia>
    </inertial>
    <collision name="col">
      <geometry><box><size>0.08 0.08 0.08</size></box></geometry>
      <surface>
        <friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction>
        <contact><ode><kp>100000</kp><kd>100</kd><min_depth>0.001</min_depth><max_vel>3.0</max_vel></ode></contact>
      </surface>
    </collision>
    <visual name="vis">
      <geometry><box><size>0.08 0.08 0.08</size></box></geometry>
      <material>
        <ambient>{color}</ambient>
        <diffuse>{color}</diffuse>
      </material>
    </visual>
  </link>
</model>
</sdf>"""

OBJ_COLORS = {
    'cube1': '0.9 0.1 0.1 1',
    'cube2': '0.9 0.1 0.1 1',
    'cube3': '0.1 0.2 0.9 1',
    'cube4': '0.95 0.85 0.1 1',
    'cube5': '0.1 0.8 0.2 1',
    'cube6': '1.0 0.5 0.0 1',
}

HAND_LINK = 'tidybot::hand_right'
ATTACH_RANGE = 1.5


class _GazeboHelper:
    """Owns its own node + executor for blocking Gazebo service calls."""

    def __init__(self):
        self._node = rclpy.create_node('_gripper_gz_helper')
        self._get = self._node.create_client(
            GetEntityState, '/gazebo/get_entity_state')
        self._spawn = self._node.create_client(
            SpawnEntity, '/spawn_entity')
        self._delete = self._node.create_client(
            DeleteEntity, '/delete_entity')
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)

    def get_pose(self, name):
        if not self._get.wait_for_service(timeout_sec=1.0):
            return None
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = 'world'
        future = self._get.call_async(req)
        self._exec.spin_until_future_complete(future, timeout_sec=2.0)
        if future.done() and future.result() and future.result().success:
            p = future.result().state.pose.position
            return (p.x, p.y, p.z)
        return None

    def delete_model(self, name):
        if not self._delete.wait_for_service(timeout_sec=1.0):
            return False
        req = DeleteEntity.Request()
        req.name = name
        future = self._delete.call_async(req)
        self._exec.spin_until_future_complete(future, timeout_sec=2.0)
        return future.done() and future.result() and future.result().success

    def spawn_model(self, name, x, y, z):
        if not self._spawn.wait_for_service(timeout_sec=1.0):
            return False
        color = OBJ_COLORS.get(name, '0.5 0.5 0.5 1')
        req = SpawnEntity.Request()
        req.name = name
        req.xml = OBJ_SDF.format(name=name, color=color)
        req.initial_pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        req.reference_frame = 'world'
        future = self._spawn.call_async(req)
        self._exec.spin_until_future_complete(future, timeout_sec=2.0)
        return future.done() and future.result() and future.result().success

    def destroy(self):
        self._node.destroy_node()


class GripperNode(Node):

    def __init__(self):
        super().__init__('gripper_node')

        self.held_object = None
        self._gz = _GazeboHelper()

        self.create_service(
            SetBool, '/gripper/attach', self._attach_cb)

        self.holding_pub = self.create_publisher(String, '/gripper/holding', 10)
        self.create_timer(0.1, self._pub_state)

        self.get_logger().info('Gripper node ready — call /gripper/attach')

    def _pub_state(self):
        msg = String()
        msg.data = self.held_object or ''
        self.holding_pub.publish(msg)

    def _attach_cb(self, request, response):
        if request.data:
            # --- GRAB ---
            hand = self._gz.get_pose(HAND_LINK)
            if hand is None:
                response.success = False
                response.message = 'Cannot read hand pose'
                return response

            best_name, best_dist = None, ATTACH_RANGE
            for obj in PICKUP_OBJECTS:
                p = self._gz.get_pose(obj)
                if p is None:
                    continue
                d = math.sqrt((p[0]-hand[0])**2 +
                              (p[1]-hand[1])**2 +
                              (p[2]-hand[2])**2)
                if d < best_dist:
                    best_dist = d
                    best_name = obj

            if best_name:
                # Delete the object from the world
                if self._gz.delete_model(best_name):
                    self.held_object = best_name
                    response.success = True
                    response.message = f'Grabbed {best_name} (d={best_dist:.2f}m)'
                    self.get_logger().info(response.message)
                else:
                    response.success = False
                    response.message = f'Failed to delete {best_name}'
            else:
                response.success = False
                response.message = f'No object within {ATTACH_RANGE}m'
                self.get_logger().warn(response.message)
        else:
            # --- RELEASE ---
            if self.held_object:
                hand = self._gz.get_pose(HAND_LINK)
                if hand:
                    # Spawn below hand position
                    drop_z = max(0.04, hand[2] - 0.15)
                    self._gz.spawn_model(
                        self.held_object, hand[0], hand[1], drop_z)
                    self.get_logger().info(
                        f'Released {self.held_object} at ({hand[0]:.2f}, {hand[1]:.2f}, {drop_z:.2f})')
                else:
                    self.get_logger().warn('Cannot read hand pose for drop')
                self.held_object = None
            response.success = True
            response.message = 'Released'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._gz.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
