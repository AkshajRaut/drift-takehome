#!/usr/bin/env python3
"""
TidyBot LiDAR Mapper — real-time occupancy grid mapping.

Subscribes to /scan (LaserScan) and /odom (Odometry), builds a 2-D
occupancy grid via Bresenham ray tracing, publishes nav_msgs/OccupancyGrid
on /map, and saves a PNG visualization when exploration finishes.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# --- Grid parameters ---------------------------------------------------
# Grid covers a generous area; the LiDAR discovers what's in it.
RESOLUTION = 0.05          # metres per cell
ORIGIN_X = -2.0            # world x of grid origin (bottom-left)
ORIGIN_Y = -5.0            # world y of grid origin
WIDTH = 16.0               # metres
HEIGHT = 11.0              # metres
GRID_W = int(WIDTH / RESOLUTION)
GRID_H = int(HEIGHT / RESOLUTION)

# Log-odds constants
L_FREE = -0.3              # ray passes through
L_OCC = 0.9                # ray endpoint (strong occupied signal)
L_PRIOR = 0.0
L_MIN = -5.0
L_MAX = 5.0

# Publish rate
MAP_PUBLISH_HZ = 2.0


def _yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _bresenham(x0, y0, x1, y1):
    """Yield (col, row) cells along the line from (x0,y0) to (x1,y1)."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


class TidybotMapper(Node):

    def __init__(self):
        super().__init__('tidybot_mapper')

        # Log-odds grid
        self.grid = np.full((GRID_H, GRID_W), L_PRIOR, dtype=np.float32)

        # Robot pose
        self.px = 0.0
        self.py = 0.0
        self.yaw = 0.0
        self.odom_ok = False

        self._done = False

        # Subscriptions
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_subscription(String, 'task_log', self._log_cb, 10)

        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)

        # Timer for publishing map
        self.create_timer(1.0 / MAP_PUBLISH_HZ, self._publish_map)

        self.get_logger().info(
            f'Mapper started — grid {GRID_W}x{GRID_H} '
            f'({WIDTH}x{HEIGHT}m @ {RESOLUTION}m/cell)')

    # -- helpers --------------------------------------------------------

    def _world_to_grid(self, wx, wy):
        col = int((wx - ORIGIN_X) / RESOLUTION)
        row = int((wy - ORIGIN_Y) / RESOLUTION)
        return col, row

    def _in_bounds(self, c, r):
        return 0 <= c < GRID_W and 0 <= r < GRID_H

    # -- callbacks ------------------------------------------------------

    def _odom_cb(self, msg):
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.yaw = _yaw(msg.pose.pose.orientation)
        self.odom_ok = True

    def _scan_cb(self, msg):
        if not self.odom_ok:
            return

        rx, ry, ryaw = self.px, self.py, self.yaw
        rc, rr = self._world_to_grid(rx, ry)

        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Endpoint in world
                beam_angle = ryaw + angle
                ex = rx + r * math.cos(beam_angle)
                ey = ry + r * math.sin(beam_angle)

                ec, er = self._world_to_grid(ex, ey)

                # Ray trace free cells
                for c, row in _bresenham(rc, rr, ec, er):
                    if not self._in_bounds(c, row):
                        continue
                    if c == ec and row == er:
                        break
                    self.grid[row, c] = np.clip(
                        self.grid[row, c] + L_FREE, L_MIN, L_MAX)

                # Mark endpoint occupied
                if self._in_bounds(ec, er):
                    self.grid[er, ec] = np.clip(
                        self.grid[er, ec] + L_OCC, L_MIN, L_MAX)

            angle += msg.angle_increment

    def _log_cb(self, msg):
        if 'FINISH' in msg.data and not self._done:
            self._done = True
            self.get_logger().info('Exploration finished — saving map PNG')
            self._save_map_png()

    # -- publishing -----------------------------------------------------

    def _publish_map(self):
        og = OccupancyGrid()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'odom'
        og.info.resolution = RESOLUTION
        og.info.width = GRID_W
        og.info.height = GRID_H
        og.info.origin.position.x = ORIGIN_X
        og.info.origin.position.y = ORIGIN_Y

        # Convert log-odds to probability [0..100], -1 = unknown
        prob = np.full(self.grid.shape, -1, dtype=np.int8)
        known = np.abs(self.grid) > 0.1
        p = 1.0 / (1.0 + np.exp(-self.grid[known]))
        prob[known] = (p * 100).astype(np.int8)
        og.data = prob.ravel().tolist()
        self.map_pub.publish(og)

    # -- visualisation --------------------------------------------------

    def _save_map_png(self):
        """Save the occupancy grid as a PNG using the standard ROS map
        colour scheme (matching nav2 map_saver output):
        unknown = grey (205), free = white (254), occupied = black (0),
        with linear interpolation for intermediate probabilities.
        """
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        out_dir = os.path.join(os.getcwd(), 'output')
        os.makedirs(out_dir, exist_ok=True)
        out = os.path.join(out_dir, 'lidar_map.png')

        try:
            prob = 1.0 / (1.0 + np.exp(-self.grid))
            known = np.abs(self.grid) > 0.1

            # nav2 map_saver colour mapping:
            # unknown → 205/255 ≈ 0.804 (grey)
            # free (prob~0) → 254/255 ≈ 1.0 (white)
            # occupied (prob~1) → 0/255 = 0.0 (black)
            img = np.full(self.grid.shape, 205.0 / 255.0, dtype=np.float32)
            img[known] = 1.0 - prob[known]  # free→white, occupied→black

            # Crop to explored region
            rows, cols = np.where(known)
            if len(rows) == 0:
                self.get_logger().warn('No explored cells — skip PNG')
                return
            pad = 20
            r_min = max(rows.min() - pad, 0)
            r_max = min(rows.max() + pad, GRID_H)
            c_min = max(cols.min() - pad, 0)
            c_max = min(cols.max() + pad, GRID_W)
            cropped = img[r_min:r_max, c_min:c_max]

            fig, ax = plt.subplots(1, 1, figsize=(12, 9))
            ax.imshow(cropped, cmap='gray', vmin=0.0, vmax=1.0,
                      origin='lower', interpolation='nearest')
            ax.set_title('LiDAR Occupancy Grid Map')
            ax.set_xlabel(f'cells ({RESOLUTION}m each)')
            ax.set_ylabel(f'cells ({RESOLUTION}m each)')
            ax.set_aspect('equal')
            fig.tight_layout()
            fig.savefig(out, dpi=150)
            plt.close(fig)
            self.get_logger().info(f'Occupancy map saved: {out}')

        except Exception as e:
            self.get_logger().warn(f'Map save failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TidybotMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._save_map_png()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
