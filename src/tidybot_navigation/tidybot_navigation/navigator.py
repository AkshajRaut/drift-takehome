#!/usr/bin/env python3
"""
TidyBot Navigator — autonomous two-room exploration
=====================================================
Hard-coded collision-free waypoints covering both rooms including
north corridors. No obstacle avoidance — pure waypoint following.
Logs odometry stats throughout.
"""

import math
import os
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


TOUR_WPS = [
    # === Living Room clockwise sweep ===
    (1.0,  0.0),    # start
    (0.5,  0.0),    # west
    (0.5, -1.0),    # SW direction
    (0.5, -2.5),    # SW corner
    (3.0, -2.5),    # south centre
    (5.5, -2.5),    # SE corner
    (5.5, -0.5),    # up east wall
    # North: table/couch gap then above shelf
    (3.5,  1.4),    # north gap east
    (3.5,  2.2),    # above shelf line
    (5.0,  2.2),    # sweep above shelf to east
    (3.5,  2.2),    # back
    (3.5,  1.4),    # down to gap
    (1.3,  1.4),    # north gap west
    (0.5,  1.0),    # NW (below collection box)
    # === Transit to Bedroom ===
    (0.5,  0.0),    # south along west wall
    (2.0,  0.0),    # east (above cube3 zone)
    (2.0, -0.5),    # drop below table
    (5.5, -0.5),    # east lane
    (5.5,  0.0),    # doorway approach
    (6.5,  0.0),    # through doorway centre
    # === Bedroom south sweep ===
    (6.5, -1.0),    # south (below cube5/chair_r2)
    (8.0, -1.0),    # mid
    (8.5, -0.5),    # east of desk zone
    (8.5, -2.5),    # south
    (10.5, -2.5),   # SE corner
    (10.5, -1.5),   # east wall
    (8.5, -1.5),    # back west (below wardrobe)
    # === Bedroom north sweep ===
    (7.8,  0.0),    # bedroom centre (west of bed)
    (7.5,  2.5),    # NW bedroom (above nightstand & bed)
    (10.5,  2.5),   # NE bedroom (closer to east wall)
    (7.5,  2.5),    # back west
    (7.8,  0.0),    # back down
    # === Return to Living Room ===
    (7.8, -1.0),    # south
    (6.5, -1.0),    # west
    (6.5,  0.0),    # up to doorway
    (5.5,  0.0),    # through doorway
    (5.5, -0.5),    # south (table blocks y=0 transit)
    (2.0, -0.5),    # west
    (2.0,  0.0),    # up
    (1.0,  0.0),    # home
]

WP_RADIUS     = 0.40
GOAL_RADIUS   = 0.25
MAX_LIN       = 0.30
MIN_LIN       = 0.06
ANG_KP        = 1.8
ANG_KD        = 0.4
MAX_ANG       = 1.2
DECEL_DIST    = 0.6
LOG_INTERVAL  = 2.0


def _yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _room_name(x):
    return 'living_room' if x < 6.1 else 'bedroom'


class Phase(Enum):
    WAIT    = auto()
    EXPLORE = auto()
    DONE    = auto()


class TidybotNavigator(Node):

    def __init__(self):
        super().__init__('tidybot_navigator')

        self.px, self.py, self.yaw_ = 0.0, 0.0, 0.0
        self.odom_ok = False
        self.scan_ok = False

        self.phase = Phase.WAIT
        self.wp_i = 0

        self.dist = 0.0
        self._pp = None
        self.rooms = set()
        self.wps_reached = 0
        self.t_start = 0.0
        self.t_last_log = 0.0
        self.path_log = []

        self._herr_prev = 0.0

        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.logp = self.create_publisher(String, 'task_log', 10)
        self.create_subscription(Odometry, 'odom', self._odom, 10)
        self.create_subscription(LaserScan, 'scan', self._scan, 10)
        self.create_timer(0.05, self._tick)  # 20 Hz

        self.get_logger().info('Navigator started — waiting for sensors')

    def _odom(self, m):
        self.px = m.pose.pose.position.x
        self.py = m.pose.pose.position.y
        self.yaw_ = _yaw(m.pose.pose.orientation)
        self.odom_ok = True
        if self._pp:
            self.dist += math.hypot(self.px - self._pp[0], self.py - self._pp[1])
        self._pp = (self.px, self.py)
        self.rooms.add(_room_name(self.px))

    def _scan(self, m):
        self.scan_ok = True

    def _drive(self, gx, gy, final):
        dx, dy = gx - self.px, gy - self.py
        d = math.hypot(dx, dy)

        r = GOAL_RADIUS if final else WP_RADIUS
        if d < r:
            if final:
                self._stop()
            self._herr_prev = 0.0
            return True

        desired = math.atan2(dy, dx)
        herr = _wrap(desired - self.yaw_)
        derr = herr - self._herr_prev
        self._herr_prev = herr

        ang = ANG_KP * herr + ANG_KD * derr
        ang = max(-MAX_ANG, min(MAX_ANG, ang))

        align = max(0.0, math.cos(min(abs(herr), math.pi / 2)))
        dfactor = min(1.0, d / DECEL_DIST) if final else 1.0

        spd = MAX_LIN * align * dfactor
        if abs(herr) > math.radians(55):
            spd = 0.0
        else:
            spd = max(MIN_LIN, spd)

        t = Twist()
        t.linear.x = float(spd)
        t.angular.z = float(ang)
        self.cmd.publish(t)
        return False

    def _stop(self):
        self.cmd.publish(Twist())
        self._herr_prev = 0.0

    def _log_stats(self, event=''):
        elapsed = time.time() - self.t_start
        room = _room_name(self.px)
        msg = (f'[NAV] {event:20s} | t={elapsed:6.1f}s | '
               f'pos=({self.px:5.2f},{self.py:5.2f}) | '
               f'dist={self.dist:6.2f}m | '
               f'wps={self.wps_reached}/{len(TOUR_WPS)} | '
               f'room={room:12s} | '
               f'rooms_visited={sorted(self.rooms)}')
        self.get_logger().info(msg)
        self.path_log.append((elapsed, self.px, self.py, self.dist))
        s = String()
        s.data = msg
        self.logp.publish(s)

    def _print_summary(self):
        elapsed = time.time() - self.t_start
        self.get_logger().info('=' * 65)
        self.get_logger().info('  EXPLORATION COMPLETE')
        self.get_logger().info('=' * 65)
        self.get_logger().info(f'  Total time:        {elapsed:.1f} s')
        self.get_logger().info(f'  Total distance:    {self.dist:.2f} m')
        self.get_logger().info(f'  Waypoints reached: {self.wps_reached}/{len(TOUR_WPS)}')
        self.get_logger().info(f'  Rooms visited:     {sorted(self.rooms)}')
        self.get_logger().info(f'  Final position:    ({self.px:.2f}, {self.py:.2f})')
        self.get_logger().info('-' * 65)
        self.get_logger().info('  PATH LOG:')
        self.get_logger().info(f'  {"time":>8s}  {"x":>7s}  {"y":>7s}  {"dist":>8s}')
        for t, x, y, d in self.path_log:
            self.get_logger().info(f'  {t:8.1f}  {x:7.2f}  {y:7.2f}  {d:8.2f}')
        self.get_logger().info('=' * 65)
        self._save_path_map()

    def _save_path_map(self):
        """Auto-save exploration path map as PNG."""
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches
        except ImportError:
            self.get_logger().warn('matplotlib not installed — skipping path map')
            return

        xs = [p[1] for p in self.path_log]
        ys = [p[2] for p in self.path_log]

        fig, ax = plt.subplots(1, 1, figsize=(16, 8))
        ax.set_aspect('equal')
        ax.set_xlim(-0.5, 12.0)
        ax.set_ylim(-3.8, 3.8)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('TidyBot Exploration Path')
        ax.grid(True, alpha=0.2)

        # Walls
        wc = '#888888'
        ax.plot([-0.1, -0.1], [-3.1, 3.1], color=wc, lw=3)
        ax.plot([-0.1, 6.1], [3.1, 3.1], color=wc, lw=3)
        ax.plot([-0.1, 6.1], [-3.1, -3.1], color=wc, lw=3)
        ax.plot([6.1, 6.1], [-3.1, -0.8], color=wc, lw=3)
        ax.plot([6.1, 6.1], [0.8, 3.1], color=wc, lw=3)
        ax.plot([11.3, 11.3], [-3.1, 3.1], color=wc, lw=3)
        ax.plot([6.1, 11.3], [3.1, 3.1], color=wc, lw=3)
        ax.plot([6.1, 11.3], [-3.1, -3.1], color=wc, lw=3)

        # Furniture
        furniture = [
            ('Table', 3.0, 0.5, 1.2, 0.8, '#8B6914'),
            ('Chair A', 2.1, -1.5, 0.45, 0.45, '#6B4226'),
            ('Chair B', 3.9, -1.5, 0.45, 0.45, '#6B4226'),
            ('Couch', 2.0, 2.35, 1.8, 0.7, '#4444AA'),
            ('Shelf', 4.8, 1.5, 0.8, 0.4, '#8B5A2B'),
            ('Bed', 9.5, 1.1, 2.0, 1.5, '#C4A882'),
            ('Nightstand', 6.9, 1.7, 0.5, 0.5, '#8B5A2B'),
            ('Wardrobe', 9.6, -0.6, 1.0, 0.6, '#8B5A2B'),
            ('Desk', 7.5, -2.0, 1.0, 0.6, '#8B6914'),
            ('Chair R2', 7.0, 0.5, 0.45, 0.45, '#6B4226'),
        ]
        for name, cx, cy, w, h, color in furniture:
            rect = patches.Rectangle((cx - w / 2, cy - h / 2), w, h,
                                      lw=1, edgecolor='black',
                                      facecolor=color, alpha=0.6)
            ax.add_patch(rect)
            ax.text(cx, cy, name, ha='center', va='center',
                    fontsize=6, color='white', fontweight='bold')

        # Collection box
        bx = patches.Rectangle((0.175, 1.675), 0.65, 0.65, lw=1.5,
                                 edgecolor='green', facecolor='#22AA22', alpha=0.4)
        ax.add_patch(bx)
        ax.text(0.5, 2.0, 'Box', ha='center', va='center',
                fontsize=7, color='darkgreen', fontweight='bold')

        # Waypoints
        wp_x = [w[0] for w in TOUR_WPS]
        wp_y = [w[1] for w in TOUR_WPS]
        ax.plot(wp_x, wp_y, '--', color='#AAAAFF', lw=1, alpha=0.5,
                label='Planned waypoints')

        # Actual path
        ax.plot(xs, ys, '-', color='#FF4444', lw=2, alpha=0.8,
                label='Actual path')
        ax.plot(xs[0], ys[0], '*', color='lime', markersize=15,
                markeredgecolor='black', zorder=10, label='Start')
        ax.plot(xs[-1], ys[-1], 'X', color='red', markersize=12,
                markeredgecolor='black', zorder=10, label='End')

        ax.text(3.0, -3.5, 'Living Room', ha='center', fontsize=12,
                color='#555555')
        ax.text(8.7, -3.5, 'Bedroom', ha='center', fontsize=12,
                color='#555555')
        ax.legend(loc='upper left', fontsize=8)
        plt.tight_layout()

        out = os.path.expanduser('~/exploration_path.png')
        plt.savefig(out, dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Path map saved: {out}')

    def _tick(self):
        if self.phase == Phase.WAIT:
            if self.odom_ok and self.scan_ok:
                self.get_logger().info('Sensors online — starting exploration')
                self.t_start = time.time()
                self.t_last_log = self.t_start
                self.phase = Phase.EXPLORE
                self._log_stats('START')
            return

        if self.phase == Phase.DONE:
            return

        if self.phase == Phase.EXPLORE:
            now = time.time()
            if now - self.t_last_log >= LOG_INTERVAL:
                self._log_stats('periodic')
                self.t_last_log = now

            if self.wp_i >= len(TOUR_WPS):
                self._stop()
                self.phase = Phase.DONE
                self._log_stats('FINISH')
                self._print_summary()
                return

            gx, gy = TOUR_WPS[self.wp_i]
            final = (self.wp_i == len(TOUR_WPS) - 1)
            if self._drive(gx, gy, final):
                self.wps_reached += 1
                self.wp_i += 1
                self._log_stats(f'wp{self.wps_reached}({gx},{gy})')


def main(args=None):
    rclpy.init(args=args)
    node = TidybotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
