#!/usr/bin/env python3
"""Plot the robot's exploration path over the home floor plan."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# ── Read path data ──
xs, ys = [], []
with open('/tmp/path_data.csv') as f:
    for line in f:
        parts = line.strip().split(',')
        xs.append(float(parts[0]))
        ys.append(float(parts[1]))

# ── Waypoints ──
wps = [
    (1.0,0.0),(0.5,-1.0),(0.5,-2.5),(3.0,-2.5),(5.5,-2.5),
    (5.5,-0.5),(5.5,0.8),
    (3.8,0.8),(3.8,2.5),(5.5,2.5),(3.8,2.5),(3.8,0.8),(3.8,-0.5),
    (5.5,-0.5),(5.5,0.0),(6.5,0.0),
    (7.5,-1.0),(8.0,-0.5),(8.5,-0.5),(8.5,-2.5),(10.5,-2.5),
    (10.5,-1.5),(8.5,-1.5),
    (7.8,0.0),(7.5,2.5),(10.5,2.5),(7.5,2.5),(7.8,-0.5),
    (6.5,0.0),(5.5,0.0),
    (5.5,-0.5),(3.0,-0.5),(1.0,-0.5),(1.0,0.0),
]

fig, ax = plt.subplots(1, 1, figsize=(16, 8))
ax.set_aspect('equal')
ax.set_xlim(-0.5, 12.0)
ax.set_ylim(-3.8, 3.8)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('TidyBot Exploration Path — Both Rooms')
ax.grid(True, alpha=0.2)

# ── Walls ──
wall_color = '#888888'
lw = 3
# Room 1 outer walls
ax.plot([-0.1, -0.1], [-3.1, 3.1], color=wall_color, lw=lw)  # west
ax.plot([-0.1, 6.1], [3.1, 3.1], color=wall_color, lw=lw)    # north r1
ax.plot([-0.1, 6.1], [-3.1, -3.1], color=wall_color, lw=lw)  # south r1
# Shared wall with doorway
ax.plot([6.1, 6.1], [-3.1, -0.8], color=wall_color, lw=lw)   # south segment
ax.plot([6.1, 6.1], [0.8, 3.1], color=wall_color, lw=lw)     # north segment
# Room 2 outer walls
ax.plot([11.3, 11.3], [-3.1, 3.1], color=wall_color, lw=lw)  # east
ax.plot([6.1, 11.3], [3.1, 3.1], color=wall_color, lw=lw)    # north r2
ax.plot([6.1, 11.3], [-3.1, -3.1], color=wall_color, lw=lw)  # south r2

# ── Furniture (as rectangles) ──
furniture = [
    ('Table',      2.5,  1.0,  1.2, 0.8,  '#8B6914'),
    ('Chair A',    2.1, -1.5,  0.45,0.45, '#6B4226'),
    ('Chair B',    3.9, -1.5,  0.45,0.45, '#6B4226'),
    ('Couch',      2.0,  2.35, 1.8, 0.7,  '#4444AA'),
    ('Shelf',      4.8,  1.5,  0.8, 0.4,  '#8B5A2B'),
    ('Bed',        9.5,  1.1,  2.0, 1.5,  '#C4A882'),
    ('Nightstand', 6.9,  1.7,  0.5, 0.5,  '#8B5A2B'),
    ('Wardrobe',   9.6, -0.6,  1.0, 0.6,  '#8B5A2B'),
    ('Desk',       7.5, -2.0,  1.0, 0.6,  '#8B6914'),
    ('Chair R2',   7.0,  0.5,  0.45,0.45, '#6B4226'),
]
for name, cx, cy, w, h, color in furniture:
    rect = patches.Rectangle((cx-w/2, cy-h/2), w, h,
                               linewidth=1, edgecolor='black',
                               facecolor=color, alpha=0.6)
    ax.add_patch(rect)
    ax.text(cx, cy, name, ha='center', va='center', fontsize=6, color='white', fontweight='bold')

# ── Collection box ──
box_rect = patches.Rectangle((0.175, 1.675), 0.65, 0.65,
                               linewidth=1.5, edgecolor='green',
                               facecolor='#22AA22', alpha=0.4)
ax.add_patch(box_rect)
ax.text(0.5, 2.0, 'Box', ha='center', va='center', fontsize=7, color='darkgreen', fontweight='bold')

# ── Cubes ──
cubes = [
    ('C1', 3.0, -2.0, 'red'),
    ('C2', 8.0,  2.0, 'red'),
    ('C3', 1.5, -0.5, 'blue'),
    ('C4', 4.0,  0.5, 'gold'),
    ('C5', 7.0, -0.5, 'green'),
    ('C6', 9.0, -2.0, 'orange'),
]
for name, cx, cy, color in cubes:
    ax.plot(cx, cy, 's', color=color, markersize=8, markeredgecolor='black', markeredgewidth=0.5)
    ax.text(cx+0.15, cy+0.15, name, fontsize=6, color=color)

# ── Waypoints (planned path) ──
wp_x = [w[0] for w in wps]
wp_y = [w[1] for w in wps]
ax.plot(wp_x, wp_y, '--', color='#AAAAFF', lw=1, alpha=0.5, label='Planned waypoints')
ax.plot(wp_x, wp_y, 'o', color='#6666CC', markersize=3, alpha=0.5)

# ── Actual robot path ──
ax.plot(xs, ys, '-', color='#FF4444', lw=2, alpha=0.8, label='Actual path')
ax.plot(xs[0], ys[0], '*', color='lime', markersize=15, markeredgecolor='black', zorder=10, label='Start')
ax.plot(xs[-1], ys[-1], 'X', color='red', markersize=12, markeredgecolor='black', zorder=10, label='End')

# ── Room labels ──
ax.text(3.0, -3.5, 'Living Room', ha='center', fontsize=12, color='#555555')
ax.text(8.7, -3.5, 'Bedroom', ha='center', fontsize=12, color='#555555')

# ── Doorway label ──
ax.annotate('Doorway', xy=(6.1, 0), fontsize=8, color='#777777',
            ha='center', va='bottom', xytext=(6.1, 0.3))

ax.legend(loc='upper left', fontsize=8)
plt.tight_layout()
plt.savefig('/home/akshaj/drift-takehome/exploration_path.png', dpi=150)
print('Saved: /home/akshaj/drift-takehome/exploration_path.png')
