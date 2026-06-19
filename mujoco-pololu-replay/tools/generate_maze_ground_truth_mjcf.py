#!/usr/bin/env python3
from pathlib import Path
import math
import argparse

GROUND_TRUTH_WALLS_M = [
    # Top-left start corridor
    ((-0.05,  0.1), (0.26,  0.1)),

    # Vertical drop after start corridor
    ((0.26,  0.1), (0.26, -0.035)),

    # Upper middle horizontal wall
    ((0.26, -0.035), (0.66, -0.035)),

    # Right vertical wall
    ((0.66, -0.035), (0.66, -0.15)),

    # Short right-side end wall
    ((0.66, -0.15), (1.0, -0.15)),

    # Left short horizontal wall below start
    ((-0.05, -0.035), (0.10, -0.035)),

    # Long left vertical wall
    ((0.10, -0.035), (0.10, -0.38)),

    # Bottom horizontal wall left side
    ((0.10, -0.38), (1.0, -0.38)),

    # Middle vertical divider near bottom
    ((0.49, -0.38), (0.49, -0.15)),
]

def wall_box_xml(name, p1, p2, thickness, height):
    x1, y1 = p1
    x2, y2 = p2

    cx = 0.5 * (x1 + x2)
    cy = 0.5 * (y1 + y2)
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)

    if length < 1e-6:
        return ""

    angle_deg = math.degrees(math.atan2(dy, dx))

    return (
        f'    <geom name="{name}" type="box" '
        f'pos="{cx:.6f} {cy:.6f} {height / 2.0:.6f}" '
        f'euler="0 0 {angle_deg:.6f}" '
        f'size="{length / 2.0:.6f} {thickness / 2.0:.6f} {height / 2.0:.6f}" '
        f'material="wall_mat"/>\n'
    )

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="models/pololu_maze_ground_truth.xml")
    ap.add_argument("--wall-thickness", type=float, default=0.020)
    ap.add_argument("--wall-height", type=float, default=0.100)
    ap.add_argument("--floor-margin", type=float, default=0.25)
    args = ap.parse_args()

    xs = []
    ys = []
    for (x1, y1), (x2, y2) in GROUND_TRUTH_WALLS_M:
        xs.extend([x1, x2])
        ys.extend([y1, y2])

    min_x = min(xs) - args.floor_margin
    max_x = max(xs) + args.floor_margin
    min_y = min(ys) - args.floor_margin
    max_y = max(ys) + args.floor_margin

    floor_cx = 0.5 * (min_x + max_x)
    floor_cy = 0.5 * (min_y + max_y)
    floor_sx = 0.5 * (max_x - min_x)
    floor_sy = 0.5 * (max_y - min_y)

    lines = []
    lines.append('<mujoco model="pololu_maze_ground_truth">')
    lines.append('  <compiler angle="degree"/>')
    lines.append('  <option timestep="0.01" gravity="0 0 -9.81"/>')
    lines.append('  <visual>')
    lines.append('    <global azimuth="90" elevation="-90"/>')
    lines.append('    <rgba haze="1 1 1 1"/>')
    lines.append('  </visual>')
    lines.append('')
    lines.append('  <asset>')
    lines.append('    <material name="floor_mat" rgba="1 1 1 1"/>')
    lines.append('    <material name="wall_mat" rgba="0 0 0 1"/>')
    lines.append('    <material name="robot_mat" rgba="0.1 0.3 0.9 1"/>')
    lines.append('    <material name="front_mat" rgba="1 0 0 1"/>')
    lines.append('  </asset>')
    lines.append('')
    lines.append('  <worldbody>')
    lines.append(f'    <geom name="floor" type="plane" size="{floor_sx:.6f} {floor_sy:.6f} 0.02" pos="{floor_cx:.6f} {floor_cy:.6f} 0" material="floor_mat"/>')
    lines.append('')

    for i, (p1, p2) in enumerate(GROUND_TRUTH_WALLS_M):
        lines.append(wall_box_xml(f"gt_wall_{i:03d}", p1, p2, args.wall_thickness, args.wall_height).rstrip())

    lines.append('')
    lines.append('    <body name="pololu" pos="0 0 0.04">')
    lines.append('      <freejoint name="pololu_free"/>')
    lines.append('      <geom name="robot_body" type="box" size="0.045 0.035 0.02" material="robot_mat"/>')
    lines.append('      <geom name="robot_front" type="box" pos="0.045 0 0.012" size="0.015 0.025 0.01" material="front_mat"/>')
    lines.append('    </body>')
    lines.append('  </worldbody>')
    lines.append('</mujoco>')

    out = Path(args.out)
    out.write_text("\n".join(lines) + "\n")
    print(f"Wrote {out}")
    print(f"Walls: {len(GROUND_TRUTH_WALLS_M)}")
    print(f"Wall thickness: {args.wall_thickness:.3f} m")
    print(f"Wall height: {args.wall_height:.3f} m")

if __name__ == "__main__":
    main()
