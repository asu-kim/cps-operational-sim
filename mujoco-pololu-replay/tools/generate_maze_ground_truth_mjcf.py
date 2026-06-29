#!/usr/bin/env python3
from pathlib import Path
import math
import argparse

INCH_TO_M = 0.0254

# Robot start point in drawing coordinates, inches.
# After translation this becomes MuJoCo/world coordinate (0, 0).
START_X_IN = -4.85
START_Y_IN = 15.25

INNER_X_IN = 12.25

GROUND_TRUTH_WALLS_DRAWING_IN = [
    # Bottom long wall: 30 in
    ((0.0, 0.0), (30.0, 0.0)),

    # Left vertical wall: 12.5 in
    ((0.0, 0.0), (0.0, 12.5)),

    # Left short horizontal wall at the start corridor: 6 in
    ((-6.0, 12.25), (0.0, 12.25)),

    # Top-left vertical rise: 6 in
    ((6.25, 12.25), (6.25, 18.25)),

    # Top-left horizontal wall: 12.25 in
    ((-6.0, 18.25), (6.25, 18.25)),

    # Middle horizontal wall
    ((6.25, 12.25), (20.0, 12.25)),

    # Right vertical drop: 6 in
    ((20.0, 12.25), (20.0, 6.5)),

    # Right horizontal wall
    ((20.0, 6.5), (29.75, 6.5)),

    # Inner vertical divider: 6 in
    ((INNER_X_IN, 0.0), (INNER_X_IN, 6.0)),
]


def translate_wall_to_robot_start_inches(wall):
    (x1, y1), (x2, y2) = wall
    return (
        (x1 - START_X_IN, y1 - START_Y_IN),
        (x2 - START_X_IN, y2 - START_Y_IN),
    )


GROUND_TRUTH_WALLS_IN = [
    translate_wall_to_robot_start_inches(wall)
    for wall in GROUND_TRUTH_WALLS_DRAWING_IN
]

GROUND_TRUTH_WALLS_M = [
    ((x1 * INCH_TO_M, y1 * INCH_TO_M), (x2 * INCH_TO_M, y2 * INCH_TO_M))
    for (x1, y1), (x2, y2) in GROUND_TRUTH_WALLS_IN
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

    # MuJoCo box size values are half-extents.
    return (
        f'    <geom name="{name}" type="box" '
        f'pos="{cx:.6f} {cy:.6f} {height / 2.0:.6f}" '
        f'euler="0 0 {angle_deg:.6f}" '
        f'size="{length / 2.0:.6f} {thickness / 2.0:.6f} {height / 2.0:.6f}" '
        f'material="wall_mat" contype="0" conaffinity="0"/>'
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
    lines.append(
        f'    <geom name="floor" type="plane" '
        f'size="{floor_sx:.6f} {floor_sy:.6f} 0.02" '
        f'pos="{floor_cx:.6f} {floor_cy:.6f} 0" material="floor_mat"/>'
    )
    lines.append('')

    for i, (p1, p2) in enumerate(GROUND_TRUTH_WALLS_M):
        lines.append(wall_box_xml(f"gt_wall_{i:03d}", p1, p2, args.wall_thickness, args.wall_height))

    lines.append('')
    lines.append('    <body name="pololu" pos="0 0 0.040000" quat="1 0 0 0">')
    lines.append('      <freejoint name="pololu_free"/>')
    lines.append('      <geom name="robot_body" type="box" size="0.045 0.035 0.02" material="robot_mat"/>')
    lines.append('      <geom name="robot_front" type="box" pos="0.045 0 0.012" size="0.015 0.025 0.01" material="front_mat"/>')
    lines.append('    </body>')
    lines.append('  </worldbody>')
    lines.append('</mujoco>')

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n")

    print(f"Wrote {out}")
    print(f"Walls: {len(GROUND_TRUTH_WALLS_M)}")
    print(f"Robot start in MuJoCo: (0.000 m, 0.000 m)")
    print(f"Wall thickness: {args.wall_thickness:.3f} m")
    print(f"Wall height: {args.wall_height:.3f} m")
    print()
    print("Wall endpoints in meters, robot-start frame:")
    for i, (p1, p2) in enumerate(GROUND_TRUTH_WALLS_M):
        print(f"  gt_wall_{i:03d}: ({p1[0]:+.4f}, {p1[1]:+.4f}) -> ({p2[0]:+.4f}, {p2[1]:+.4f})")


if __name__ == "__main__":
    main()
