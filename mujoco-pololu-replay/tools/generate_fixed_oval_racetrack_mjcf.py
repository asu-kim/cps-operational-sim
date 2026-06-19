#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

def read_start_pose(csv_path):
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                return float(r["x_m"]), float(r["y_m"]), float(r.get("theta_deg", 0.0))
            except Exception:
                continue
    return 0.0, 0.0, 0.0

def stadium_centerline_points(length, width, n_arc):
    # Stadium centerline:
    # total length = straight length + 2 * end radius
    # total width = 2 * end radius
    r = width / 2.0
    straight = max(length - width, 0.001)

    left_cx = 0.0
    right_cx = straight
    top_y = r
    bot_y = -r

    pts = []

    # Start on lower straight, a little after the left curve.
    pts.append((left_cx, bot_y))
    pts.append((right_cx, bot_y))

    # Right semicircle: bottom -> top.
    for i in range(1, n_arc + 1):
        a = -math.pi / 2.0 + math.pi * i / n_arc
        pts.append((right_cx + r * math.cos(a), r * math.sin(a)))

    pts.append((left_cx, top_y))

    # Left semicircle: top -> bottom.
    for i in range(1, n_arc + 1):
        a = math.pi / 2.0 + math.pi * i / n_arc
        pts.append((left_cx + r * math.cos(a), r * math.sin(a)))

    return pts

def write_model(out_path, csv_path, length, width, line_width, n_arc, start_side):
    sx, sy, sth = read_start_pose(csv_path)

    pts = stadium_centerline_points(length, width, n_arc)

    # Align CSV start with a point on the stadium.
    # Your photo/run starts on a side/straight section, not at the curve tip.
    if start_side == "lower":
        anchor = pts[0]  # left start of lower straight
    elif start_side == "upper":
        # point near left start of upper straight
        anchor = (0.0, width / 2.0)
    elif start_side == "left":
        anchor = (-width / 2.0, 0.0)
    elif start_side == "right":
        anchor = (length - width + width / 2.0, 0.0)
    else:
        raise SystemExit(f"Unknown start side: {start_side}")

    dx = sx - anchor[0]
    dy = sy - anchor[1]
    pts = [(x + dx, y + dy) for x, y in pts]

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    cx = 0.5 * (min(xs) + max(xs))
    cy = 0.5 * (min(ys) + max(ys))
    floor_size = max(max(xs) - min(xs), max(ys) - min(ys)) + 0.8

    yaw = math.radians(sth)
    qw = math.cos(0.5 * yaw)
    qz = math.sin(0.5 * yaw)

    lines = []
    lines.append('<mujoco model="fixed_stadium_racetrack">')
    lines.append('  <compiler angle="degree"/>')
    lines.append('  <option timestep="0.01" gravity="0 0 -9.81"/>')
    lines.append('  <visual>')
    lines.append('    <global azimuth="90" elevation="-90"/>')
    lines.append('    <rgba haze="1 1 1 1"/>')
    lines.append('  </visual>')
    lines.append('  <asset>')
    lines.append('    <material name="floor_mat" rgba="1 1 1 1"/>')
    lines.append('    <material name="track_mat" rgba="0 0 0 1"/>')
    lines.append('    <material name="robot_mat" rgba="0.1 0.3 0.9 1"/>')
    lines.append('    <material name="front_mat" rgba="1 0 0 1"/>')
    lines.append('  </asset>')
    lines.append('  <worldbody>')
    lines.append(f'    <geom name="floor" type="plane" pos="{cx:.6f} {cy:.6f} 0" size="{floor_size:.6f} {floor_size:.6f} 0.02" material="floor_mat"/>')

    geom_i = 0
    for x1, y1 in pts:
        pass

    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        # MuJoCo rejects capsule fromto geoms with nearly identical endpoints.
        if math.hypot(x2 - x1, y2 - y1) < 1e-5:
            continue

        lines.append(
            f'    <geom name="ground_truth_{geom_i:04d}" type="capsule" '
            f'fromto="{x1:.6f} {y1:.6f} 0.006 {x2:.6f} {y2:.6f} 0.006" '
            f'size="{line_width / 2.0:.6f}" material="track_mat" contype="0" conaffinity="0"/>'
        )
        geom_i += 1

    lines.append(f'    <body name="pololu" pos="{sx:.6f} {sy:.6f} 0.040000" quat="{qw:.6f} 0 0 {qz:.6f}">')
    lines.append('      <freejoint name="pololu_free"/>')
    lines.append('      <geom name="robot_body" type="box" size="0.045 0.035 0.02" material="robot_mat"/>')
    lines.append('      <geom name="robot_front" type="box" pos="0.045 0 0.012" size="0.015 0.025 0.01" material="front_mat"/>')
    lines.append('    </body>')
    lines.append('  </worldbody>')
    lines.append('</mujoco>')

    out_path.write_text("\n".join(lines) + "\n")

    print(f"Wrote {out_path}")
    print(f"CSV start pose: x={sx:.3f}, y={sy:.3f}, theta={sth:.1f}")
    print(f"Ground truth type: stadium/capsule track")
    print(f"Centerline length={length:.4f} m, centerline width={width:.4f} m")
    print(f"Displayed black line width={line_width:.4f} m")
    print(f"Start side={start_side}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="data/complete.csv")
    ap.add_argument("--out", default="models/pololu_fixed_oval_ground_truth.xml")
    ap.add_argument("--length", type=float, default=0.6096)
    ap.add_argument("--width", type=float, default=0.1905)
    ap.add_argument("--line-width", "--track-width", dest="line_width", type=float, default=0.030)
    ap.add_argument("--segments", type=int, default=48)
    ap.add_argument("--start-side", choices=["lower", "upper", "left", "right"], default="lower")
    args = ap.parse_args()

    write_model(
        Path(args.out),
        args.csv,
        args.length,
        args.width,
        args.line_width,
        args.segments,
        args.start_side,
    )

if __name__ == "__main__":
    main()
