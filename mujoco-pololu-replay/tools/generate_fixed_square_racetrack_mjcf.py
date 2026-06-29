#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path


def read_start_pose(csv_path):
    try:
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for r in reader:
                try:
                    return float(r.get("x_m", 0.0)), float(r.get("y_m", 0.0)), float(r.get("theta_deg", 0.0))
                except Exception:
                    continue
    except FileNotFoundError:
        pass
    return 0.0, 0.0, 0.0


def measured_track_points(lower, right, upper, left):
    # Robot order: lower -> right -> upper -> left -> lower.
    # Put the lower side horizontal and the right side vertical.
    # Then compute the upper-left point from the upper and left side lengths.
    p0 = (0.0, 0.0)
    p1 = (lower, 0.0)
    p2 = (lower, right)

    d = math.hypot(p2[0] - p0[0], p2[1] - p0[1])
    if d <= 1e-9:
        raise SystemExit("Invalid lower/right lengths.")
    if d > upper + left or d < abs(upper - left):
        raise SystemExit(
            "The measured side lengths cannot close a 4-sided track with lower horizontal and right vertical."
        )

    # P3 is the intersection of:
    #   circle centered at p0 with radius left
    #   circle centered at p2 with radius upper
    a = (left * left - upper * upper + d * d) / (2.0 * d)
    h = math.sqrt(max(0.0, left * left - a * a))

    ux = (p2[0] - p0[0]) / d
    uy = (p2[1] - p0[1]) / d

    base_x = p0[0] + a * ux
    base_y = p0[1] + a * uy

    candidates = [
        (base_x + h * (-uy), base_y + h * ux),
        (base_x - h * (-uy), base_y - h * ux),
    ]

    # Choose the upper-left candidate.
    p3 = max(candidates, key=lambda p: p[1])
    return [p0, p1, p2, p3]


def write_model(out_path, csv_path, lower, right, upper, left, line_width, start_side):
    sx, sy, sth = read_start_pose(csv_path)
    pts = measured_track_points(lower, right, upper, left)

    if start_side == "lower":
        anchor = pts[0]
    elif start_side == "right":
        anchor = pts[1]
    elif start_side == "upper":
        anchor = pts[2]
    elif start_side == "left":
        anchor = pts[3]
    else:
        raise SystemExit(f"Unknown start side: {start_side}")

    dx = sx - anchor[0]
    dy = sy - anchor[1]
    pts = [(x + dx, y + dy) for x, y in pts]

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]

    cx = 0.5 * (min(xs) + max(xs))
    cy = 0.5 * (min(ys) + max(ys))
    floor_size_x = (max(xs) - min(xs)) + 0.5
    floor_size_y = (max(ys) - min(ys)) + 0.5

    yaw = math.radians(sth)
    qw = math.cos(0.5 * yaw)
    qz = math.sin(0.5 * yaw)

    lines = []
    lines.append('<mujoco model="measured_square_racetrack">')
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
    lines.append(
        f'    <geom name="floor" type="plane" pos="{cx:.6f} {cy:.6f} 0" '
        f'size="{floor_size_x:.6f} {floor_size_y:.6f} 0.02" material="floor_mat"/>'
    )

    for geom_i, ((x1, y1), (x2, y2)) in enumerate(zip(pts, pts[1:] + pts[:1])):
        if math.hypot(x2 - x1, y2 - y1) < 1e-5:
            continue

        lines.append(
            f'    <geom name="ground_truth_{geom_i:04d}" type="capsule" '
            f'fromto="{x1:.6f} {y1:.6f} 0.006 {x2:.6f} {y2:.6f} 0.006" '
            f'size="{line_width / 2.0:.6f}" material="track_mat" contype="0" conaffinity="0"/>'
        )

    lines.append(f'    <body name="pololu" pos="{sx:.6f} {sy:.6f} 0.040000" quat="{qw:.6f} 0 0 {qz:.6f}">')
    lines.append('      <freejoint name="pololu_free"/>')
    lines.append('      <geom name="robot_body" type="box" size="0.045 0.035 0.02" material="robot_mat"/>')
    lines.append('      <geom name="robot_front" type="box" pos="0.045 0 0.012" size="0.015 0.025 0.01" material="front_mat"/>')
    lines.append('    </body>')
    lines.append('  </worldbody>')
    lines.append('</mujoco>')

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n")

    segs = [
        math.hypot(pts[(i + 1) % 4][0] - pts[i][0], pts[(i + 1) % 4][1] - pts[i][1])
        for i in range(4)
    ]

    print(f"Wrote {out_path}")
    print(f"CSV start pose: x={sx:.3f}, y={sy:.3f}, theta={sth:.1f}")
    print(f"Measured side lengths:")
    print(f"  lower={segs[0]:.4f} m")
    print(f"  right={segs[1]:.4f} m")
    print(f"  upper={segs[2]:.4f} m")
    print(f"  left ={segs[3]:.4f} m")
    print(f"Displayed black line width={line_width:.4f} m")
    print(f"Start side={start_side}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="data/square_log.csv")
    ap.add_argument("--out", default="models/pololu_fixed_square_ground_truth.xml")
    ap.add_argument("--lower-length", type=float, default=0.277)
    ap.add_argument("--right-length", type=float, default=0.269)
    ap.add_argument("--upper-length", type=float, default=0.290)
    ap.add_argument("--left-length", type=float, default=0.282)
    ap.add_argument("--line-width", "--track-width", dest="line_width", type=float, default=0.030)
    ap.add_argument("--start-side", choices=["lower", "upper", "left", "right"], default="lower")
    args = ap.parse_args()

    write_model(
        Path(args.out),
        args.csv,
        args.lower_length,
        args.right_length,
        args.upper_length,
        args.left_length,
        args.line_width,
        args.start_side,
    )


if __name__ == "__main__":
    main()
