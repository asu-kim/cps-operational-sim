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
                    return float(r["x_m"]), float(r["y_m"]), float(r.get("theta_deg", 0.0))
                except Exception:
                    continue
    except FileNotFoundError:
        pass
    return 0.0, 0.0, 0.0


def polyline_length(pts):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(pts, pts[1:] + pts[:1])
    )


def circular_arc_points(center, radius, start_angle, delta_angle, segments):
    pts = []
    for i in range(1, segments + 1):
        a = start_angle + delta_angle * i / segments
        pts.append((
            center[0] + radius * math.cos(a),
            center[1] + radius * math.sin(a),
        ))
    return pts


def solve_theta_for_chord_and_arc(chord, arc_length):
    if arc_length <= chord:
        return 0.0

    # Solve chord / arc = 2*sin(theta/2)/theta for theta in (0, 2*pi).
    target = chord / arc_length
    lo = 1e-9
    hi = 2.0 * math.pi - 1e-9

    for _ in range(100):
        mid = 0.5 * (lo + hi)
        value = 2.0 * math.sin(mid / 2.0) / mid

        # value decreases over the useful range from ~1 to 0.
        if value > target:
            lo = mid
        else:
            hi = mid

    return 0.5 * (lo + hi)


def closing_arc_points(start, end, arc_length, segments):
    sx, sy = start
    ex, ey = end

    dx = ex - sx
    dy = ey - sy
    chord = math.hypot(dx, dy)

    if chord < 1e-9:
        return []

    if arc_length <= chord + 1e-6:
        return [end]

    theta = solve_theta_for_chord_and_arc(chord, arc_length)
    radius = arc_length / theta

    mx = 0.5 * (sx + ex)
    my = 0.5 * (sy + ey)

    h_sq = max(0.0, radius * radius - (chord * 0.5) ** 2)
    h = math.sqrt(h_sq)

    ux = dx / chord
    uy = dy / chord

    # Perpendicular unit vectors.
    nx1, ny1 = -uy, ux
    nx2, ny2 = uy, -ux

    centers = [
        (mx + h * nx1, my + h * ny1),
        (mx + h * nx2, my + h * ny2),
    ]

    candidates = []

    for c in centers:
        a0 = math.atan2(sy - c[1], sx - c[0])
        a1 = math.atan2(ey - c[1], ex - c[0])

        ccw_delta = (a1 - a0) % (2.0 * math.pi)
        cw_delta = ccw_delta - 2.0 * math.pi

        for delta in (ccw_delta, cw_delta):
            length_err = abs(abs(delta) * radius - arc_length)
            pts = circular_arc_points(c, radius, a0, delta, segments)
            if not pts:
                continue

            # Pick the candidate that has the requested length and bulges farther left.
            min_x = min(p[0] for p in pts)
            candidates.append((length_err, min_x, pts, radius, delta))

    if not candidates:
        return [end]

    candidates.sort(key=lambda item: (item[0], item[1]))
    return candidates[0][2]


def measured_stadium_points(lower, upper, right_curve, left_curve, segments):
    # Order: lower -> right curve -> upper -> left curve -> lower.
    # Lower side starts at (0, 0) and goes right.
    p0 = (0.0, 0.0)
    p1 = (lower, 0.0)

    # Treat right side as a circular half-turn whose arc length is measured.
    r_right = right_curve / math.pi
    c_right = (lower, r_right)

    pts = [p0, p1]

    # Right curve: bottom -> top, around the right side.
    pts.extend(
        circular_arc_points(
            c_right,
            r_right,
            -math.pi / 2.0,
            math.pi,
            segments,
        )
    )

    p2 = pts[-1]

    # Upper straight goes left.
    p3 = (p2[0] - upper, p2[1])
    pts.append(p3)

    # Left curve closes back to p0 with the measured left-curve length.
    pts.extend(closing_arc_points(p3, p0, left_curve, segments))

    # Avoid duplicate final point; the XML loop will close the polyline.
    if math.hypot(pts[-1][0] - p0[0], pts[-1][1] - p0[1]) < 1e-9:
        pts.pop()

    return pts


def write_model(
    out_path,
    csv_path,
    lower,
    upper,
    right_curve,
    left_curve,
    line_width,
    segments,
    start_side,
    start_offset,
):
    sx, sy, sth = read_start_pose(csv_path)

    pts = measured_stadium_points(lower, upper, right_curve, left_curve, segments)

    if start_side == "lower":
        anchor = (min(lower, max(0.0, start_offset)), 0.0)
    elif start_side == "right":
        anchor = (lower, right_curve / math.pi)
    elif start_side == "upper":
        r_right = right_curve / math.pi
        anchor = (lower - min(upper, max(0.0, start_offset)), 2.0 * r_right)
    elif start_side == "left":
        anchor = pts[-1]
    else:
        raise SystemExit(f"Unknown start side: {start_side}")

    dx = sx - anchor[0]
    dy = sy - anchor[1]
    pts = [(x + dx, y + dy) for x, y in pts]

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]

    cx = 0.5 * (min(xs) + max(xs))
    cy = 0.5 * (min(ys) + max(ys))
    floor_size_x = (max(xs) - min(xs)) + 0.8
    floor_size_y = (max(ys) - min(ys)) + 0.8

    yaw = math.radians(sth)
    qw = math.cos(0.5 * yaw)
    qz = math.sin(0.5 * yaw)

    lines = []
    lines.append('<mujoco model="measured_stadium_racetrack">')
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

    geom_i = 0
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        if math.hypot(x2 - x1, y2 - y1) < 1e-5:
            continue

        lines.append(
            f'    <geom name="ground_truth_{geom_i:04d}" type="capsule" '
            f'fromto="{x1:.6f} {y1:.6f} 0.006 {x2:.6f} {y2:.6f} 0.006" '
            f'size="{line_width / 2.0:.6f}" material="track_mat" contype="0" conaffinity="0"/>'
        )
        geom_i += 1

    lines.append(
        f'    <body name="pololu" pos="{sx:.6f} {sy:.6f} 0.040000" '
        f'quat="{qw:.6f} 0 0 {qz:.6f}">'
    )
    lines.append('      <freejoint name="pololu_free"/>')
    lines.append('      <geom name="robot_body" type="box" size="0.045 0.035 0.02" material="robot_mat"/>')
    lines.append('      <geom name="robot_front" type="box" pos="0.045 0 0.012" size="0.015 0.025 0.01" material="front_mat"/>')
    lines.append('    </body>')
    lines.append('  </worldbody>')
    lines.append('</mujoco>')

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n")

    expected = lower + upper + right_curve + left_curve
    actual = polyline_length(pts)

    print(f"Wrote {out_path}")
    print(f"CSV start pose: x={sx:.3f}, y={sy:.3f}, theta={sth:.1f}")
    print("Ground truth type: measured asymmetric stadium")
    print(f"lower straight     = {lower:.5f} m")
    print(f"right curve length = {right_curve:.5f} m")
    print(f"upper straight     = {upper:.5f} m")
    print(f"left curve length  = {left_curve:.5f} m")
    print(f"target lap length  = {expected:.5f} m")
    print(f"XML polyline length= {actual:.5f} m")
    print(f"right curve radius = {right_curve / math.pi:.5f} m")
    print(f"line width         = {line_width:.5f} m")
    print(f"start side={start_side}, start offset={start_offset:.5f} m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="data/racetrack_log.csv")
    ap.add_argument("--out", default="models/pololu_fixed_oval_ground_truth.xml")

    ap.add_argument("--lower-straight", type=float, default=0.4318)
    ap.add_argument("--upper-straight", type=float, default=0.34925)
    ap.add_argument("--right-curve-length", type=float, default=0.38795)
    ap.add_argument("--left-curve-length", type=float, default=0.4130)

    ap.add_argument("--line-width", "--track-width", dest="line_width", type=float, default=0.030)
    ap.add_argument("--segments", type=int, default=96)
    ap.add_argument("--start-side", choices=["lower", "upper", "left", "right"], default="lower")
    ap.add_argument("--start-offset", type=float, default=0.0)

    args = ap.parse_args()

    write_model(
        Path(args.out),
        args.csv,
        args.lower_straight,
        args.upper_straight,
        args.right_curve_length,
        args.left_curve_length,
        args.line_width,
        args.segments,
        args.start_side,
        args.start_offset,
    )


if __name__ == "__main__":
    main()
