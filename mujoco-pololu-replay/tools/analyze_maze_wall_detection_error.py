#!/usr/bin/env python3
import argparse
import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt


def parse_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return default


def read_robot_rows(csv_path):
    rows = []

    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for r in reader:
            if "x_m" not in r or "y_m" not in r or "theta_deg" not in r:
                continue

            try:
                row = {
                    "sample": int(float(r.get("sample", len(rows)))),
                    "mode": str(r.get("mode", "")).strip().upper(),
                    "x_m": float(r["x_m"]),
                    "y_m": float(r["y_m"]),
                    "theta_deg": float(r["theta_deg"]),
                    "bump_left": int(float(r.get("bump_left", 0) or 0)),
                    "bump_right": int(float(r.get("bump_right", 0) or 0)),
                    "robot_elapsed_us": int(float(r.get("robot_elapsed_us", 0) or 0)),
                }
            except Exception:
                continue

            rows.append(row)

    if len(rows) < 2:
        raise SystemExit(f"Not enough valid pose rows in {csv_path}")

    return rows


def detect_reverse_wall_points(rows, forward_offset_m, lateral_offset_m, use_previous_row=True):
    detections = []

    prev_mode = rows[0]["mode"]

    for i in range(1, len(rows)):
        mode = rows[i]["mode"]

        # Detect transition into BACKUP.
        if mode == "BACKUP" and prev_mode != "BACKUP":
            pose_i = i - 1 if use_previous_row else i
            pose = rows[pose_i]

            theta = math.radians(pose["theta_deg"])
            fx = math.cos(theta)
            fy = math.sin(theta)

            # Left normal.
            lx = -math.sin(theta)
            ly = math.cos(theta)

            # Usually use only forward_offset_m. Optional lateral offset is useful
            # if you want left/right bumper-specific contact points.
            lateral = 0.0
            if pose["bump_left"] and not pose["bump_right"]:
                lateral = lateral_offset_m
            elif pose["bump_right"] and not pose["bump_left"]:
                lateral = -lateral_offset_m

            detected_x = pose["x_m"] + forward_offset_m * fx + lateral * lx
            detected_y = pose["y_m"] + forward_offset_m * fy + lateral * ly

            detections.append({
                "event_index": len(detections),
                "row_index": pose_i,
                "sample": pose["sample"],
                "robot_elapsed_us": pose["robot_elapsed_us"],
                "mode_before_reverse": pose["mode"],
                "bump_left": pose["bump_left"],
                "bump_right": pose["bump_right"],
                "robot_x_m": pose["x_m"],
                "robot_y_m": pose["y_m"],
                "robot_theta_deg": pose["theta_deg"],
                "detected_wall_x_m": detected_x,
                "detected_wall_y_m": detected_y,
            })

        prev_mode = mode

    return detections


def parse_vec(text):
    return [float(x) for x in text.split()]


def parse_gt_wall_segments_from_xml(xml_path):
    root = ET.parse(xml_path).getroot()
    segments = []

    for geom in root.findall(".//geom"):
        name = geom.attrib.get("name", "")

        if not name.startswith("gt_wall_") and not name.startswith("ground_truth_"):
            continue

        # Maze walls are usually boxes.
        if geom.attrib.get("type") == "box":
            pos = parse_vec(geom.attrib.get("pos", "0 0 0"))
            size = parse_vec(geom.attrib.get("size", "0 0 0"))
            euler = parse_vec(geom.attrib.get("euler", "0 0 0"))

            cx, cy = pos[0], pos[1]
            half_len = size[0]
            yaw = math.radians(euler[2])

            dx = half_len * math.cos(yaw)
            dy = half_len * math.sin(yaw)

            p1 = (cx - dx, cy - dy)
            p2 = (cx + dx, cy + dy)
            segments.append((name, p1, p2))
            continue

        # Racetrack-style ground truth can be capsules with fromto.
        fromto = geom.attrib.get("fromto")
        if fromto:
            vals = parse_vec(fromto)
            if len(vals) == 6:
                p1 = (vals[0], vals[1])
                p2 = (vals[3], vals[4])
                segments.append((name, p1, p2))

    segments.sort(key=lambda item: item[0])

    if not segments:
        raise SystemExit(f"No gt_wall_* or ground_truth_* segments found in {xml_path}")

    return segments


def point_to_segment_distance(px, py, ax, ay, bx, by):
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay

    denom = vx * vx + vy * vy
    if denom <= 1e-15:
        return math.hypot(px - ax, py - ay), ax, ay

    t = (wx * vx + wy * vy) / denom
    t = max(0.0, min(1.0, t))

    cx = ax + t * vx
    cy = ay + t * vy

    return math.hypot(px - cx, py - cy), cx, cy


def nearest_wall(point, segments):
    px, py = point
    best = None

    for name, a, b in segments:
        dist, cx, cy = point_to_segment_distance(px, py, a[0], a[1], b[0], b[1])
        if best is None or dist < best["distance_m"]:
            best = {
                "nearest_wall": name,
                "distance_m": dist,
                "nearest_x_m": cx,
                "nearest_y_m": cy,
            }

    return best


def rms(values):
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def write_detection_csv(out_csv, detections):
    if not detections:
        return

    out_csv.parent.mkdir(parents=True, exist_ok=True)

    fields = list(detections[0].keys())

    with out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(detections)


def plot_results(out_pdf, rows, detections, segments):
    out_pdf.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(9, 7))

    first = True
    for name, a, b in segments:
        ax.plot(
            [a[0], b[0]],
            [a[1], b[1]],
            color="black",
            linewidth=3.0,
            label="ground truth walls" if first else None,
        )
        first = False

    ax.plot(
        [r["x_m"] for r in rows],
        [r["y_m"] for r in rows],
        linewidth=1.2,
        label="robot path",
    )

    if detections:
        ax.scatter(
            [d["detected_wall_x_m"] for d in detections],
            [d["detected_wall_y_m"] for d in detections],
            marker="x",
            s=80,
            label="detected wall points",
        )

        for d in detections:
            ax.plot(
                [d["detected_wall_x_m"], d["nearest_x_m"]],
                [d["detected_wall_y_m"], d["nearest_y_m"]],
                linestyle="--",
                linewidth=0.8,
            )

    ax.scatter(rows[0]["x_m"], rows[0]["y_m"], marker="o", s=60, label="start")
    ax.scatter(rows[-1]["x_m"], rows[-1]["y_m"], marker="s", s=60, label="end")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.5, alpha=0.5)
    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")
    ax.set_title("Detected wall points vs MuJoCo ground truth")
    ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(out_pdf)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="Bump maze robot CSV log.")
    ap.add_argument("--xml", required=True, help="MuJoCo maze XML ground truth.")
    ap.add_argument("--out-csv", default="results/pololu-3pi/maze/detected_walls.csv")
    ap.add_argument("--out-pdf", default="results/pololu-3pi/maze/detected_walls_vs_ground_truth.pdf")
    ap.add_argument("--wall-forward-offset-m", type=float, default=0.045)
    ap.add_argument("--wall-lateral-offset-m", type=float, default=0.0)
    ap.add_argument("--use-current-backup-row", action="store_true")
    args = ap.parse_args()

    rows = read_robot_rows(args.csv)
    segments = parse_gt_wall_segments_from_xml(args.xml)

    detections = detect_reverse_wall_points(
        rows,
        forward_offset_m=args.wall_forward_offset_m,
        lateral_offset_m=args.wall_lateral_offset_m,
        use_previous_row=not args.use_current_backup_row,
    )

    for d in detections:
        best = nearest_wall(
            (d["detected_wall_x_m"], d["detected_wall_y_m"]),
            segments,
        )
        d.update(best)
        d["error_cm"] = d["distance_m"] * 100.0

    distances = [d["distance_m"] for d in detections]

    write_detection_csv(Path(args.out_csv), detections)
    plot_results(Path(args.out_pdf), rows, detections, segments)

    print()
    print("MAZE WALL DETECTION ERROR REPORT")
    print("================================")
    print(f"CSV: {args.csv}")
    print(f"XML: {args.xml}")
    print(f"Detected wall points: {len(detections)}")
    print(f"Wall projection offset: {args.wall_forward_offset_m:.3f} m")
    print()

    if detections:
        print(f"Mean error: {sum(distances) / len(distances):.4f} m  ({sum(distances) / len(distances) * 100.0:.2f} cm)")
        print(f"RMS error:  {rms(distances):.4f} m  ({rms(distances) * 100.0:.2f} cm)")
        print(f"Max error:  {max(distances):.4f} m  ({max(distances) * 100.0:.2f} cm)")
        print()
        print(f"Wrote: {args.out_csv}")
        print(f"Wrote: {args.out_pdf}")
    else:
        print("No BACKUP transitions found. Check that the CSV mode column contains BACKUP.")


if __name__ == "__main__":
    main()
