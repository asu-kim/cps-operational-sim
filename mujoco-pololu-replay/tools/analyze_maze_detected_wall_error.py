#!/usr/bin/env python3
import argparse
import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_floats(s):
    return [float(x) for x in s.split()]


def read_detected_points(csv_path):
    points = []

    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)

        for row_num, row in enumerate(reader, start=2):
            try:
                x = float(row["detected_wall_x_m"])
                y = float(row["detected_wall_y_m"])
            except Exception:
                continue

            points.append({
                "row_num": row_num,
                "x": x,
                "y": y,
                "source": row,
            })

    if not points:
        raise SystemExit(f"No detected wall points found in {csv_path}")

    return points


def parse_gt_wall_segments(xml_path):
    root = ET.parse(xml_path).getroot()
    segments = []

    for geom in root.findall(".//geom"):
        name = geom.attrib.get("name", "")
        if not name.startswith("gt_wall_"):
            continue

        pos = parse_floats(geom.attrib["pos"])
        size = parse_floats(geom.attrib["size"])
        euler = parse_floats(geom.attrib.get("euler", "0 0 0"))

        cx = pos[0]
        cy = pos[1]
        half_length = size[0]
        yaw = math.radians(euler[2])

        ux = math.cos(yaw)
        uy = math.sin(yaw)

        x1 = cx - half_length * ux
        y1 = cy - half_length * uy
        x2 = cx + half_length * ux
        y2 = cy + half_length * uy

        segments.append({
            "name": name,
            "p1": (x1, y1),
            "p2": (x2, y2),
            "length": 2.0 * half_length,
        })

    if not segments:
        raise SystemExit(f"No gt_wall_* geoms found in {xml_path}")

    segments.sort(key=lambda s: s["name"])
    return segments


def point_to_segment_distance(px, py, ax, ay, bx, by):
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay

    denom = vx * vx + vy * vy
    if denom <= 1e-15:
        qx, qy = ax, ay
    else:
        t = (wx * vx + wy * vy) / denom
        t = max(0.0, min(1.0, t))
        qx = ax + t * vx
        qy = ay + t * vy

    dist = math.hypot(px - qx, py - qy)
    return dist, qx, qy


def nearest_wall(point, segments):
    px = point["x"]
    py = point["y"]

    best = None

    for seg in segments:
        ax, ay = seg["p1"]
        bx, by = seg["p2"]

        dist, qx, qy = point_to_segment_distance(px, py, ax, ay, bx, by)

        if best is None or dist < best["error_m"]:
            best = {
                "wall": seg["name"],
                "error_m": dist,
                "nearest_x_m": qx,
                "nearest_y_m": qy,
            }

    return best


def summarize(values):
    n = len(values)
    mean = sum(values) / n
    rms = math.sqrt(sum(v * v for v in values) / n)
    max_v = max(values)
    min_v = min(values)
    return min_v, mean, rms, max_v


def main():
    ap = argparse.ArgumentParser(
        description="Analyze error between detected maze wall points and MuJoCo ground-truth maze walls."
    )
    ap.add_argument(
        "--detected",
        default="data/maze_detected_walls.csv",
        help="CSV with detected_wall_x_m and detected_wall_y_m columns.",
    )
    ap.add_argument(
        "--xml",
        default="models/pololu_maze_ground_truth.xml",
        help="MuJoCo XML containing gt_wall_* box geoms.",
    )
    ap.add_argument(
        "--out",
        default=None,
        help="Optional per-point error CSV output. Defaults next to detected CSV.",
    )
    ap.add_argument(
        "--wall-thickness",
        type=float,
        default=0.020,
        help="Wall thickness in meters, used for normalized error percentage.",
    )
    args = ap.parse_args()

    detected_path = Path(args.detected)
    xml_path = Path(args.xml)

    points = read_detected_points(detected_path)
    segments = parse_gt_wall_segments(xml_path)

    xs = []
    ys = []

    for seg in segments:
        xs.extend([seg["p1"][0], seg["p2"][0]])
        ys.extend([seg["p1"][1], seg["p2"][1]])

    maze_width = max(xs) - min(xs)
    maze_height = max(ys) - min(ys)
    maze_diagonal = math.hypot(maze_width, maze_height)
    total_wall_length = sum(seg["length"] for seg in segments)

    rows = []
    errors = []

    for idx, point in enumerate(points):
        best = nearest_wall(point, segments)
        e = best["error_m"]
        errors.append(e)

        source = point["source"]

        rows.append({
            "detected_index": idx,
            "source_row": source.get("source_row", ""),
            "sample": source.get("sample", ""),
            "mode": source.get("mode", ""),
            "contact_type": source.get("contact_type", ""),
            "detected_wall_x_m": f"{point['x']:.6f}",
            "detected_wall_y_m": f"{point['y']:.6f}",
            "nearest_gt_wall": best["wall"],
            "nearest_gt_x_m": f"{best['nearest_x_m']:.6f}",
            "nearest_gt_y_m": f"{best['nearest_y_m']:.6f}",
            "error_m": f"{e:.6f}",
            "error_cm": f"{e * 100.0:.3f}",
            "error_pct_wall_thickness": f"{(e / args.wall_thickness * 100.0):.2f}" if args.wall_thickness > 0 else "",
            "error_pct_maze_diagonal": f"{(e / maze_diagonal * 100.0):.2f}" if maze_diagonal > 0 else "",
            "error_pct_total_wall_length": f"{(e / total_wall_length * 100.0):.2f}" if total_wall_length > 0 else "",
        })

    min_e, mean_e, rms_e, max_e = summarize(errors)

    print()
    print("MAZE DETECTED WALL ERROR REPORT")
    print("===============================")
    print(f"Detected wall CSV:        {detected_path}")
    print(f"Ground-truth XML:         {xml_path}")
    print(f"Detected points:          {len(points)}")
    print(f"Ground-truth wall count:  {len(segments)}")
    print()
    print("Ground-truth scale")
    print("------------------")
    print(f"Maze width:               {maze_width:.4f} m")
    print(f"Maze height:              {maze_height:.4f} m")
    print(f"Maze diagonal:            {maze_diagonal:.4f} m")
    print(f"Total GT wall length:     {total_wall_length:.4f} m")
    print(f"Wall thickness:           {args.wall_thickness:.4f} m")
    print()
    print("Detected wall error")
    print("-------------------")
    print(f"Min error:                {min_e:.4f} m  ({min_e * 100.0:.2f} cm)")
    print(f"Mean error:               {mean_e:.4f} m  ({mean_e * 100.0:.2f} cm)")
    print(f"RMS error:                {rms_e:.4f} m  ({rms_e * 100.0:.2f} cm)")
    print(f"Max error:                {max_e:.4f} m  ({max_e * 100.0:.2f} cm)")
    print()
    print("Error percentages")
    print("-----------------")
    print(f"Mean / wall thickness:    {mean_e / args.wall_thickness * 100.0:.2f} %")
    print(f"RMS / wall thickness:     {rms_e / args.wall_thickness * 100.0:.2f} %")
    print(f"Max / wall thickness:     {max_e / args.wall_thickness * 100.0:.2f} %")
    print(f"Mean / maze diagonal:     {mean_e / maze_diagonal * 100.0:.2f} %")
    print(f"RMS / maze diagonal:      {rms_e / maze_diagonal * 100.0:.2f} %")
    print(f"Max / maze diagonal:      {max_e / maze_diagonal * 100.0:.2f} %")
    print(f"Mean / total wall length: {mean_e / total_wall_length * 100.0:.2f} %")
    print(f"RMS / total wall length:  {rms_e / total_wall_length * 100.0:.2f} %")
    print(f"Max / total wall length:  {max_e / total_wall_length * 100.0:.2f} %")
    print()

    if args.out is None:
        out_path = detected_path.with_name(detected_path.stem + "_error.csv")
    else:
        out_path = Path(args.out)

    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote per-point error CSV: {out_path}")


if __name__ == "__main__":
    main()
