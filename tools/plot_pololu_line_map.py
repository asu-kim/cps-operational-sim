#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate a 2D track map from Pololu 3Pi line sensor logs."
    )

    parser.add_argument(
        "log_file",
        type=Path,
        help="Path to the Pololu CSV log file."
    )

    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("data/pololu-3pi/maps"),
        help="Output directory for generated plots."
    )

    parser.add_argument(
        "--pose-source",
        choices=["auto", "encoder", "imu", "motor"],
        default="auto",
        help="Pose source. auto chooses encoder first, then IMU, then motor commands."
    )

    parser.add_argument(
        "--wheel-base-m",
        type=float,
        default=0.095,
        help="Approximate distance between left and right wheels in meters."
    )

    parser.add_argument(
        "--max-speed-mps",
        type=float,
        default=0.25,
        help="Approximate robot speed in m/s when motor command is 1000 milli."
    )

    parser.add_argument(
        "--sensor-spacing-m",
        type=float,
        default=0.008,
        help="Spacing between adjacent line sensors in meters."
    )

    parser.add_argument(
        "--sensor-forward-offset-m",
        type=float,
        default=0.035,
        help="Forward offset of line sensor array from robot center in meters."
    )

    parser.add_argument(
        "--dark-threshold",
        type=float,
        default=500.0,
        help="Line sensor threshold. Values above this are treated as track/line."
    )

    parser.add_argument(
        "--cell-size-m",
        type=float,
        default=0.005,
        help="Grid cell size in meters."
    )

    parser.add_argument(
        "--format",
        choices=["pdf", "png"],
        default="pdf",
        help="Output plot format."
    )

    return parser.parse_args()


def clean_fieldnames(fieldnames):
    return [name.strip().lstrip("\ufeff") for name in fieldnames]


def parse_float(row, name, default=None):
    value = row.get(name, "")
    if value is None or value == "":
        return default
    return float(value)


def parse_int(row, name, default=None):
    value = row.get(name, "")
    if value is None or value == "":
        return default
    return int(float(value))


def read_log(log_file):
    encodings_to_try = ["utf-8-sig", "utf-16", "utf-16-le", "latin-1"]
    last_error = None

    for encoding in encodings_to_try:
        rows = []

        try:
            with log_file.open("r", newline="", encoding=encoding) as f:
                reader = csv.DictReader(f)

                if reader.fieldnames is None:
                    raise ValueError("CSV file has no header row.")

                reader.fieldnames = clean_fieldnames(reader.fieldnames)
                fieldnames = set(reader.fieldnames)

                required_common = {
                    "sample",
                    "mode",
                    "s0",
                    "s1",
                    "s2",
                    "s3",
                    "s4",
                    "left_cmd_milli",
                    "right_cmd_milli",
                }

                missing = sorted(required_common - fieldnames)

                has_time = (
                    "robot_elapsed_us" in fieldnames
                    or "robot_time_us" in fieldnames
                    or "time_us" in fieldnames
                )

                if not has_time:
                    missing.append("robot_elapsed_us OR robot_time_us OR time_us")

                if missing:
                    raise ValueError(f"Missing required columns: {missing}")

                for row in reader:
                    try:
                        if "robot_elapsed_us" in fieldnames and row.get("robot_elapsed_us", "") != "":
                            time_us = parse_int(row, "robot_elapsed_us")
                        elif "robot_time_us" in fieldnames and row.get("robot_time_us", "") != "":
                            time_us = parse_int(row, "robot_time_us")
                        else:
                            time_us = parse_int(row, "time_us")

                        parsed = {
                            "time_us": time_us,
                            "sample": parse_int(row, "sample"),
                            "mode": row.get("mode", ""),
                            "s": [
                                parse_float(row, "s0"),
                                parse_float(row, "s1"),
                                parse_float(row, "s2"),
                                parse_float(row, "s3"),
                                parse_float(row, "s4"),
                            ],
                            "left_cmd_milli": parse_float(row, "left_cmd_milli"),
                            "right_cmd_milli": parse_float(row, "right_cmd_milli"),

                            # IMU fields
                            "gyro_z_dps": parse_float(row, "gyro_z_dps"),
                            "yaw_deg": parse_float(row, "yaw_deg"),

                            # Encoder logger fields
                            "left_encoder_deg": parse_float(row, "left_encoder_deg"),
                            "right_encoder_deg": parse_float(row, "right_encoder_deg"),
                            "delta_left_encoder_deg": parse_float(row, "delta_left_encoder_deg"),
                            "delta_right_encoder_deg": parse_float(row, "delta_right_encoder_deg"),
                            "left_distance_m": parse_float(row, "left_distance_m"),
                            "right_distance_m": parse_float(row, "right_distance_m"),
                            "x_m": parse_float(row, "x_m"),
                            "y_m": parse_float(row, "y_m"),
                            "theta_deg": parse_float(row, "theta_deg"),
                        }

                        if parsed["time_us"] is None:
                            continue

                        if any(value is None for value in parsed["s"]):
                            continue

                        rows.append(parsed)

                    except ValueError:
                        continue

            if len(rows) < 2:
                raise ValueError("Log file does not contain enough valid rows.")

            return rows

        except UnicodeError as error:
            last_error = error
            continue

    raise UnicodeError(
        f"Could not read {log_file} with supported encodings. Last error: {last_error}"
    )


def has_encoder_pose(rows):
    return all(
        row.get("x_m") is not None
        and row.get("y_m") is not None
        and row.get("theta_deg") is not None
        for row in rows
    )


def has_imu_yaw(rows):
    return all(row.get("yaw_deg") is not None for row in rows)


def choose_pose_source(rows, requested):
    if requested != "auto":
        return requested

    if has_encoder_pose(rows):
        return "encoder"

    if has_imu_yaw(rows):
        return "imu"

    return "motor"


def integrate_pose(rows, wheel_base_m, max_speed_mps, pose_source):
    poses = []

    t0 = rows[0]["time_us"]

    if pose_source == "encoder":
        x0 = rows[0]["x_m"]
        y0 = rows[0]["y_m"]
        theta0 = rows[0]["theta_deg"]

        for row in rows:
            poses.append({
                "time_s": (row["time_us"] - t0) / 1_000_000.0,
                "x": row["x_m"] - x0,
                "y": row["y_m"] - y0,
                "theta": math.radians(row["theta_deg"] - theta0),
            })

        return poses

    x = 0.0
    y = 0.0
    theta = 0.0

    yaw0 = rows[0]["yaw_deg"] if pose_source == "imu" else 0.0

    for i, row in enumerate(rows):
        if i == 0:
            dt = 0.0
        else:
            dt = (row["time_us"] - rows[i - 1]["time_us"]) / 1_000_000.0
            dt = max(0.0, min(dt, 0.25))

        left_cmd = row["left_cmd_milli"] / 1000.0
        right_cmd = row["right_cmd_milli"] / 1000.0

        v_left = left_cmd * max_speed_mps
        v_right = right_cmd * max_speed_mps
        v = 0.5 * (v_left + v_right)

        if pose_source == "imu":
            theta = math.radians(row["yaw_deg"] - yaw0)
        else:
            omega = (v_right - v_left) / wheel_base_m
            theta += omega * dt

        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt

        poses.append({
            "time_s": (row["time_us"] - t0) / 1_000_000.0,
            "x": x,
            "y": y,
            "theta": theta,
        })

    return poses


def sensor_world_points(rows, poses, sensor_spacing_m, sensor_forward_offset_m, dark_threshold):
    sensor_offsets = [
        -2.0 * sensor_spacing_m,
        -1.0 * sensor_spacing_m,
         0.0 * sensor_spacing_m,
         1.0 * sensor_spacing_m,
         2.0 * sensor_spacing_m,
    ]

    dark_points = []
    light_points = []

    for row, pose in zip(rows, poses):
        x = pose["x"]
        y = pose["y"]
        theta = pose["theta"]

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        for sensor_index, lateral_offset in enumerate(sensor_offsets):
            local_x = sensor_forward_offset_m
            local_y = lateral_offset

            world_x = x + local_x * cos_t - local_y * sin_t
            world_y = y + local_x * sin_t + local_y * cos_t

            value = row["s"][sensor_index]

            if value >= dark_threshold:
                dark_points.append((world_x, world_y, value))
            else:
                light_points.append((world_x, world_y, value))

    return dark_points, light_points


def build_occupancy_grid(dark_points, light_points, cell_size_m):
    all_points = dark_points + light_points

    if not all_points:
        raise ValueError("No sensor points available to build a map.")

    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]

    margin = 0.10

    min_x = min(xs) - margin
    max_x = max(xs) + margin
    min_y = min(ys) - margin
    max_y = max(ys) + margin

    width = max(1, int(math.ceil((max_x - min_x) / cell_size_m)))
    height = max(1, int(math.ceil((max_y - min_y) / cell_size_m)))

    dark_count = np.zeros((height, width), dtype=float)
    light_count = np.zeros((height, width), dtype=float)

    def to_cell(px, py):
        col = int((px - min_x) / cell_size_m)
        row = int((py - min_y) / cell_size_m)
        col = max(0, min(width - 1, col))
        row = max(0, min(height - 1, row))
        return row, col

    for px, py, _ in dark_points:
        r, c = to_cell(px, py)
        dark_count[r, c] += 1.0

    for px, py, _ in light_points:
        r, c = to_cell(px, py)
        light_count[r, c] += 1.0

    grid = dark_count / (dark_count + light_count + 1e-9)

    extent = [min_x, max_x, min_y, max_y]
    return grid, extent


def plot_route_and_points(poses, dark_points, light_points, out_file, pose_source):
    xs = [p["x"] for p in poses]
    ys = [p["y"] for p in poses]

    plt.figure(figsize=(8, 8))

    if light_points:
        lx = [p[0] for p in light_points]
        ly = [p[1] for p in light_points]
        plt.scatter(lx, ly, s=4, alpha=0.25, label="floor / light readings")

    if dark_points:
        dx = [p[0] for p in dark_points]
        dy = [p[1] for p in dark_points]
        plt.scatter(dx, dy, s=8, alpha=0.75, label="track / dark readings")

    plt.plot(xs, ys, linewidth=2, label=f"estimated robot path ({pose_source})")

    plt.axis("equal")
    plt.xlabel("x position (m)")
    plt.ylabel("y position (m)")
    plt.title("Pololu 2D Route Map from Line Sensor Logs")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(out_file, dpi=200)
    plt.close()


def plot_occupancy_grid(grid, extent, out_file, pose_source):
    plt.figure(figsize=(8, 8))
    plt.imshow(
        grid,
        origin="lower",
        extent=extent,
        interpolation="nearest",
        aspect="equal",
    )
    plt.colorbar(label="track probability from line sensors")
    plt.xlabel("x position (m)")
    plt.ylabel("y position (m)")
    plt.title(f"2D Track Map using {pose_source} pose")
    plt.tight_layout()
    plt.savefig(out_file, dpi=200)
    plt.close()


def write_pose_csv(poses, out_file):
    with out_file.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["time_s", "x", "y", "theta_rad", "theta_deg"])
        writer.writeheader()

        for pose in poses:
            writer.writerow({
                "time_s": pose["time_s"],
                "x": pose["x"],
                "y": pose["y"],
                "theta_rad": pose["theta"],
                "theta_deg": math.degrees(pose["theta"]),
            })


def main():
    args = parse_args()

    rows = read_log(args.log_file)
    pose_source = choose_pose_source(rows, args.pose_source)

    if pose_source == "encoder" and not has_encoder_pose(rows):
        raise ValueError("Requested --pose-source encoder, but x_m, y_m, theta_deg are missing.")

    if pose_source == "imu" and not has_imu_yaw(rows):
        raise ValueError("Requested --pose-source imu, but yaw_deg is missing.")

    poses = integrate_pose(
        rows,
        wheel_base_m=args.wheel_base_m,
        max_speed_mps=args.max_speed_mps,
        pose_source=pose_source,
    )

    dark_points, light_points = sensor_world_points(
        rows,
        poses,
        sensor_spacing_m=args.sensor_spacing_m,
        sensor_forward_offset_m=args.sensor_forward_offset_m,
        dark_threshold=args.dark_threshold,
    )

    grid, extent = build_occupancy_grid(
        dark_points,
        light_points,
        cell_size_m=args.cell_size_m,
    )

    args.out_dir.mkdir(parents=True, exist_ok=True)

    stem = args.log_file.stem

    route_plot = args.out_dir / f"{stem}_{pose_source}_route_points.{args.format}"
    grid_plot = args.out_dir / f"{stem}_{pose_source}_track_grid.{args.format}"
    pose_csv = args.out_dir / f"{stem}_{pose_source}_estimated_pose.csv"

    plot_route_and_points(poses, dark_points, light_points, route_plot, pose_source)
    plot_occupancy_grid(grid, extent, grid_plot, pose_source)
    write_pose_csv(poses, pose_csv)

    print("Generated:")
    print(f"  {route_plot}")
    print(f"  {grid_plot}")
    print(f"  {pose_csv}")
    print()
    print(f"Pose source used: {pose_source}")

    if pose_source == "encoder":
        print("  Using x_m, y_m, theta_deg from wheel encoder odometry.")
    elif pose_source == "imu":
        print("  Using yaw_deg from IMU and motor commands for approximate distance.")
    else:
        print("  Using motor commands for approximate distance and heading.")
        print("  This is the weakest pose estimate.")


if __name__ == "__main__":
    main()
