#!/usr/bin/env python3
"""
Plot a 2D maze/contact map from Pololu bump + encoder + optional gyro CSV logs.

Usage:
  python plot_pololu_bump_maze_map.py --csv pololu_log.csv --out-dir results

Outputs:
  <name>_maze_path.pdf
  <name>_maze_grid.pdf
  <name>_maze_cleaned.csv
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

BASE_COLUMNS = [
    "host_receive_iso",
    "host_receive_unix_ms",
    "robot_time_us",
    "robot_elapsed_us",
    "sample",
    "mode",
    "bump_left",
    "bump_right",
    "left_encoder_deg",
    "right_encoder_deg",
    "delta_left_encoder_deg",
    "delta_right_encoder_deg",
    "left_distance_m",
    "right_distance_m",
    "x_m",
    "y_m",
    "theta_deg",
    "left_cmd_milli",
    "right_cmd_milli",
]

GYRO_COLUMNS = [
    "host_receive_iso",
    "host_receive_unix_ms",
    "robot_time_us",
    "robot_elapsed_us",
    "sample",
    "mode",
    "bump_left",
    "bump_right",
    "left_encoder_deg",
    "right_encoder_deg",
    "delta_left_encoder_deg",
    "delta_right_encoder_deg",
    "left_distance_m",
    "right_distance_m",
    "x_m",
    "y_m",
    "theta_deg",
    "gyro_z_deg",
    "turn_target_deg",
    "left_cmd_milli",
    "right_cmd_milli",
]

NUMERIC_COLUMNS = [
    "host_receive_unix_ms",
    "robot_time_us",
    "robot_elapsed_us",
    "sample",
    "bump_left",
    "bump_right",
    "left_encoder_deg",
    "right_encoder_deg",
    "delta_left_encoder_deg",
    "delta_right_encoder_deg",
    "left_distance_m",
    "right_distance_m",
    "x_m",
    "y_m",
    "theta_deg",
    "gyro_z_deg",
    "turn_target_deg",
    "left_cmd_milli",
    "right_cmd_milli",
]


def read_log(csv_path: Path) -> pd.DataFrame:
    """Read logs even when the header and data rows have different widths.

    Some captured files have the old 19-column header but newer 21-column
    rows that include gyro_z_deg and turn_target_deg. Pandas' CSV parser
    rejects that mix before the script can repair it, so this function reads
    the file line-by-line first and then assigns the correct schema.
    """
    import csv

    rows = []
    with csv_path.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            if row[0].strip().startswith("#"):
                continue
            rows.append([cell.strip() for cell in row])

    if not rows:
        raise ValueError(f"No data found in {csv_path}")

    first_row = rows[0]
    has_header = "robot_time_us" in first_row and "x_m" in first_row and "y_m" in first_row

    if has_header:
        header = first_row
        data_rows = rows[1:]
    else:
        header = []
        data_rows = rows

    if not data_rows:
        raise ValueError(f"No data rows found in {csv_path}")

    row_widths = [len(row) for row in data_rows]
    max_width = max(row_widths)

    if max_width == len(GYRO_COLUMNS):
        columns = GYRO_COLUMNS
    elif max_width == len(BASE_COLUMNS):
        columns = BASE_COLUMNS
    elif has_header and len(header) == max_width:
        columns = header
    else:
        raise ValueError(
            f"Data rows have up to {max_width} columns. Expected "
            f"{len(BASE_COLUMNS)} or {len(GYRO_COLUMNS)}. Row widths found: {sorted(set(row_widths))}"
        )

    fixed_rows = []
    skipped = 0
    for row in data_rows:
        if len(row) < len(columns):
            # Pad short rows so one bad/truncated line does not break the whole log.
            row = row + [""] * (len(columns) - len(row))
        elif len(row) > len(columns):
            # Keep the schema columns and drop trailing garbage, if any.
            row = row[: len(columns)]

        # Keep only rows that look like data rows, not repeated headers.
        if "robot_time_us" in row or "x_m" in row:
            skipped += 1
            continue

        fixed_rows.append(row)

    if not fixed_rows:
        raise ValueError("No usable data rows after removing headers/blank lines.")

    data = pd.DataFrame(fixed_rows, columns=columns)

    for col in NUMERIC_COLUMNS:
        if col in data.columns:
            data[col] = pd.to_numeric(data[col], errors="coerce")

    data = data.dropna(subset=["x_m", "y_m", "theta_deg"]).reset_index(drop=True)
    if data.empty:
        raise ValueError("No valid x_m/y_m/theta_deg rows after cleaning.")

    data["bump_any"] = (data["bump_left"].fillna(0).astype(int) != 0) | (
        data["bump_right"].fillna(0).astype(int) != 0
    )
    return data


def project_contacts(df: pd.DataFrame, forward_offset_m: float, lateral_offset_m: float) -> pd.DataFrame:
    """Estimate wall-contact points from robot pose and left/right bump side."""
    contacts = df[df["bump_any"]].copy()
    if contacts.empty:
        return contacts.assign(contact_x_m=[], contact_y_m=[], contact_type=[])

    theta = np.deg2rad(contacts["theta_deg"].to_numpy(dtype=float))
    x = contacts["x_m"].to_numpy(dtype=float)
    y = contacts["y_m"].to_numpy(dtype=float)
    left = contacts["bump_left"].fillna(0).to_numpy(dtype=int) != 0
    right = contacts["bump_right"].fillna(0).to_numpy(dtype=int) != 0

    contact_x = []
    contact_y = []
    contact_type = []

    for i in range(len(contacts)):
        # Forward direction unit vector.
        fx = math.cos(theta[i])
        fy = math.sin(theta[i])
        # Left-normal unit vector.
        lx = -math.sin(theta[i])
        ly = math.cos(theta[i])

        if left[i] and right[i]:
            lateral = 0.0
            ctype = "both"
        elif left[i]:
            lateral = lateral_offset_m
            ctype = "left"
        elif right[i]:
            lateral = -lateral_offset_m
            ctype = "right"
        else:
            lateral = 0.0
            ctype = "unknown"

        contact_x.append(x[i] + forward_offset_m * fx + lateral * lx)
        contact_y.append(y[i] + forward_offset_m * fy + lateral * ly)
        contact_type.append(ctype)

    contacts["contact_x_m"] = contact_x
    contacts["contact_y_m"] = contact_y
    contacts["contact_type"] = contact_type
    return contacts


def set_equal_axes(ax):
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.5, alpha=0.5)
    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")


def plot_path_contacts(df: pd.DataFrame, contacts: pd.DataFrame, out_base: Path, heading_stride: int):
    fig, ax = plt.subplots(figsize=(8, 8))

    ax.plot(df["x_m"], df["y_m"], linewidth=1.8, label="estimated robot path")
    ax.scatter(df["x_m"].iloc[0], df["y_m"].iloc[0], marker="o", s=70, label="start")
    ax.scatter(df["x_m"].iloc[-1], df["y_m"].iloc[-1], marker="s", s=70, label="end")

    if not contacts.empty:
        ax.scatter(
            contacts["contact_x_m"],
            contacts["contact_y_m"],
            marker="x",
            s=60,
            label="single bump",
        )

        both = contacts[contacts["contact_type"] == "both"]
        if not both.empty:
            ax.scatter(both["contact_x_m"], both["contact_y_m"], marker="D", s=35, label="both bump")

    if heading_stride > 0 and len(df) > heading_stride:
        sub = df.iloc[::heading_stride]
        theta = np.deg2rad(sub["theta_deg"].to_numpy(dtype=float))
        ax.quiver(
            sub["x_m"],
            sub["y_m"],
            np.cos(theta),
            np.sin(theta),
            angles="xy",
            scale_units="xy",
            scale=20,
            width=0.003,
            alpha=0.6,
            color="gray",
            label="reverse motion after bump",
        )

    set_equal_axes(ax)
    ymin, ymax = ax.get_ylim()
    ax.set_ylim(ymin, ymax + 0.05)
    ax.set_title("Pololu maze path")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_base.with_name(out_base.name + "_maze_path.pdf"))
    plt.close(fig)


def plot_contact_grid(contacts: pd.DataFrame, out_base: Path, grid_resolution_m: float):
    if contacts.empty:
        return

    x = contacts["contact_x_m"].to_numpy(dtype=float)
    y = contacts["contact_y_m"].to_numpy(dtype=float)

    xmin = math.floor((x.min() - grid_resolution_m) / grid_resolution_m) * grid_resolution_m
    xmax = math.ceil((x.max() + grid_resolution_m) / grid_resolution_m) * grid_resolution_m
    ymin = math.floor((y.min() - grid_resolution_m) / grid_resolution_m) * grid_resolution_m
    ymax = math.ceil((y.max() + grid_resolution_m) / grid_resolution_m) * grid_resolution_m

    xbins = np.arange(xmin, xmax + grid_resolution_m, grid_resolution_m)
    ybins = np.arange(ymin, ymax + grid_resolution_m, grid_resolution_m)
    hist, xedges, yedges = np.histogram2d(x, y, bins=[xbins, ybins])

    fig, ax = plt.subplots(figsize=(8, 8))
    mesh = ax.pcolormesh(xedges, yedges, hist.T, shading="auto")
    fig.colorbar(mesh, ax=ax, label="bump contact count")
    ax.scatter(x, y, marker="x", s=25, label="contact samples")
    set_equal_axes(ax)
    ax.set_title(f"Bump-contact occupancy grid, {grid_resolution_m:.3f} m cells")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_base.with_name(out_base.name + "_maze_grid.pdf"))
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot Pololu bump maze map from CSV log.")
    parser.add_argument("--csv", required=True, type=Path, help="Input CSV log file")
    parser.add_argument("--out-dir", default=Path("results"), type=Path, help="Output directory")
    parser.add_argument("--bump-forward-offset-m", default=0.055, type=float, help="Forward offset from robot center to bump contact point")
    parser.add_argument("--bump-lateral-offset-m", default=0.025, type=float, help="Left/right lateral offset for single-side contacts")
    parser.add_argument("--grid-resolution-m", default=0.025, type=float, help="Contact-grid cell size")
    parser.add_argument("--heading-stride", default=5, type=int, help="Draw one heading arrow every N samples; use 0 to disable")
    args = parser.parse_args()

    df = read_log(args.csv)
    contacts = project_contacts(df, args.bump_forward_offset_m, args.bump_lateral_offset_m)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_base = args.out_dir / args.csv.stem

    df.to_csv(out_base.with_name(out_base.name + "_maze_cleaned.csv"), index=False)
    if not contacts.empty:
        contacts.to_csv(out_base.with_name(out_base.name + "_contacts.csv"), index=False)

    plot_path_contacts(df, contacts, out_base, args.heading_stride)
    plot_contact_grid(contacts, out_base, args.grid_resolution_m)

    print(f"Read rows: {len(df)}")
    print(f"Bump/contact rows: {len(contacts)}")
    print(f"Wrote outputs under: {args.out_dir}")


if __name__ == "__main__":
    main()
