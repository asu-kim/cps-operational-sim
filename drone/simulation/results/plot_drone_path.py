#!/usr/bin/env python3
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


MID = 1500.0
SPAN = 500.0

SENSOR_NAMES = ("front", "left", "right", "top", "bottom")


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def rc_to_unit(pwm):
    return clamp((pwm - MID) / SPAN, -1.0, 1.0)


def parse_float(text):
    try:
        return float(str(text).strip())
    except (TypeError, ValueError):
        return None


def robust_deadband(pwm_series):
    x = np.asarray(pwm_series, dtype=float)
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return 12

    dev = np.abs(x - MID)
    near = dev[dev < 80]
    if len(near) < 20:
        return 12

    mad = np.median(np.abs(near - np.median(near)))
    return int(clamp(3.0 * mad, 6, 30))


def infer_dt_from_step(step):
    step = np.asarray(step, dtype=float)
    if len(step) < 2:
        return np.full(len(step), 0.02, dtype=float)

    dstep = np.diff(step)
    bad = (~np.isfinite(dstep)) | (dstep <= 0)
    dstep[bad] = 1.0

    dt = 0.02 * dstep
    dt = np.concatenate([[dt[0]], dt])
    return dt


def auto_scales(pitch_u, roll_u, yaw_u):
    mag_xy = np.sqrt(pitch_u ** 2 + roll_u ** 2)
    mag_xy = mag_xy[np.isfinite(mag_xy)]

    yaw_mag = np.abs(yaw_u)
    yaw_mag = yaw_mag[np.isfinite(yaw_mag)]

    p95_xy = np.percentile(mag_xy, 95) if len(mag_xy) else 0.0
    p95_yaw = np.percentile(yaw_mag, 95) if len(yaw_mag) else 0.0

    default_vmax = 1.2
    default_yaw = math.radians(120)

    if p95_xy < 0.05:
        vmax = default_vmax
    else:
        vmax = float(clamp(default_vmax / p95_xy, 0.6, 3.0))

    if p95_yaw < 0.05:
        yawrate = default_yaw
    else:
        yawrate = float(
            clamp(default_yaw / p95_yaw, math.radians(40), math.radians(240))
        )

    return vmax, yawrate


def read_rc_csv(path):
    required = ["step", "roll", "pitch", "yaw", "throttle", "aux1", "aux2"]
    data = {name: [] for name in required}

    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"Empty or headerless RC CSV: {path}")

        missing = [name for name in required if name not in reader.fieldnames]
        if missing:
            raise ValueError(
                f"rc_out missing columns {missing}. Found: {reader.fieldnames}"
            )

        for row in reader:
            for name in required:
                value = parse_float(row.get(name, ""))
                data[name].append(np.nan if value is None else value)

    return {name: np.asarray(values, dtype=float) for name, values in data.items()}


def read_single_sensor_csv(path, sensor_name):
    values = []
    header_index = None
    started = False

    preferred_names = [
        f"{sensor_name}_m",
        "value_m",
        "distance_m",
        "range_m",
        "tof_m",
        sensor_name,
        "value",
        "distance",
        "range",
        "tof",
    ]

    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            cells = [cell.strip() for cell in row]

            if not cells or all(cell == "" for cell in cells):
                if started:
                    values.append(np.nan)
                continue

            if cells[0].startswith("#"):
                continue

            if not started:
                lowered = [cell.lower() for cell in cells]
                has_header_text = any(any(ch.isalpha() for ch in cell) for cell in lowered)

                if has_header_text:
                    for name in preferred_names:
                        if name in lowered:
                            header_index = lowered.index(name)
                            break

                    if header_index is None:
                        header_index = len(cells) - 1

                    started = True
                    continue

                started = True

            value = None

            if header_index is not None and header_index < len(cells):
                value = parse_float(cells[header_index])

            if value is None:
                for cell in reversed(cells):
                    value = parse_float(cell)
                    if value is not None:
                        break

            values.append(np.nan if value is None else value)

    return np.asarray(values, dtype=float)


def load_tof_inputs(data_dir):
    tof = {}
    for name in SENSOR_NAMES:
        path = data_dir / f"{name}.csv"
        if not path.exists():
            raise FileNotFoundError(f"Missing ToF CSV for '{name}': {path}")
        tof[name] = read_single_sensor_csv(path, name)
    return tof


def integrate_from_rc(rc):
    step = rc["step"]
    dt = infer_dt_from_step(step)

    roll = rc["roll"].astype(float).copy()
    pitch = rc["pitch"].astype(float).copy()
    yaw = rc["yaw"].astype(float).copy()

    roll[~np.isfinite(roll)] = MID
    pitch[~np.isfinite(pitch)] = MID
    yaw[~np.isfinite(yaw)] = MID

    db_r = robust_deadband(roll)
    db_p = robust_deadband(pitch)
    db_y = robust_deadband(yaw)

    def apply_db(x, db):
        x = x.copy()
        x[np.abs(x - MID) < db] = MID
        return x

    roll = apply_db(roll, db_r)
    pitch = apply_db(pitch, db_p)
    yaw = apply_db(yaw, db_y)

    roll_u = np.array([rc_to_unit(v) for v in roll], dtype=float)
    pitch_u = np.array([rc_to_unit(v) for v in pitch], dtype=float)
    yaw_u = np.array([rc_to_unit(v) for v in yaw], dtype=float)

    vmax, yawrate_scale = auto_scales(pitch_u, roll_u, yaw_u)

    v_des_fwd = pitch_u * vmax
    v_des_right = roll_u * vmax

    tau_v = 0.35
    tau_y = 0.25

    n = len(step)
    x = np.zeros(n, dtype=float)
    y = np.zeros(n, dtype=float)
    psi = np.zeros(n, dtype=float)

    v_fwd = 0.0
    v_right = 0.0
    yawrate = 0.0

    for i in range(1, n):
        dti = float(dt[i])

        yawrate_des = yaw_u[i - 1] * yawrate_scale
        alpha_v = 1.0 - math.exp(-dti / tau_v)
        alpha_y = 1.0 - math.exp(-dti / tau_y)

        v_fwd += alpha_v * (v_des_fwd[i - 1] - v_fwd)
        v_right += alpha_v * (v_des_right[i - 1] - v_right)
        yawrate += alpha_y * (yawrate_des - yawrate)

        psi[i] = psi[i - 1] + yawrate * dti

        c = math.cos(psi[i])
        s = math.sin(psi[i])

        vx = c * v_fwd - s * v_right
        vy = s * v_fwd + c * v_right

        x[i] = x[i - 1] + vx * dti
        y[i] = y[i - 1] + vy * dti

    return x, y, psi


def point_to_segment_distance(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    denom = abx * abx + aby * aby
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)

    t = (apx * abx + apy * aby) / denom
    t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t

    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy)


def point_to_polyline_distance(px, py, x, y):
    best = float("inf")
    for i in range(1, len(x)):
        d = point_to_segment_distance(px, py, x[i - 1], y[i - 1], x[i], y[i])
        if d < best:
            best = d
    return best


def resolve_paths(argv):
    results_dir = Path(__file__).resolve().parent
    simulation_dir = results_dir.parent

    default_rc = results_dir / "rc-out-2.csv"
    default_data = simulation_dir / "data"

    if len(argv) == 1:
        return default_rc, default_data

    if len(argv) == 3:
        return Path(argv[1]).resolve(), Path(argv[2]).resolve()

    prog = Path(argv[0]).name
    raise SystemExit(f"Usage: python3 {prog} [rc-out.csv tof-data-dir]")


def main():
    rc_path, data_dir = resolve_paths(sys.argv)

    if not rc_path.exists():
        raise FileNotFoundError(f"Missing RC log: {rc_path}")
    if not data_dir.exists():
        raise FileNotFoundError(f"Missing ToF data directory: {data_dir}")

    rc = read_rc_csv(rc_path)
    tof = load_tof_inputs(data_dir)

    lengths = [len(rc["step"])] + [len(values) for values in tof.values()]
    n = min(lengths)
    if n == 0:
        raise ValueError("No aligned samples found in rc-out.csv / ToF CSVs")

    rc = {name: values[:n] for name, values in rc.items()}
    tof = {name: values[:n] for name, values in tof.items()}

    x, y, psi = integrate_from_rc(rc)

    x = x - x[0]
    y = y - y[0]

    f = tof["front"]
    l = tof["left"]
    r = tof["right"]
    t = tof["top"]
    b = tof["bottom"]

    obs_thresh = 0.30
    top_thresh = 0.30
    cruise_bottom = 0.30

    cap_dist = 0.05
    max_path_dist = 0.20
    min_clearance = 0.003

    in_cruise = np.isfinite(b) & (b > cruise_bottom)

    hit_front = in_cruise & np.isfinite(f) & (f < obs_thresh)
    hit_left = in_cruise & np.isfinite(l) & (l < obs_thresh)
    hit_right = in_cruise & np.isfinite(r) & (r < obs_thresh)
    hit_top = in_cruise & np.isfinite(t) & (t < top_thresh)

    hit_any_xy = hit_front | hit_left | hit_right

    ox, oy = [], []
    for i in np.where(hit_any_xy)[0]:
        candidates = []

        if hit_front[i]:
            candidates.append(("front", float(f[i])))
        if hit_left[i]:
            candidates.append(("left", float(l[i])))
        if hit_right[i]:
            candidates.append(("right", float(r[i])))

        if not candidates:
            continue

        direction, dist = min(candidates, key=lambda item: item[1])
        d = clamp(dist, 0.01, cap_dist)

        if direction == "front":
            bf, br = 1.0, 0.0
        elif direction == "left":
            bf, br = 0.0, -1.0
        else:
            bf, br = 0.0, 1.0

        c = math.cos(psi[i])
        s = math.sin(psi[i])

        dx = c * (bf * d) - s * (br * d)
        dy = s * (bf * d) + c * (br * d)

        px = float(x[i] + dx)
        py = float(y[i] + dy)

        dist_to_path = point_to_polyline_distance(px, py, x, y)
        if min_clearance <= dist_to_path <= max_path_dist:
            ox.append(px)
            oy.append(py)

    ox = np.asarray(ox, dtype=float)
    oy = np.asarray(oy, dtype=float)

    print(f"Loaded {n} aligned samples")
    print(
        "Obstacle hits: "
        f"front={int(np.sum(hit_front))} "
        f"left={int(np.sum(hit_left))} "
        f"right={int(np.sum(hit_right))} "
        f"top={int(np.sum(hit_top))}"
    )
    if int(np.sum(hit_top)) > 0:
        print("Note: top.csv is read and counted, but top obstacles are not projected onto the 2D X-Y path plot.")

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(x, y, linewidth=1.8, label="Trajectory")
    ax.scatter([0.0], [0.0], marker="o", label="Start")
    ax.scatter([x[-1]], [y[-1]], marker="x", label="End")

    if len(ox):
        ax.scatter(ox, oy, marker="s", s=45, label="Obstacles")
    else:
        print("No obstacle points survived filtering. Try increasing max_path_dist or cap_dist.")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.6, alpha=0.6)

    allx = np.concatenate([x, ox]) if len(ox) else x
    ally = np.concatenate([y, oy]) if len(oy) else y

    padx = max(0.05, 0.15 * float(np.ptp(allx)))
    pady = max(0.05, 0.15 * float(np.ptp(ally)))

    ax.set_xlim(float(np.min(allx)) - padx, float(np.max(allx)) + padx)
    ax.set_ylim(float(np.min(ally)) - pady, float(np.max(ally)) + pady)

    ax.set_xlabel("Forward/Back (m)   (+ forward)")
    ax.set_ylabel("Right/Left (m)     (+ right)")
    ax.set_title("Drone Path with Obstacles")
    ax.legend(loc="best")

    out_pdf = rc_path.with_name(f"{rc_path.stem}-path-obstacles.pdf")
    plt.savefig(out_pdf, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved: {out_pdf}")


if __name__ == "__main__":
    main()