#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import datetime as dt
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Tuple, List, Optional

try:
    from zoneinfo import ZoneInfo
except Exception:
    ZoneInfo = None  # type: ignore

PHX_TZ_NAME = "America/Phoenix"

# CSV column order you asked for
CSV_ORDER = ["front", "bottom", "left", "right", "top"]


@dataclass
class SensorCfg:
    name: str
    bus: int
    addr: int
    mode: int
    rate: float
    timing_ms: int
    max_m: float


def now_local_filename(prefix: str = "tof", ext: str = ".csv") -> str:
    if ZoneInfo is not None:
        t = dt.datetime.now(tz=ZoneInfo(PHX_TZ_NAME))
    else:
        t = dt.datetime.now()
    return f"{prefix}_{t.strftime('%Y%m%d_%H%M%S')}{ext}"


def ensure_parent_dir(path: str) -> None:
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.exists(parent):
        os.makedirs(parent, exist_ok=True)


def parse_sensor_arg(s: str, defaults: dict) -> SensorCfg:
    """
    Format:
      name,bus=24,addr=0x29,mode=1,rate=20,timing_ms=100,max_m=4.0
    Only name is required. Everything else falls back to defaults.
    """
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if not parts:
        raise ValueError("Empty --sensor entry")

    name = parts[0]
    kv = {}
    for p in parts[1:]:
        if "=" not in p:
            raise ValueError(f"Bad sensor field '{p}'. Use key=value.")
        k, v = p.split("=", 1)
        kv[k.strip()] = v.strip()

    bus = int(kv.get("bus", defaults["bus"]))
    addr_raw = kv.get("addr", defaults["addr"])
    addr = int(addr_raw, 0) if isinstance(addr_raw, str) else int(addr_raw)
    mode = int(kv.get("mode", defaults["mode"]))
    rate = float(kv.get("rate", defaults["rate"]))
    timing_ms = int(kv.get("timing_ms", defaults["timing_ms"]))
    max_m = float(kv.get("max_m", defaults["max_m"]))

    return SensorCfg(name=name, bus=bus, addr=addr, mode=mode, rate=rate, timing_ms=timing_ms, max_m=max_m)


def default_sensor_set(rate: float, timing_ms: int, max_m: float) -> List[SensorCfg]:
    # Your exact mapping (same addr 0x29 on different buses)
    mapping = [
        ("bottom", 22, 0x29, 1),
        ("left",   23, 0x29, 1),
        ("front",  24, 0x29, 1),
        ("top",    25, 0x29, 1),
        ("right",  26, 0x29, 1),
    ]
    return [
        SensorCfg(name=n, bus=b, addr=a, mode=m, rate=rate, timing_ms=timing_ms, max_m=max_m)
        for (n, b, a, m) in mapping
    ]


def read_lines_to_queue(
    proc: subprocess.Popen,
    sensor_name: str,
    out_q: "queue.Queue[Tuple[str, float]]",
    stop_evt: threading.Event,
) -> None:
    """
    Expects one float (meters) per line from tof_reader.py stdout.
    Pushes (sensor_name, distance_m).
    """
    assert proc.stdout is not None
    while not stop_evt.is_set():
        line = proc.stdout.readline()
        if not line:
            break
        line = line.strip()
        if not line:
            continue
        try:
            d_m = float(line)
        except ValueError:
            continue
        out_q.put((sensor_name, d_m))


def obstacle_direction_within(latest_vals: Dict[str, float], threshold_m: float) -> str:
    # Closest among the five, if <= threshold
    candidates: List[Tuple[float, str]] = []
    for name in CSV_ORDER:
        if name in latest_vals:
            candidates.append((latest_vals[name], name))
    if not candidates:
        return "none"
    d_min, name_min = min(candidates, key=lambda x: x[0])
    return name_min if d_min <= threshold_m else "none"


def main() -> int:
    ap = argparse.ArgumentParser()

    ap.add_argument("--reader", default="./tof_reader.py", help="Path to your tof_reader.py")
    ap.add_argument("--log", default=None, help="CSV path. Default: tof_YYYYMMDD_HHMMSS.csv")
    ap.add_argument("--print", dest="do_print", action="store_true", help="Print live values")

    ap.add_argument("--sensor", action="append", default=[],
                    help="Repeatable sensor spec: name,bus=24,addr=0x29,mode=1,rate=20,timing_ms=100,max_m=4.0")

    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--timing_ms", type=int, default=100)
    ap.add_argument("--max_m", type=float, default=4.0)

    ap.add_argument("--threshold_m", type=float, default=0.20,
                    help="Obstacle threshold in meters (default 0.20)")

    args = ap.parse_args()

    log_path = args.log if args.log else now_local_filename("tof", ".csv")
    ensure_parent_dir(log_path)

    stop_evt = threading.Event()
    out_q: "queue.Queue[Tuple[str, float]]" = queue.Queue()
    procs: List[subprocess.Popen] = []
    threads: List[threading.Thread] = []

    latest_vals: Dict[str, float] = {}

    def shutdown() -> None:
        stop_evt.set()
        for p in procs:
            try:
                p.terminate()
            except Exception:
                pass

    def on_signal(signum, frame):  # noqa: ANN001
        shutdown()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # Decide sensors:
    # - If --sensor provided, use only those
    # - Else use your default mapping (all five)
    if args.sensor:
        defaults = {"bus": 24, "addr": 0x29, "mode": 1, "rate": args.rate, "timing_ms": args.timing_ms, "max_m": args.max_m}
        sensor_cfgs = [parse_sensor_arg(s, defaults) for s in args.sensor]
    else:
        sensor_cfgs = default_sensor_set(rate=args.rate, timing_ms=args.timing_ms, max_m=args.max_m)

    # Launch tof_reader.py per sensor
    for cfg in sensor_cfgs:
        err_path = f"tof_reader_{cfg.name}.err"
        err_f = open(err_path, "w", buffering=1)

        cmd = [
            sys.executable,
            args.reader,
            "--bus", str(cfg.bus),
            "--addr", hex(cfg.addr),
            "--rate", str(cfg.rate),
            "--mode", str(cfg.mode),
            "--timing_ms", str(cfg.timing_ms),
            "--max_m", str(cfg.max_m),
        ]

        p = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=err_f,      # IMPORTANT: keep errors per-sensor
            text=True,
            bufsize=1,
        )
        procs.append(p)

        t = threading.Thread(target=read_lines_to_queue, args=(p, cfg.name, out_q, stop_evt), daemon=True)
        t.start()
        threads.append(t)

    # Write CSV
    new_file = not os.path.exists(log_path) or os.path.getsize(log_path) == 0
    with open(log_path, "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow([
                "front_m",
                "bottom_m",
                "left_m",
                "right_m",
                "top_m",
                "obstacle_within_0p20m_direction",
            ])
            f.flush()

        try:
            while not stop_evt.is_set():
                try:
                    name, d_m = out_q.get(timeout=0.25)
                except queue.Empty:
                    continue

                latest_vals[name] = d_m
                direction = obstacle_direction_within(latest_vals, args.threshold_m)

                row = []
                for sensor in CSV_ORDER:
                    row.append(f"{latest_vals[sensor]:.3f}" if sensor in latest_vals else "")
                row.append(direction)

                w.writerow(row)
                f.flush()

                if args.do_print:
                    s = " ".join([f"{k}={latest_vals[k]:.3f}" if k in latest_vals else f"{k}=NA" for k in CSV_ORDER])
                    print(f"{s} | obstacle<=0.20m: {direction}", flush=True)

        except KeyboardInterrupt:
            shutdown()
        finally:
            shutdown()
            for p in procs:
                try:
                    p.wait(timeout=1.0)
                except Exception:
                    try:
                        p.kill()
                    except Exception:
                        pass

    if args.do_print:
        print(f"saved: {log_path}", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
