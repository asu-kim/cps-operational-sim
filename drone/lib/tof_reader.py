#!/usr/bin/env python3
# Prints one float (meters) per line at the requested rate.

import argparse
import sys
import time
import traceback

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", type=int, default=22)
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x29)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--mode", type=int, default=3, choices=[1,2,3])
    ap.add_argument("--timing_ms", type=int, default=100)
    ap.add_argument("--max_m", type=float, default=4.0)
    args = ap.parse_args()

    try:
        # Your working import
        from VL53L1X import VL53L1X
    except Exception:
        traceback.print_exc()
        print("ERR: cannot import VL53L1X", file=sys.stderr)
        sys.exit(2)

    try:
        sensor = VL53L1X(i2c_bus=args.bus, i2c_address=args.addr)
        if hasattr(sensor, "open"):
            sensor.open()
        # some builds print device info on start_ranging
        sensor.start_ranging(args.mode)
        if hasattr(sensor, "set_timing_budget"):
            sensor.set_timing_budget(args.timing_ms)

        period = 1.0 / max(1.0, args.rate)
        while True:
            d_mm = sensor.get_distance()
            if d_mm and d_mm > 0:
                d_m = d_mm / 1000.0
                if d_m > args.max_m:
                    d_m = args.max_m
                print(f"{d_m:.3f}", flush=True)
            time.sleep(period)

    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
        sys.exit(3)
    finally:
        try:
            if "sensor" in locals() and hasattr(sensor, "stop_ranging"):
                sensor.stop_ranging()
        except Exception:
            pass

if __name__ == "__main__":
    main()
