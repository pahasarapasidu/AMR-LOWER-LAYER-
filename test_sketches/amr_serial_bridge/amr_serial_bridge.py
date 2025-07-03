#!/usr/bin/env python3
"""
amr_serial_bridge.py  -  bidirectional console + motion sequencer
-----------------------------------------------------------------
  · Streams every line from the AMR controller to your terminal
  · Parses telemetry to track   emerg   and   profile_done   flags
  · Sends the next motion-profile CSV command only when the MCU
    is idle and safe to accept a new segment.

Author : Kiran Gunathilaka / Endeavor360
Date   : July 2025
"""

import argparse
import time
import sys
from collections import deque
from typing import Tuple, List

import serial                       # pip install pyserial
import serial.tools.list_ports      # for auto-detect (optional)

# ──────────────────────────── USER-TUNABLE QUEUE ─────────────────────────── #
# Edit or generate this list to build any path you like.
# (distance_mm, angle_deg, max_v_mm_s, max_w_dps,
#  final_v_mm_s, final_w_dps, lin_acc_mm_s2, ang_acc_dps2)
COMMAND_SEQUENCE: List[Tuple[float, float, int, int, int, int, float, float]] = [
    (5000.0,   0.0, 300,   0,   0,   0, 500.0,  0.0),
    (  0.0,  900.0,   0, 60,   0,   0,   0.0, 120.0),
    (700.0,   0.0, 350,   0,   0,   0, 600.0,  0.0),
    (  0.0, -900.0,   0, 60,   0,   0,   0.0, 120.0),
]

# ─────────────────────────────― ARGUMENT PARSER ―─────────────────────────── #
def find_default_port() -> str:
    """Pick the first port that looks like an Arduino/ATmega32U4."""
    for p in serial.tools.list_ports.comports():
        if "Arduino" in p.description or "ATmega32U4" in p.description:
            return p.device
    return "COM1"      # fallback; user can override

def get_args():
    ap = argparse.ArgumentParser(
        description="Serial console + motion sequencer for the AMR controller")
    ap.add_argument("port", nargs="?", default=find_default_port(),
                    help="Serial port (COMx or /dev/tty*)")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="Baud rate (default 115200)")
    ap.add_argument("--freq", type=float, default=100.0,
                    help="Command-send check rate in Hz (default 100)")
    ap.add_argument("--timeout", type=float, default=0.02,
                    help="Serial read timeout in seconds (default 0.02)")
    return ap.parse_args()

# ─────────────────────────────― HELPERS ―─────────────────────────────────── #
def build_command(cmd: Tuple[float, float, int, int, int, int, float, float]) -> str:
    """Format exactly as parse_jetson_auto() expects on the MCU."""
    return f"{cmd[0]},{cmd[1]},{cmd[2]},{cmd[3]},{cmd[4]},{cmd[5]},{cmd[6]},{cmd[7]}\n"

def parse_telemetry(line: str):
    """
    MCU sends 12 space-separated fields; last two are emerg and profileDone.
    Returns dict or None on malformed packet.
    """
    parts = line.strip().split()
    if len(parts) != 12:
        return None
    try:
        emerg        = int(parts[10])
        profile_done = int(parts[11])
        return {"emerg": emerg, "profile_done": profile_done}
    except ValueError:
        return None

# ─────────────────────────────― MAIN LOOP ―──────────────────────────────── #
def main() -> None:
    args = get_args()
    cmd_queue = deque(COMMAND_SEQUENCE)

    try:
        with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
            print(f"[INFO] Connected to {ser.port} @ {ser.baudrate} baud "
                  f"(polling {args.freq:.1f} Hz) — Ctrl-C to quit")
            next_tx = time.perf_counter()
            outstanding = False  # True after we’ve sent a cmd, until MCU finishes

            while True:
                # 1 ─ Read and print every available line
                while ser.in_waiting:
                    raw = ser.readline()
                    if not raw:
                        break
                    text = raw.decode("ascii", errors="replace").rstrip()
                    print(f"< {text}")   # echo to console

                    pkt = parse_telemetry(text)
                    if pkt:
                        if pkt["emerg"]:
                            print("[WARN] Emergency stop asserted -- clearing queue")
                            cmd_queue.clear()
                            outstanding = False

                        elif pkt["profile_done"] and outstanding:
                            outstanding = False
                            print("[INFO] MCU reports profile complete")

                # 2 ─ Periodic transmit check
                now = time.perf_counter()
                if now >= next_tx:
                    next_tx += 1.0 / args.freq

                    if not outstanding and cmd_queue:
                        cmd = cmd_queue.popleft()
                        ser.write(build_command(cmd).encode())
                        ser.flush()
                        outstanding = True
                        print(f"> {cmd}")

                # 3 ─ Tiny sleep to ease CPU load
                time.sleep(0.001)

    except serial.SerialException as e:
        sys.exit(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\n[INFO] User aborted — exiting.")

# ─────────────────────────────────────────────────────────────────────────── #
if __name__ == "__main__":
    main()
