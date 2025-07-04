#!/usr/bin/env python3
"""
Jetson-side motion sequencer for Endeavor360's AMR controller.
Sends profile commands only when the MCU reports profile_done==1.
Author: Kiran Gunathilaka (July-2025)
"""

import serial
import time
from collections import deque
from typing import Tuple, List

# ───────────────────────── CONFIGURABLE PARAMETERS ────────────────────────── #
PORT      = "COM6"             # adjust if needed
BAUD      = 9600
POLL_HZ   = 100                        # 10 ms loop
TIMEOUT   = 0.02                       # serial read timeout (s)

# ────────────────────────── PROFILE COMMAND SEQUENCE ──────────────────────── #
# Each tuple: (distance_mm, angle_deg,
#              max_vel_mm_s, max_omega_dps,
#              final_vel_mm_s, final_omega_dps,
#              lin_acc_mm_s2,  ang_acc_dps2)
#
# Edit / extend this list to build any path you like.
CMD_SEQUENCE: List[Tuple[float, float, int, int, int, int, float, float]] = [
    ( 5000.0,   0.0, 300,   0,   0,   0, 500.0,  0.0),  # drive 0.5 m
    (   0.0,  500.0,   0, 120,   0,   0,   0.0, 360.0), # turn 90 °
    ( 700.0,   0.0, 350,   0,   0,   0, 600.0,  0.0),
    (   0.0, -90.0,   0, 120,   0,   0,   0.0, 360.0),
]
command_q = deque(CMD_SEQUENCE)

# ───────────────────────────────── HELPERS ────────────────────────────────── #
def build_command(cmd: Tuple[float, float, int, int, int, int, float, float]) -> str:
    """
    Format exactly as the AVR expects:
    distance,angle,max_v,max_w,last_v,last_w,lin_acc,ang_acc\n
    """
    return f"{cmd[0]},{cmd[1]},{cmd[2]},{cmd[3]},{cmd[4]},{cmd[5]},{cmd[6]},{cmd[7]}\n"

def parse_telemetry(line: str):
    """
    Expect 12 space-separated fields (see send_telemetry()).
    Return dict or None on bad packet.
    """
    parts = line.strip().split()
    if len(parts) != 12:
        return None
    try:
        emerg         = int(parts[10])
        profile_done  = int(parts[11])
        return {"emerg": emerg, "profile_done": profile_done}
    except ValueError:
        return None

# ───────────────────────────────── MAIN ───────────────────────────────────── #
def main() -> None:
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    print(f"[INFO] Connected to {PORT} @ {BAUD} baud")

    next_tx_time = time.perf_counter()
    outstanding = False  # True after we’ve sent a command, until MCU finishes
    try:
        while True:
            # ── 1 · Read any pending telemetry line ──────────────────────────
            if ser.in_waiting:
                raw = ser.readline().decode(errors="ignore")
                pkt = parse_telemetry(raw)
                if pkt:
                    # Emergency → halt all comms
                    if pkt["emerg"]:
                        print("[WARN] Emergency stop asserted – aborting mission")
                        command_q.clear()
                        outstanding = False

                    # MCU finished previous profile
                    elif pkt["profile_done"] and outstanding:
                        outstanding = False
                        print("✔ MCU reports profile complete")

            # ── 2 · Time-driven transmission check (every 10 ms) ─────────────
            now = time.perf_counter()
            if now >= next_tx_time:
                next_tx_time += 1.0 / POLL_HZ

                if not outstanding and command_q:
                    cmd = command_q.popleft()
                    ser.write(build_command(cmd).encode())
                    ser.flush()
                    outstanding = True
                    print(f"→ Sent cmd: {cmd}")

            # ── 3 · CPU-friendly sleep ───────────────────────────────────────
            time.sleep(0.001)  # 1 ms granularity is fine

    except KeyboardInterrupt:
        print("\n[INFO] User aborted; closing port.")
    finally:
        ser.close()

# ──────────────────────────────────────────────────────────────────────────── #
if __name__ == "__main__":
    main()
