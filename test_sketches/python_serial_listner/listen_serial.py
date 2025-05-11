#!/usr/bin/env python3
"""
listen_serial.py – stream text from a chosen COM/tty port

Usage examples
--------------
Windows:  python listen_serial.py COM5           # default 9600 baud
Linux  :  python listen_serial.py /dev/ttyACM0 -b 115200
"""

import argparse
import sys
import serial        # pip install pyserial

def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Read text lines from a serial port and echo to stdout")
    ap.add_argument("port", help="COMx or /dev/tty… device name")
    ap.add_argument("-b", "--baud", type=int, default=9600,
                    help="Baud rate (default 9600)")
    ap.add_argument("--timeout", type=float, default=1.0,
                    help="Read timeout in seconds (default 1.0)")
    return ap.parse_args()

def main() -> None:
    args = parse_args()

    try:
        with serial.Serial(args.port, args.baud,
                           timeout=args.timeout) as ser:
            print(f"Listening on {ser.port} @ {ser.baudrate} baud "
                  "(Ctrl-C to quit)")
            while True:
                try:
                    line = ser.readline()        # blocks ≤ timeout seconds
                    if line:                     # got something?
                        print(line.decode("ascii", errors="replace").rstrip())
                except KeyboardInterrupt:
                    break
    except serial.SerialException as e:
        sys.exit(f"Serial error: {e}")

if __name__ == "__main__":
    main()
