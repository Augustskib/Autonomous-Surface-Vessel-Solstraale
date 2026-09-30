#!/usr/bin/env python3
"""Hold one thrust command until Ctrl-C, so you can measure with a meter.

Resends every 100 ms to keep the 500 ms firmware watchdog fed, and prints
the STATUS lines. Props off, motors clamped.

    python3 hold.py --left 40
    python3 hold.py --right 40
    python3 hold.py --left 40 --right 40
    python3 hold.py --left 100

Full duty for a few seconds is fine with props off. If a motor only moves
at 100%, that is a dead zone and needs a minimum duty in firmware.
"""
import argparse
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial --break-system-packages")


def reader_loop(ser, stop_evt):
    """Print STATUS lines. Skips the 50Hz IMU flood."""
    while not stop_evt.is_set():
        try:
            line = ser.readline().decode("ascii", "replace").strip()
        except Exception as exc:                      # noqa: BLE001
            print(f"\n[reader died: {type(exc).__name__}: {exc}]")
            return
        if line.startswith("STATUS"):
            print(f"  {line}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--left", type=int, default=0)
    p.add_argument("--right", type=int, default=0)
    args = p.parse_args()

    for v in (args.left, args.right):
        if not -100 <= v <= 100:
            sys.exit("Thrust must be between -100 and 100.")

    if input("Props OFF and motors clamped? Type NOPROPS: ") != "NOPROPS":
        sys.exit("Aborted.")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {args.port}: {e}\n"
                 "Is a monitor still running? Only one process can hold it.")

    time.sleep(2.0)               # board resets when the port opens
    ser.reset_input_buffer()

    stop_evt = threading.Event()
    t = threading.Thread(target=reader_loop, args=(ser, stop_evt), daemon=True)
    t.start()

    cmd = f"L:{args.left} R:{args.right}\n".encode()
    print(f"\nHolding  L:{args.left}  R:{args.right}")
    print("Resending every 100ms. Ctrl-C to stop.\n")
    print("Probe now. Useful points, black probe on Arduino GND:")
    print("    driver M+ to M-     driver output, expect duty x pack volts")
    print("    driver R_EN         expect ~5V while this is running")
    print("    driver L_EN         expect ~5V while this is running")
    print("    driver RPWM         a PWM average, not a clean 5V")
    print()

    try:
        while True:
            ser.write(cmd)
            ser.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        try:
            for _ in range(5):
                ser.write(b"L:0 R:0\n")
                ser.flush()
                time.sleep(0.05)
        except Exception:                              # noqa: BLE001
            pass
        stop_evt.set()
        time.sleep(0.2)
        ser.close()
        print("Motors commanded to zero, port closed.")


if __name__ == "__main__":
    main()
