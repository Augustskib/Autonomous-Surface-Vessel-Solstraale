#!/usr/bin/env python3
"""Find out which physical motor is left and which is right.

Runs RIGHT alone, then LEFT alone, announcing each first. Repeat until you
are sure, then label the motors.

  RIGHT = driver on D9 / D10 / D7 / D8   (PIN_R_* in the firmware)
  LEFT  = driver on D5 / D6  / D2 / D4   (PIN_L_*)

Props off, motors clamped.

    python3 identify_motors.py
    python3 identify_motors.py --seconds 1.0 --throttle 40
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial --break-system-packages")


def drive(ser, left, right, seconds):
    """Hold a command for `seconds`, resending at 10 Hz for the watchdog."""
    end = time.time() + seconds
    while time.time() < end:
        ser.write(f"L:{left} R:{right}\n".encode())
        ser.flush()
        time.sleep(0.1)
    for _ in range(3):
        ser.write(b"L:0 R:0\n")
        ser.flush()
        time.sleep(0.05)


def countdown(label, detail):
    print()
    print("=" * 52)
    print(f"  {label}")
    print(f"  {detail}")
    print("=" * 52)
    for i in (3, 2, 1):
        print(f"    {i}...")
        time.sleep(0.7)
    print("    >>> RUNNING NOW <<<")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--throttle", type=int, default=40)
    p.add_argument("--seconds", type=float, default=1.5)
    p.add_argument("--gap", type=float, default=3.0,
                   help="quiet pause between the two motors")
    args = p.parse_args()

    if args.throttle > 60:
        sys.exit("Refusing throttle above 60% for bench identification.")

    if input("Props OFF and motors clamped? Type NOPROPS: ") != "NOPROPS":
        sys.exit("Aborted.")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {args.port}: {e}\n"
                 "Is hold.py or a monitor still running? Only one process "
                 "can hold the port.")

    time.sleep(2.0)                 # board resets when the port opens
    ser.reset_input_buffer()

    print(f"\nThrottle {args.throttle}%, {args.seconds}s each, "
          f"{args.gap}s gap between them.")

    try:
        while True:
            countdown("RIGHT MOTOR",
                      "driver on D9 / D10 / D7 / D8")
            drive(ser, 0, args.throttle, args.seconds)
            print("    stopped.")

            print(f"\n  ... {args.gap:.0f} second pause, nothing should move ...")
            time.sleep(args.gap)

            countdown("LEFT MOTOR",
                      "driver on D5 / D6 / D2 / D4")
            drive(ser, args.throttle, 0, args.seconds)
            print("    stopped.")

            print()
            print("-" * 52)
            print("  Which motor ran FIRST is your RIGHT.")
            print("  Which motor ran SECOND is your LEFT.")
            print("  If the SAME motor ran both times, or neither did,")
            print("  say so - that is a wiring fault, not a labelling one.")
            print("-" * 52)

            again = input("\nRun the sequence again? [y/N] ").strip().lower()
            if again != "y":
                break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            for _ in range(5):
                ser.write(b"L:0 R:0\n")
                ser.flush()
                time.sleep(0.05)
        except Exception:                     # noqa: BLE001
            pass
        ser.close()
        print("Motors commanded to zero, port closed.")

    print("\nGo put tape on them now, while you still remember which is which.")


if __name__ == "__main__":
    main()
