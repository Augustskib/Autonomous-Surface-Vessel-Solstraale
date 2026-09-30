#!/usr/bin/env python3
"""Minimal motor test: spin each motor for about a second over serial.

No ROS, IMU or GPS. Needs:
  - boat_motor_controller.ino flashed (the Arduino_Modulino library must be
    installed to compile it, even without the IMU)
  - props off, motors clamped
  - LiPo through its fuse, LiPo negative tied to Arduino GND
  - a way to cut power that is not this script

    python3 quick_motor_test.py
    python3 quick_motor_test.py --port /dev/ttyACM0 --throttle 25 --seconds 1.0
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed. Run: pip install pyserial --break-system-packages")


def send_for(ser, left, right, seconds):
    """Resend the command every 100 ms for `seconds` (500 ms watchdog)."""
    end = time.time() + seconds
    while time.time() < end:
        ser.write(f"L:{left} R:{right}\n".encode())
        time.sleep(0.1)
    ser.write(b"L:0 R:0\n")
    time.sleep(0.1)


def run_motor(ser, name, left, right, throttle, seconds):
    input(f"\n{name}: about to spin at {throttle}% for {seconds}s. "
          f"Press Enter when clear of the prop shaft...")
    print(f"  -> running {name}...")
    send_for(ser, left, right, seconds)
    print("  -> stopped.")
    ok = input(f"  Did {name} spin the way you expected? [y/n] ").strip().lower()
    return ok == "y"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--throttle", type=int, default=25,
                    help="percent, kept low on purpose for a first test")
    p.add_argument("--seconds", type=float, default=1.0)
    args = p.parse_args()

    if args.throttle > 40:
        sys.exit("Refusing throttle above 40%% for a first test. Lower it.")

    confirm = input("Props OFF and motors secured? Type NOPROPS to continue: ")
    if confirm != "NOPROPS":
        sys.exit("Aborted.")

    print(f"Opening {args.port} @ {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Couldn't open {args.port}: {e}\n"
                  f"Check: ls -l /dev/ttyACM* -- is the board there, "
                  f"and is the USB cable a data cable (not charge-only)?")
    time.sleep(2)  # the Arduino resets when the port opens
    ser.reset_input_buffer()

    results = {}
    try:
        results["LEFT forward"] = run_motor(
            ser, "LEFT forward", args.throttle, 0, args.throttle, args.seconds)
        results["RIGHT forward"] = run_motor(
            ser, "RIGHT forward", 0, args.throttle, args.throttle, args.seconds)
    finally:
        ser.write(b"L:0 R:0\n")
        ser.close()

    print("\n--- Summary ---")
    for k, v in results.items():
        print(f"  {k}: {'OK' if v else 'CHECK WIRING'}")

    if not all(results.values()):
        print("\nSomething didn't match. Check in order: "
              "fuse/LiPo voltage -> ground bond -> EN pins "
              "-> PWM pin -> motor leads.")
    else:
        print("\nBoth motors spin on command. Next: hardware_bringup.py "
              "--stage 2 for reverse + both-together + the current-sense "
              "readout, then --stage 3 for the watchdog/cable-pull test "
              "before adding anything else.")


if __name__ == "__main__":
    main()
