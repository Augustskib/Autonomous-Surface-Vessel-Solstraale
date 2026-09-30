#!/usr/bin/env python3
"""Motor burst like quick_motor_test.py, but reading the STATUS current
sense while commanding, to tell why a motor did not turn:

  current rises, motor turns   -> OK
  current rises, motor still   -> motor or its M+/M- leads
  current flat, motor still    -> enable pins, PWM pin or pin map
  current flat, WDT goes 0     -> firmware hears you, driver does not

Props off, motors clamped, throttle capped at 40%, always sends L:0 R:0 on
exit. The firmware watchdog is the real backstop.

    python3 motor_test_instrumented.py
    python3 motor_test_instrumented.py --throttle 25 --seconds 1.0
"""
import argparse
import statistics
import sys
import threading
import time
from collections import deque

try:
    import serial
except ImportError:
    sys.exit("pyserial missing. Run: pip install pyserial --break-system-packages")


class StatusReader(threading.Thread):
    """Read serial in the background, keeping only STATUS lines (IMU lines
    arrive at 50 Hz)."""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.samples = deque(maxlen=4000)   # (t, l_is, r_is, wdt)
        self.raw = deque(maxlen=400)
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode("ascii", "replace").strip()
            except Exception:
                return
            if not line.startswith("STATUS"):
                continue
            self.raw.append(line)
            l_is = r_is = wdt = None
            for tok in line.split():
                if tok.startswith("L_IS:"):
                    l_is = _toint(tok[5:])
                elif tok.startswith("R_IS:"):
                    r_is = _toint(tok[5:])
                elif tok.startswith("WDT:"):
                    wdt = _toint(tok[4:])
            if l_is is not None and r_is is not None:
                self.samples.append((time.time(), l_is, r_is, wdt))

    def stop(self):
        self._stop.set()

    def since(self, t0):
        return [s for s in self.samples if s[0] >= t0]


def _toint(s):
    try:
        return int(s)
    except ValueError:
        return None


def summarize(samples, label):
    if not samples:
        return f"  {label}: no STATUS lines received"
    l = [s[1] for s in samples]
    r = [s[2] for s in samples]
    w = [s[3] for s in samples if s[3] is not None]
    return (f"  {label}: n={len(samples)}  "
            f"L_IS min/mean/max = {min(l)}/{statistics.mean(l):.1f}/{max(l)}  "
            f"R_IS min/mean/max = {min(r)}/{statistics.mean(r):.1f}/{max(r)}  "
            f"WDT seen = {sorted(set(w)) if w else 'n/a'}")


def send_for(ser, reader, left, right, seconds):
    """Resend every 100 ms for the 500 ms firmware watchdog."""
    t0 = time.time()
    end = t0 + seconds
    while time.time() < end:
        ser.write(f"L:{left} R:{right}\n".encode())
        ser.flush()
        time.sleep(0.1)
    ser.write(b"L:0 R:0\n")
    ser.flush()
    return t0


def run_one(ser, reader, name, left, right, args):
    input(f"\n=== {name} ===\n"
          f"  {args.throttle}% for {args.seconds}s. Hands clear. Enter to run: ")

    base_t = time.time()
    time.sleep(1.2)                       # collect a fresh zero-current baseline
    baseline = reader.since(base_t)

    for i in (3, 2, 1):
        print(f"  {i}...")
        time.sleep(0.6)

    print("  RUNNING")
    t0 = send_for(ser, reader, left, right, args.seconds)
    burst = reader.since(t0)
    time.sleep(0.8)
    print("  stopped.")

    print(summarize(baseline, "baseline"))
    print(summarize(burst, "during  "))

    verdict = ""
    if baseline and burst:
        bl = statistics.mean([s[1] for s in baseline])
        br = statistics.mean([s[2] for s in baseline])
        dl = statistics.mean([s[1] for s in burst]) - bl
        dr = statistics.mean([s[2] for s in burst]) - br
        print(f"  delta: L_IS {dl:+.1f}   R_IS {dr:+.1f}")
        if max(abs(dl), abs(dr)) < 3:
            verdict = "NO CURRENT DETECTED - signal side or no pack voltage"
            print(f"  >> {verdict}")
        else:
            verdict = "current flowed"
            print(f"  >> {verdict}")

    spun = input("  Did it actually turn? [y/n] ").strip().lower() == "y"
    direction = ""
    if spun:
        direction = input("  Which way, looking at the shaft? [cw/ccw/unsure] ").strip()
    return {"spun": spun, "direction": direction, "current": verdict}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--throttle", type=int, default=25)
    p.add_argument("--seconds", type=float, default=1.0)
    args = p.parse_args()

    if args.throttle > 40:
        sys.exit("Refusing throttle above 40% for a bench test.")

    if input("Props OFF and motors clamped? Type NOPROPS: ") != "NOPROPS":
        sys.exit("Aborted.")

    print(f"\nOpening {args.port} @ {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {args.port}: {e}\n"
                 "Is arduino-cli monitor still running? Only one process "
                 "can hold the port. Close it and retry.")

    time.sleep(2.0)                       # board resets when the port opens
    ser.reset_input_buffer()

    reader = StatusReader(ser)
    reader.start()
    time.sleep(1.5)

    if not reader.samples:
        print("WARNING: no STATUS lines yet. Firmware may not be running.")
        if input("Continue anyway? [y/N] ").strip().lower() != "y":
            ser.close()
            sys.exit("Aborted.")
    else:
        print(f"Firmware talking. Last: {reader.raw[-1] if reader.raw else '?'}")

    results = {}
    try:
        results["LEFT forward"] = run_one(
            ser, reader, "LEFT forward", args.throttle, 0, args)
        results["RIGHT forward"] = run_one(
            ser, reader, "RIGHT forward", 0, args.throttle, args)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            ser.write(b"L:0 R:0\n")
            ser.flush()
            time.sleep(0.2)
        except Exception:
            pass
        reader.stop()
        ser.close()

    print("\n================ SUMMARY ================")
    for k, v in results.items():
        print(f"  {k:<16} spun={v['spun']}  dir={v['direction'] or '-':<6} "
              f"{v['current']}")
    print("=========================================")
    print("\nPaste this whole output back.")


if __name__ == "__main__":
    main()
