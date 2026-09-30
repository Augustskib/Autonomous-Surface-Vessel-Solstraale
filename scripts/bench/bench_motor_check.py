#!/usr/bin/env python3
"""Short, bounded motor test. Props off, motors secured, hand on the cutoff.

    python3 bench_motor_check.py --side both --seconds 3 --throttle 15

Needs the firmware that prints STATUS L:/R:/WDT: lines.
"""
import argparse
import math
import time
import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right', 'both'), required=True)
    parser.add_argument('--seconds', type=float, default=3.0)
    parser.add_argument('--throttle', type=int, default=15)
    parser.add_argument('--port', default='/dev/ttyACM0')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 3:
        parser.error('seconds must be greater than zero and at most 3')
    if not 0 < args.throttle <= 25:
        parser.error('throttle must be between 1 and 25')

    with serial.Serial(args.port, 115200, timeout=0.02,
                       write_timeout=0.1, exclusive=True) as port:
        rx = bytearray()
        latest = {}

        def drain():
            rx.extend(port.read(min(port.in_waiting, 4096) or 1))
            while b'\n' in rx:
                raw, _, rest = rx.partition(b'\n')
                rx[:] = rest
                line = raw.decode('ascii', 'replace').strip()
                if line.startswith('STATUS '):
                    latest.clear()
                    for token in line.split()[1:]:
                        key, sep, value = token.partition(':')
                        if sep:
                            latest[key] = value
                    print(line, flush=True)
            if len(rx) > 4096:
                raise RuntimeError('Invalid oversized telemetry line')

        def hold(left, right, seconds):
            end = time.monotonic() + seconds
            next_send = 0
            while time.monotonic() < end:
                if time.monotonic() >= next_send:
                    port.write(f'L:{left} R:{right}\n'.encode('ascii'))
                    next_send = time.monotonic() + 0.05
                drain()

        def stop():
            for _ in range(5):
                port.write(b'L:0 R:0\n')
                drain()
                time.sleep(0.02)
            latest.clear()
            end = time.monotonic() + 1.5
            while time.monotonic() < end:
                drain()
            if not all(latest.get(k) == v for k, v in
                       (('L', '0'), ('R', '0'), ('WDT', '1'))):
                raise RuntimeError('STOP NOT CONFIRMED: disconnect battery now')

        try:
            hold(0, 0, 1)
            stop()
            sides = ('left', 'right') if args.side == 'both' else (args.side,)
            for side in sides:
                print(f'{side.upper()} in 3 seconds. Hands clear.', flush=True)
                hold(0, 0, 3)
                print(f'{side.upper()} NOW: {args.throttle}% for {args.seconds}s', flush=True)
                hold(args.throttle if side == 'left' else 0,
                     args.throttle if side == 'right' else 0, args.seconds)
                stop()
                print(f'{side.upper()} command stopped.', flush=True)
        finally:
            stop()
            print('DONE: zero commands and watchdog stop confirmed.', flush=True)


if __name__ == '__main__':
    main()
