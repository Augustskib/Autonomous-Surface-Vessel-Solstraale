#!/usr/bin/env python3
"""Interactive hardware bring-up and bench test for the ASV boat.

    python3 scripts/bench/hardware_bringup.py            # menu
    python3 scripts/bench/hardware_bringup.py --all      # every stage
    python3 scripts/bench/hardware_bringup.py --stage 2  # one stage
    python3 scripts/bench/hardware_bringup.py --list

Run on the Pi. Most stages need only Python 3 and pyserial, so the wiring
can be checked before the ROS workspace builds. Stages 8 and 9 need a
sourced ROS 2 workspace and skip if it is missing.


READ THIS BEFORE THE MOTOR STAGES
=================================
  1. No propellers. The tool asks you to type NOPROPS.
  2. Motors clamped to the bench or hull. Never hand-held.
  3. Fuse in line on the battery positive before any power reaches a
     motor driver.
  4. A way to cut power fast that is not this script.
  5. Short bursts. The motor ratings are unclear for a 3S pack, so the tool
     caps itself at 30% and 2 seconds per burst.
  6. Fire extinguisher or sand bucket near the LiPo.

A stop is always sent on exit, including Ctrl-C and exceptions. The
Arduino watchdog is the real protection (stage 3 checks it).
"""

import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

# ----------------------------------------------------------------------
# Configuration. Mirrors the firmware and ROS nodes; keep them in sync.
# ----------------------------------------------------------------------

# motor_driver.py's 'port' default, and the Arduino's native USB-serial.
ARDUINO_PORT_CANDIDATES = ['/dev/ttyACM0', '/dev/ttyACM1',
                           '/dev/ttyUSB0', '/dev/ttyUSB1']
# Must match SERIAL_BAUD in boat_motor_controller.ino.
ARDUINO_BAUD = 115200

# gps_publisher defaults. GY-NEO6MV2: 9600 8N1, 1 Hz.
GPS_PORT_CANDIDATES = ['/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0',
                       '/dev/ttyUSB0']
GPS_BAUD = 9600

# power_monitor DEFAULT_CHANNELS. Not verified against the board jumpers,
# stage 7 checks.
INA226_EXPECTED = {0x40: 'motor_left', 0x41: 'motor_right', 0x44: 'battery'}

# Safety caps for the motor stages, deliberately not command line options.
MAX_TEST_THROTTLE_PCT = 30
BURST_SECONDS = 2.0

# The Arduino's own watchdog, from boat_motor_controller.ino.
FIRMWARE_WATCHDOG_MS = 500

BOLD = '\033[1m'
DIM = '\033[2m'
GRN = '\033[32m'
RED = '\033[31m'
YEL = '\033[33m'
CYN = '\033[36m'
RST = '\033[0m'

if not sys.stdout.isatty() or os.environ.get('NO_COLOR'):
    BOLD = DIM = GRN = RED = YEL = CYN = RST = ''


# ----------------------------------------------------------------------
# Small output helpers
# ----------------------------------------------------------------------

def hdr(text):
    print('\n%s%s%s' % (BOLD, '=' * 70, RST))
    print('%s %s%s' % (BOLD, text, RST))
    print('%s%s%s' % (BOLD, '=' * 70, RST))


def ok(text):
    print('  %s[ OK ]%s %s' % (GRN, RST, text))


def bad(text):
    print('  %s[FAIL]%s %s' % (RED, RST, text))


def warn(text):
    print('  %s[WARN]%s %s' % (YEL, RST, text))


def info(text):
    print('  %s[ .. ]%s %s' % (DIM, RST, text))


def note(text):
    for line in text.strip().splitlines():
        print('        %s%s%s' % (DIM, line.strip(), RST))


def ask(prompt, default=None):
    suffix = ' [%s]' % default if default else ''
    try:
        answer = input('  %s?%s %s%s: ' % (CYN, RST, prompt, suffix)).strip()
    except EOFError:
        return default or ''
    return answer or (default or '')


def confirm(prompt):
    return ask('%s (y/N)' % prompt, 'n').lower().startswith('y')


class Result:
    """Accumulates pass/fail across stages so the summary is honest."""

    def __init__(self):
        self.rows = []

    def add(self, stage, name, status, detail=''):
        self.rows.append((stage, name, status, detail))

    def summary(self):
        hdr('SUMMARY')
        if not self.rows:
            print('  nothing was run')
            return 0
        width = max(len(r[1]) for r in self.rows)
        fails = 0
        for stage, name, status, detail in self.rows:
            colour = {'PASS': GRN, 'FAIL': RED, 'WARN': YEL,
                      'SKIP': DIM}.get(status, '')
            if status == 'FAIL':
                fails += 1
            print('  %s%-4s %-*s  %s%-4s%s  %s'
                  % (DIM, stage, width, name, colour, status, RST, detail))
        print()
        if fails:
            print('  %s%d check(s) FAILED.%s Do not put the boat in water '
                  'until these are resolved.' % (RED, fails, RST))
        else:
            print('  %sNo failures.%s Note that PASS here means "the '
                  'electronics respond as expected on a bench", which is '
                  'not the same as "the boat is ready".' % (GRN, RST))
        return 1 if fails else 0


RESULT = Result()


# ----------------------------------------------------------------------
# Optional imports, handled rather than assumed
# ----------------------------------------------------------------------

def try_import(name, pip_hint):
    try:
        return __import__(name)
    except ImportError:
        warn('python module %r not installed' % name)
        note('install it with:  %s' % pip_hint)
        return None


def have_ros():
    """True if the ros2 CLI is on PATH (what the ROS stages use)."""
    return shutil.which('ros2') is not None


def run(cmd, timeout=10):
    """Run a command, return (rc, stdout+stderr). Never raises."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True,
                           timeout=timeout)
        return p.returncode, (p.stdout or '') + (p.stderr or '')
    except FileNotFoundError:
        return 127, 'command not found: %s' % cmd[0]
    except subprocess.TimeoutExpired:
        return 124, 'timed out after %ss' % timeout
    except OSError as exc:
        return 1, str(exc)


def find_port(candidates, label):
    """First existing device from a candidate list, plus what else is there."""
    present = [p for p in candidates if os.path.exists(p)]
    everything = sorted(set(glob.glob('/dev/tty[AU]*') + glob.glob('/dev/serial*')))
    if present:
        return present[0], everything
    return None, everything


# ======================================================================
# STAGE 0: what is plugged in
# ======================================================================

def stage0_inventory():
    hdr('STAGE 0  Inventory: what does the Pi actually see?')
    note("""
        This stage touches nothing and changes nothing. It answers "is the
        hardware visible to the operating system at all", which is the
        question you want answered before you start suspecting your code.
        """)

    # --- host ---------------------------------------------------------
    model = Path('/proc/device-tree/model')
    if model.exists():
        text = model.read_text(errors='ignore').strip('\x00').strip()
        ok('host: %s' % text)
        if 'Raspberry Pi 4' not in text:
            warn('this was written against a Pi 4 -- some paths may differ')
    else:
        warn('not a Raspberry Pi (no /proc/device-tree/model)')
        note('the serial/I2C/camera stages will almost certainly fail here; '
             'that is expected on a laptop')

    # --- temperature and throttling -----------------------------------
    zone = Path('/sys/class/thermal/thermal_zone0/temp')
    if zone.exists():
        try:
            temp = int(zone.read_text().strip()) / 1000.0
            (ok if temp < 70 else warn)('CPU temperature: %.1f C' % temp)
        except (OSError, ValueError):
            warn('could not read CPU temperature')

    if shutil.which('vcgencmd'):
        rc, out = run(['vcgencmd', 'get_throttled'])
        if rc == 0:
            raw = out.strip().split('=')[-1]
            try:
                value = int(raw, 16)
            except ValueError:
                value = 0
            if value == 0:
                ok('no throttling or under-voltage flags (%s)' % raw)
            else:
                bad('throttle/under-voltage flags set: %s' % raw)
                note("""
                    Bit 0/16 = under-voltage. THIS MATTERS MORE THAN IT LOOKS.
                    An under-volted Pi corrupts SD cards, drops camera frames
                    and kills nodes at random, and every one of those looks
                    like a software bug. If you see this, fix the supply
                    before debugging anything else.
                    """)
            RESULT.add(0, 'pi power/throttle', 'PASS' if value == 0 else 'FAIL',
                       raw)

    # --- disk ---------------------------------------------------------
    try:
        usage = shutil.disk_usage(str(Path.home()))
        free_gb = usage.free / 1e9
        (ok if free_gb > 20 else warn)(
            'free disk: %.1f GB (~%.0f min of imagery at ~7 GB/h)'
            % (free_gb, free_gb / 7.0 * 60))
        if free_gb < 20:
            note('keep at least 20 GB free before a run; '
                 'a 32GB SD card is tight for camera-heavy '
                 'work')
    except OSError:
        warn('could not stat the filesystem')

    # --- serial devices -----------------------------------------------
    arduino, all_tty = find_port(ARDUINO_PORT_CANDIDATES, 'Arduino')
    print()
    info('serial devices present: %s' % (', '.join(all_tty) or 'none'))
    if arduino:
        ok('Arduino (USB-serial) found at %s' % arduino)
        RESULT.add(0, 'arduino present', 'PASS', arduino)
    else:
        bad('no Arduino USB-serial device found')
        note("""
            Checked: %s
            Things to try, in order:
              * is the USB cable a DATA cable, not a charge-only one?
                This wastes more afternoons than any other single cause.
              * `lsusb` -- does the board appear at all?
              * `dmesg | tail -20` right after plugging it in
              * the UNO R4 WiFi should enumerate as ttyACM0; the parts log
                flags "confirm it still does" as an open question
            """ % ', '.join(ARDUINO_PORT_CANDIDATES))
        RESULT.add(0, 'arduino present', 'FAIL', 'no device')

    gps, _ = find_port(GPS_PORT_CANDIDATES, 'GPS')
    if gps:
        ok('GPS serial port present at %s' % gps)
        RESULT.add(0, 'gps port present', 'PASS', gps)
    else:
        warn('no GPS serial port found (%s)' % ', '.join(GPS_PORT_CANDIDATES))
        note("""
            /dev/serial0 only exists once the UART is enabled. On a Pi 4:
              sudo raspi-config -> Interface Options -> Serial Port
                 -> login shell over serial: NO
                 -> serial hardware enabled: YES
            then reboot. Both answers matter: leaving the login shell
            enabled means getty fights your GPS for the port.
            """)
        RESULT.add(0, 'gps port present', 'WARN', 'not found')

    # --- I2C ----------------------------------------------------------
    i2c_devs = sorted(glob.glob('/dev/i2c-*'))
    if i2c_devs:
        ok('I2C buses: %s' % ', '.join(i2c_devs))
    else:
        warn('no /dev/i2c-* -- enable I2C via raspi-config')
    note('NOTE: on this boat the IMU is on the ARDUINO\'s Qwiic port, not '
         'the Pi\'s I2C bus. The Pi\'s I2C is for the INA226 '
         'power monitors only.')

    # --- camera -------------------------------------------------------
    print()
    if shutil.which('rpicam-hello') or shutil.which('libcamera-hello'):
        ok('rpicam/libcamera tools installed')
    else:
        warn('rpicam-hello / libcamera-hello not found')
        note('camera_publisher.py shells out to rpicam-vid; without it the '
             'node silently falls back to a MOCK TEST PATTERN. That is the '
             'failure boat/camera/status makes visible.')
    return True


# ======================================================================
# STAGE 1: Arduino serial link
# ======================================================================

def open_arduino(serial_mod, port=None, quiet=False):
    port = port or find_port(ARDUINO_PORT_CANDIDATES, 'Arduino')[0]
    if not port:
        if not quiet:
            bad('no Arduino serial port found -- run stage 0')
        return None, None
    try:
        ser = serial_mod.Serial(port, ARDUINO_BAUD, timeout=0.2)
    except Exception as exc:  # noqa: BLE001 - pyserial raises several types
        if not quiet:
            bad('could not open %s: %s' % (port, exc))
            note("""
                If this is a permissions error:
                  sudo usermod -aG dialout $USER      (then log out and back in)
                If the port is busy, something else has it open -- most
                likely a motor_driver node still running. Stop the ROS
                stack before running this tool: they cannot share the port.
                """)
        return None, None
    # The UNO R4 resets when the port opens. Wait for the sketch to boot.
    time.sleep(2.0)
    ser.reset_input_buffer()
    return ser, port


def read_lines(ser, seconds):
    """Collect whole lines for `seconds`. Returns a list of str."""
    lines = []
    buf = b''
    deadline = time.time() + seconds
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', errors='replace').strip()
                if text:
                    lines.append(text)
        else:
            time.sleep(0.01)
    return lines


def stage1_arduino_link():
    hdr('STAGE 1  Arduino link: is the firmware alive and talking?')
    note("""
        The Arduino should be chattering on its own with no prompting:
          IMU AX:.. AY:.. AZ:.. GR:.. GP:.. GY:..   every 20ms
          STATUS L_IS:.. R_IS:.. WDT:..             every 500ms
        If you see nothing at all, the sketch is not running or is not the
        one in this repo. If you see IMU lines but no STATUS lines (or
        vice versa) you are running an older sketch.
        """)
    serial_mod = try_import('serial', 'pip install pyserial --break-system-packages')
    if serial_mod is None:
        RESULT.add(1, 'arduino link', 'SKIP', 'pyserial missing')
        return False

    ser, port = open_arduino(serial_mod)
    if ser is None:
        RESULT.add(1, 'arduino link', 'FAIL', 'port not open')
        return False

    try:
        info('listening on %s @ %d for 3 seconds...' % (port, ARDUINO_BAUD))
        lines = read_lines(ser, 3.0)

        imu = [ln for ln in lines if ln.startswith('IMU ')]
        status = [ln for ln in lines if ln.startswith('STATUS ')]
        other = [ln for ln in lines
                 if not ln.startswith('IMU ') and not ln.startswith('STATUS ')]

        if not lines:
            bad('nothing received in 3 seconds')
            note("""
                The board is enumerating but the sketch is not talking. Check:
                  * is boat_motor_controller.ino actually flashed?
                  * does its SERIAL_BAUD match 115200?
                  * does it compile? It needs the Arduino_Modulino library --
                    if that is missing the sketch will not have uploaded, and
                    the board may still be running whatever was on it before.
                """)
            RESULT.add(1, 'arduino link', 'FAIL', 'silent')
            return False

        ok('received %d lines (%d IMU, %d STATUS, %d other)'
           % (len(lines), len(imu), len(status), len(other)))
        for ln in lines[:3]:
            note(ln)

        rate = len(imu) / 3.0
        if imu:
            ok('IMU stream at ~%.0f Hz (firmware targets 50 Hz)' % rate)
            RESULT.add(1, 'arduino imu stream', 'PASS', '%.0f Hz' % rate)
        else:
            bad('no IMU lines -- the Modulino is not being read')
            note("""
                Most likely the Modulino Movement is not detected on the
                Qwiic port. Check the Qwiic cable is fully seated at BOTH
                ends (they unclip easily), and that the sketch's
                Modulino.begin() succeeded. A 6-axis IMU that is not
                reporting means no heading at all, which now visibly
                degrades the failsafe.
                """)
            RESULT.add(1, 'arduino imu stream', 'FAIL', 'no IMU lines')

        if status:
            ok('STATUS stream present')
            last = status[-1]
            note(last)
            if 'WDT:1' in last:
                ok('watchdog is currently TRIPPED -- correct, nothing is '
                   'commanding the motors')
                note('the firmware starts in the stopped state and stays '
                     'there until a valid command arrives. That is the '
                     'behaviour you want.')
            else:
                warn('watchdog reports NOT tripped with nothing commanding')
                note('something else is sending commands on this port -- is '
                     'a motor_driver node still running?')
            RESULT.add(1, 'arduino status stream', 'PASS')
        else:
            warn('no STATUS lines in 3 s (expected ~6)')
            RESULT.add(1, 'arduino status stream', 'WARN', 'none seen')

        return bool(imu or status)
    finally:
        try:
            ser.write(b'L:0 R:0\n')
            ser.close()
        except Exception:  # noqa: BLE001
            pass


# ======================================================================
# STAGE 2: motors
# ======================================================================

def motor_safety_gate():
    print()
    print('  %s%s MOTOR TEST SAFETY GATE %s' % (BOLD, RED, RST))
    note("""
        Before you continue, physically confirm ALL of these:

          [ ] PROPELLERS ARE OFF the motor shafts
          [ ] motors are BOLTED DOWN to a bench or into the hull
          [ ] an in-line FUSE is fitted on the LiPo positive lead
          [ ] you can DISCONNECT THE BATTERY within one second, by hand
          [ ] nobody's hands, hair, cables or fingers are near the shafts
          [ ] the LiPo is on a fire-safe surface, not on carpet or a sofa

        This tool caps throttle at %d%% and each burst at %.0f seconds.
        Even so: a shaft with a set screw sticking out is a hazard at
        16,800 RPM.
        """ % (MAX_TEST_THROTTLE_PCT, BURST_SECONDS))
    answer = ask('type NOPROPS to confirm the propellers are removed')
    if answer != 'NOPROPS':
        warn('not confirmed -- skipping every motor test')
        return False
    return True


def send_thrust(ser, left, right):
    ser.write(('L:%d R:%d\n' % (int(left), int(right))).encode('ascii'))
    ser.flush()


def hold_thrust(ser, left, right, seconds, label):
    """Hold a command for `seconds`, re-sending at 10 Hz.

    The firmware watchdog zeroes the motors after FIRMWARE_WATCHDOG_MS
    without a command, so a single write only gives a twitch.
    """
    print('  %s>>>%s %s  (L=%d R=%d for %.1fs)'
          % (YEL, RST, label, left, right, seconds))
    end = time.time() + seconds
    try:
        while time.time() < end:
            send_thrust(ser, left, right)
            time.sleep(0.1)
    finally:
        for _ in range(5):
            send_thrust(ser, 0, 0)
            time.sleep(0.02)


def stage2_motors():
    hdr('STAGE 2  Motors: do they turn, the right way, on the right side?')
    note("""
        THE THREE QUESTIONS THIS ANSWERS, in order of how much time they
        save you later:

          1. Does each motor get power at all?  (wiring / fuse / driver)
          2. Is LEFT actually left?             (swapped signal wires)
          3. Does forward actually go forward?  (swapped M+/M- on the
                                                 motor terminals)

        Question 3 is worth dwelling on. A reversed motor is not a code
        bug and must not be fixed in code -- swap the two motor wires at
        the BTS7960's M+/M- terminals. If you "fix" it in software you
        will have a boat whose left and right disagree with every comment
        in the codebase, and the next person to touch it (you, in three
        months) will lose a day.
        """)
    serial_mod = try_import('serial', 'pip install pyserial --break-system-packages')
    if serial_mod is None:
        RESULT.add(2, 'motor test', 'SKIP', 'pyserial missing')
        return False
    if not motor_safety_gate():
        RESULT.add(2, 'motor test', 'SKIP', 'safety gate not confirmed')
        return False

    ser, port = open_arduino(serial_mod)
    if ser is None:
        RESULT.add(2, 'motor test', 'FAIL', 'port not open')
        return False

    throttle = MAX_TEST_THROTTLE_PCT
    try:
        raw = ask('test throttle percent (1-%d)' % MAX_TEST_THROTTLE_PCT,
                  str(MAX_TEST_THROTTLE_PCT))
        throttle = max(1, min(MAX_TEST_THROTTLE_PCT, int(raw)))
    except ValueError:
        pass

    tests = [
        ('LEFT forward', throttle, 0,
         'the LEFT (port) motor should turn, pushing the boat FORWARD'),
        ('LEFT reverse', -throttle, 0,
         'the LEFT motor should turn the OTHER way'),
        ('RIGHT forward', 0, throttle,
         'the RIGHT (starboard) motor should turn, pushing FORWARD'),
        ('RIGHT reverse', 0, -throttle,
         'the RIGHT motor should turn the OTHER way'),
        ('BOTH forward', throttle, throttle,
         'both motors together -- watch the current draw and listen for '
         'one being noticeably weaker'),
    ]

    try:
        for name, left, right, expectation in tests:
            print()
            print('  %s%s%s' % (BOLD, name, RST))
            note('EXPECT: %s' % expectation)
            if not confirm('run this test now'):
                RESULT.add(2, name, 'SKIP', 'operator skipped')
                continue
            for count in (3, 2, 1):
                print('  starting in %d...' % count, end='\r', flush=True)
                time.sleep(1)
            hold_thrust(ser, left, right, BURST_SECONDS, name)

            answer = ask('did it behave as expected? (y / n / nothing)', 'y')
            if answer.lower().startswith('y'):
                ok('%s confirmed' % name)
                RESULT.add(2, name, 'PASS')
            elif answer.lower().startswith('not'):
                bad('%s -- NO MOVEMENT AT ALL' % name)
                _diagnose_dead_motor()
                RESULT.add(2, name, 'FAIL', 'no movement')
            else:
                bad('%s -- wrong behaviour' % name)
                _diagnose_wrong_motor()
                RESULT.add(2, name, 'FAIL', 'wrong direction or side')

        # Current sense, read straight off the STATUS line.
        print()
        info('reading BTS7960 current-sense values at rest...')
        ser.reset_input_buffer()
        idle = [ln for ln in read_lines(ser, 2.0) if ln.startswith('STATUS')]
        if idle:
            note('at rest: %s' % idle[-1])
            note("""
                L_IS/R_IS are RAW analogRead values (0-1023), not amps --
                the firmware deliberately does not convert them, because
                the conversion depends on your specific module. They are
                still useful as a relative signal: they should sit near
                zero at rest and rise visibly under load. If they read a
                few hundred at rest, the sense pins are miswired or
                floating.
                """)
            RESULT.add(2, 'current sense idle', 'PASS', idle[-1])
        else:
            warn('no STATUS line to read current sense from')
            RESULT.add(2, 'current sense idle', 'WARN')
        return True
    finally:
        for _ in range(5):
            send_thrust(ser, 0, 0)
            time.sleep(0.02)
        ser.close()
        info('motors commanded to zero and port closed')


def _diagnose_dead_motor():
    note("""
        NOTHING MOVED. Work through this in order -- it is roughly cheapest
        and most likely first:

        1. IS THE LIPO CONNECTED AND THE FUSE INTACT?
           Multimeter across the BTS7960 B+ / B- terminals. You should see
           your pack voltage (~11-12.6V for a 3S). Zero here means fuse,
           switch, or connector -- not code.

        2. IS THE GROUND SHARED?
           THIS IS THE MOST COMMON CAUSE and it is invisible. You are now
           running the Pi from a POWERBANK and the motors from a LiPo --
           two completely separate supplies. The Arduino's PWM output is a
           voltage relative to ARDUINO ground; the BTS7960 reads it
           relative to ITS ground. If those two grounds are not physically
           bonded, the driver sees noise instead of a signal and the motor
           does nothing (or twitches randomly).
           FIX: run a wire from the LiPo negative / BTS7960 GND to an
           Arduino GND pin. The Arduino gets its ground from the Pi via
           USB, so this one wire ties all three together.

        3. ARE THE ENABLE PINS HIGH?
           R_EN and L_EN must be driven HIGH for that half-bridge to do
           anything. The firmware drops them LOW on watchdog timeout. If
           they are not wired to D2/D4 (left) and D7/D8 (right), or the
           wires are in the wrong holes, PWM will have no effect at all.

        4. IS THE PWM ON A PWM-CAPABLE PIN?
           RPWM/LPWM must be on D5/D6 (left) and D9/D10 (right). The parts
           log flags an open question here: the UNO R4 is a different chip
           from a classic Uno, and "are these still PWM pins with the same
           behaviour" was never confirmed on hardware. If everything else
           checks out, this is a real suspect.

        5. MEASURE THE DRIVER OUTPUT.
           Multimeter across the BTS7960 M+ / M- while a test is running.
           Voltage there but no rotation = motor or its leads.
           No voltage there = signal, enable, or driver.
        """)


def _diagnose_wrong_motor():
    note("""
        WRONG SIDE (left command moved the right motor):
          swap the two signal harnesses at the ARDUINO end. Left is
          D5/D6/D2/D4, right is D9/D10/D7/D8. Do not fix this in code.

        WRONG DIRECTION (forward went backwards):
          swap the two MOTOR wires at that driver's M+ / M- terminals.
          Do not fix this in code either.

        Why not in code: waypoint_follower's differential steering has an
        explicit, hard-won comment about which thruster must be stronger
        to turn which way -- it was wrong once already and caused the boat
        to drive away from its target. Every comment in the control stack
        assumes the physical convention. Bend the wiring to the code, not
        the code to the wiring.

        Note also that you have a matched CW/CCW propeller pair. When the
        props go back on, make sure each one is on the motor it was
        matched to, or the boat will crab under equal thrust.
        """)


# ======================================================================
# STAGE 3: watchdog
# ======================================================================

def stage3_watchdog():
    hdr('STAGE 3  Watchdog: does the Arduino stop on its own?')
    note("""
        THE SINGLE MOST IMPORTANT TEST IN THIS SCRIPT.

        Everything else in the stack -- the arbiter, the failsafe, the
        e-stop button -- runs on the Pi. If the Pi crashes, is unplugged,
        or simply stops sending, none of it can help you. The firmware's
        own watchdog is the only thing left, and it must zero the motors
        AND drop the enable pins with no dependency on anything else.

        This must be verified before the boat touches water.

        This test commands a low throttle, then stops sending, and checks
        the firmware reports the watchdog tripping within %d ms.
        """ % FIRMWARE_WATCHDOG_MS)
    serial_mod = try_import('serial', 'pip install pyserial --break-system-packages')
    if serial_mod is None:
        RESULT.add(3, 'watchdog', 'SKIP', 'pyserial missing')
        return False
    if not motor_safety_gate():
        RESULT.add(3, 'watchdog', 'SKIP', 'safety gate not confirmed')
        return False

    ser, _ = open_arduino(serial_mod)
    if ser is None:
        RESULT.add(3, 'watchdog', 'FAIL', 'port not open')
        return False

    try:
        info('commanding 20%% for 1.5s, then going silent...')
        end = time.time() + 1.5
        while time.time() < end:
            send_thrust(ser, 20, 20)
            time.sleep(0.1)

        ser.reset_input_buffer()
        t0 = time.time()
        tripped_at = None
        lines = []
        while time.time() - t0 < 3.0:
            for ln in read_lines(ser, 0.25):
                lines.append(ln)
                if ln.startswith('STATUS') and 'WDT:1' in ln and tripped_at is None:
                    tripped_at = time.time() - t0

        if tripped_at is None:
            bad('the watchdog never reported tripping within 3 seconds')
            note("""
                DO NOT PUT THE BOAT IN WATER. If the firmware does not stop
                on its own, an unplugged USB cable or a crashed Pi leaves
                the motors running at whatever they were last told.
                Check COMMAND_TIMEOUT_MS in the sketch and that the
                watchdog branch actually drops the EN pins.
                """)
            RESULT.add(3, 'firmware watchdog', 'FAIL', 'never tripped')
            return False

        # STATUS comes every 500 ms, so the observed trip time is only
        # accurate to one status period.
        ok('watchdog reported tripped %.2fs after commands stopped' % tripped_at)
        note("""
            The firmware timeout is %d ms, but STATUS is only sent every
            500 ms, so anything under about 1.1 s here is consistent with
            a correctly working watchdog. This test cannot resolve finer
            than one status period -- it proves the watchdog fires, not
            exactly when.
            """ % FIRMWARE_WATCHDOG_MS)
        RESULT.add(3, 'firmware watchdog', 'PASS' if tripped_at < 1.5 else 'WARN',
                   '%.2fs' % tripped_at)

        print()
        note("""
            NOW DO THE TEST THIS SCRIPT CANNOT DO FOR YOU:
            with the motors running (props off!), PHYSICALLY UNPLUG THE
            USB CABLE. Both motors must stop immediately. That is the
            failure mode the watchdog exists for, and pulling the cable is
            the only honest way to test it.
            """)
        if confirm('did you run the unplug test, and did the motors stop'):
            ok('USB-unplug watchdog test confirmed by operator')
            RESULT.add(3, 'usb unplug stop', 'PASS', 'operator confirmed')
        else:
            warn('USB-unplug test not confirmed')
            RESULT.add(3, 'usb unplug stop', 'WARN', 'not confirmed')
        return True
    finally:
        try:
            send_thrust(ser, 0, 0)
            ser.close()
        except Exception:  # noqa: BLE001
            pass


# ======================================================================
# STAGE 4: IMU
# ======================================================================

def stage4_imu():
    hdr('STAGE 4  IMU: is the Modulino reporting, and does it move?')
    note("""
        The IMU is on the ARDUINO's Qwiic port, not the Pi's I2C bus
        So this reads the Arduino's relayed IMU line rather
        than probing I2C.

        Two things are being checked, and the second is the one people
        skip: that data arrives, and that it CHANGES WHEN YOU MOVE THE
        BOAT. A stuck sensor reporting a constant plausible value passes
        the first check and fails the second.
        """)
    serial_mod = try_import('serial', 'pip install pyserial --break-system-packages')
    if serial_mod is None:
        RESULT.add(4, 'imu', 'SKIP', 'pyserial missing')
        return False
    ser, _ = open_arduino(serial_mod)
    if ser is None:
        RESULT.add(4, 'imu', 'FAIL', 'port not open')
        return False

    def parse(lines):
        out = []
        for ln in lines:
            if not ln.startswith('IMU '):
                continue
            values = {}
            for token in ln[4:].split():
                if ':' in token:
                    key, _, val = token.partition(':')
                    try:
                        values[key] = float(val)
                    except ValueError:
                        pass
            if len(values) == 6:
                out.append(values)
        return out

    try:
        info('hold the boat STILL for 3 seconds...')
        time.sleep(0.5)
        ser.reset_input_buffer()
        still = parse(read_lines(ser, 3.0))
        if not still:
            bad('no parsable IMU lines')
            RESULT.add(4, 'imu stream', 'FAIL', 'no data')
            return False
        ok('%d IMU samples (~%.0f Hz)' % (len(still), len(still) / 3.0))

        # At rest the accelerometer magnitude should be about 1 g.
        mags = [(s['AX'] ** 2 + s['AY'] ** 2 + s['AZ'] ** 2) ** 0.5
                for s in still]
        avg = sum(mags) / len(mags)
        if 0.85 <= avg <= 1.15:
            ok('accelerometer magnitude at rest: %.3f g  (expected ~1.000)' % avg)
            RESULT.add(4, 'imu gravity check', 'PASS', '%.3f g' % avg)
        else:
            bad('accelerometer magnitude at rest: %.3f g -- expected ~1.0' % avg)
            note("""
                Either the units are not g (check the sketch's getX/Y/Z
                usage), or the sensor is not reporting real data. Every
                downstream tilt and heading calculation assumes 1g at rest.
                """)
            RESULT.add(4, 'imu gravity check', 'FAIL', '%.3f g' % avg)

        # The axis that sees gravity gives the mounting. Roll/pitch assume Z up.
        az = sum(s['AZ'] for s in still) / len(still)
        ax = sum(s['AX'] for s in still) / len(still)
        ay = sum(s['AY'] for s in still) / len(still)
        note('mean accel at rest: AX=%.3f AY=%.3f AZ=%.3f' % (ax, ay, az))
        if abs(az) > 0.85:
            ok('gravity is on the Z axis -- the board is mounted flat')
        else:
            warn('gravity is NOT mostly on Z -- the board is not flat, or is '
                 'mounted on its side')
            note('roll/pitch and the tilt colouring on the dashboard assume '
                 'Z is up. Note the actual orientation and mount it flat if '
                 'you can.')

        # Gyro bias. This is the number heading.py spends its life fighting.
        gyro_bias = [sum(s[k] for s in still) / len(still)
                     for k in ('GR', 'GP', 'GY')]
        note('mean gyro rate at rest (= BIAS): roll=%.3f pitch=%.3f yaw=%.3f deg/s'
             % tuple(gyro_bias))
        yaw_bias = abs(gyro_bias[2])
        if yaw_bias < 1.0:
            ok('yaw gyro bias %.3f deg/s -- that is ~%.1f degrees of drift '
               'per minute' % (yaw_bias, yaw_bias * 60))
        else:
            warn('yaw gyro bias %.3f deg/s = ~%.0f degrees per MINUTE of drift'
                 % (yaw_bias, yaw_bias * 60))
        note("""
            WRITE THIS NUMBER DOWN. It is the real value behind
            heading.py's DEFAULT_ASSUMED_DRIFT_DPS, which is currently a
            guess of 0.5 deg/s that has never been measured. The heading
            estimator learns and subtracts this bias automatically while
            the boat is still, but the RESIDUAL after subtraction is what
            sets how fast a coasting heading goes bad -- and therefore how
            long DEAD_RECKONING stays trustworthy.
            """)
        RESULT.add(4, 'gyro bias', 'PASS' if yaw_bias < 1.0 else 'WARN',
                   '%.3f deg/s yaw' % yaw_bias)

        # Does it respond?
        print()
        info('now TILT AND ROTATE the boat for 3 seconds when prompted')
        if confirm('ready'):
            ser.reset_input_buffer()
            print('  %sMOVE IT NOW%s' % (YEL, RST))
            moving = parse(read_lines(ser, 3.0))
            if not moving:
                bad('no samples during the movement test')
                RESULT.add(4, 'imu responds', 'FAIL', 'no data')
            else:
                span = max(abs(s['GY']) for s in moving)
                accel_range = (max(s['AX'] for s in moving)
                               - min(s['AX'] for s in moving))
                if span > 10.0 and accel_range > 0.1:
                    ok('IMU responds: peak yaw rate %.1f deg/s, AX range %.2f g'
                       % (span, accel_range))
                    RESULT.add(4, 'imu responds', 'PASS',
                               'peak %.0f deg/s' % span)
                else:
                    bad('IMU barely changed (peak yaw %.1f deg/s, AX range '
                        '%.2f g) -- is it stuck?' % (span, accel_range))
                    note('a sensor reporting a constant plausible value is '
                         'the failure this test exists to catch')
                    RESULT.add(4, 'imu responds', 'FAIL', 'no response')
        return True
    finally:
        try:
            ser.close()
        except Exception:  # noqa: BLE001
            pass


# ======================================================================
# STAGE 5: GPS
# ======================================================================

def stage5_gps():
    hdr('STAGE 5  GPS: NMEA sentences, and an actual fix')
    note("""
        The GY-NEO6MV2 talks 9600 8N1 NMEA on /dev/serial0 and defaults to
        1 Hz. Two separate things to check:

          * SENTENCES ARRIVING -- proves wiring, baud and the UART being
            free. Works indoors.
          * AN ACTUAL FIX      -- needs sky. A cold start with no view of
            the sky can take several minutes or never succeed.

        Sentences with no fix is the NORMAL indoor result and is a pass
        for wiring purposes. Take it outside for the fix.
        """)
    serial_mod = try_import('serial', 'pip install pyserial --break-system-packages')
    if serial_mod is None:
        RESULT.add(5, 'gps', 'SKIP', 'pyserial missing')
        return False

    port, _ = find_port(GPS_PORT_CANDIDATES, 'GPS')
    if not port:
        bad('no GPS serial port -- see stage 0 for enabling the UART')
        RESULT.add(5, 'gps port', 'FAIL', 'no device')
        return False

    try:
        ser = serial_mod.Serial(port, GPS_BAUD, timeout=1.0)
    except Exception as exc:  # noqa: BLE001
        bad('could not open %s: %s' % (port, exc))
        note('if the port is busy, a getty login shell may still be bound '
             'to it -- see stage 0')
        RESULT.add(5, 'gps port', 'FAIL', str(exc))
        return False

    try:
        info('listening on %s @ %d for 8 seconds...' % (port, GPS_BAUD))
        lines = read_lines(ser, 8.0)
        nmea = [ln for ln in lines if ln.startswith('$')]
        if not nmea:
            bad('no NMEA sentences received')
            note("""
                Check, in order:
                  * TX/RX are CROSSED. GPS TX -> Pi RX, GPS RX -> Pi TX.
                    Straight-through is the classic mistake and gives
                    exactly this symptom.
                  * the module has 3.3V or 5V per its silkscreen, and GND
                  * the red LED on the module blinks once it is tracking
                  * the UART is enabled and the serial login shell is off
                """)
            RESULT.add(5, 'gps sentences', 'FAIL', 'silent')
            return False

        ok('%d NMEA sentences in 8s' % len(nmea))
        kinds = sorted({ln.split(',')[0] for ln in nmea})
        note('sentence types: %s' % ' '.join(kinds))
        RESULT.add(5, 'gps sentences', 'PASS', '%d sentences' % len(nmea))

        # GGA field 6 is fix quality (0 = no fix), field 7 satellite count.
        fixes = [ln for ln in nmea if ln.startswith(('$GPGGA', '$GNGGA'))]
        got_fix = False
        sats = 0
        for ln in fixes:
            parts = ln.split(',')
            if len(parts) > 7 and parts[6] not in ('', '0'):
                got_fix = True
                try:
                    sats = max(sats, int(parts[7] or 0))
                except ValueError:
                    pass
        if got_fix:
            ok('GPS HAS A FIX (%d satellites)' % sats)
            note(fixes[-1])
            RESULT.add(5, 'gps fix', 'PASS', '%d sats' % sats)
            note("""
                Before a real run, sanity-check the position on a map. And
                remember the geofence default centre is Lake Eola, Florida
                -- at any Norwegian site the fence is breached the instant
                this fix arrives. Set it from the dashboard at the dock.
                """)
        else:
            warn('sentences arriving but NO FIX yet')
            note("""
                Normal indoors. The wiring is proven -- that is what this
                stage was really for. For a fix: take it outside with a
                clear view of the sky, and allow up to several minutes for
                a cold start. A fix acquired once is much faster to
                reacquire.
                """)
            RESULT.add(5, 'gps fix', 'WARN', 'no fix (indoors?)')
        return True
    finally:
        try:
            ser.close()
        except Exception:  # noqa: BLE001
            pass


# ======================================================================
# STAGE 6: camera
# ======================================================================

def stage6_camera():
    hdr('STAGE 6  Camera: is it real, and can it capture?')
    note("""
        camera_publisher.py shells out to `rpicam-vid`. If that binary is
        missing or the camera is not detected, the node falls back to a
        SYNTHETIC TEST PATTERN, reported on boat/camera/status.

        This stage checks the camera at the OS level, before ROS is
        involved at all.
        """)
    tool = None
    for candidate in ('rpicam-hello', 'libcamera-hello'):
        if shutil.which(candidate):
            tool = candidate
            break
    if tool is None:
        bad('neither rpicam-hello nor libcamera-hello is installed')
        note('without rpicam-vid, camera_publisher WILL run in mock mode')
        RESULT.add(6, 'camera tools', 'FAIL', 'not installed')
        return False
    ok('using %s' % tool)

    rc, out = run([tool, '--list-cameras'], timeout=15)
    if rc != 0 or 'Available cameras' not in out:
        bad('no camera detected')
        note("""
            Check:
              * the CSI ribbon is seated at BOTH ends, contacts facing the
                right way (they are not symmetric)
              * you have the 50cm FFC extension in the parts log -- long
                ribbons are more sensitive to a poor seat
              * the camera is enabled in raspi-config
              * try reseating with the Pi POWERED OFF
            Raw output follows:
            """)
        note(out[:600])
        RESULT.add(6, 'camera detected', 'FAIL')
        return False

    ok('camera detected')
    for line in out.strip().splitlines()[:6]:
        note(line)
    RESULT.add(6, 'camera detected', 'PASS')

    if confirm('capture a 1-second test video to /tmp'):
        target = '/tmp/asv_camera_test.h264'
        vid = 'rpicam-vid' if shutil.which('rpicam-vid') else 'libcamera-vid'
        rc, out = run([vid, '-t', '1000', '--width', '640', '--height', '480',
                       '--framerate', '10', '--nopreview', '-o', target],
                      timeout=30)
        size = os.path.getsize(target) if os.path.exists(target) else 0
        if rc == 0 and size > 1000:
            ok('captured %s (%.1f KB)' % (target, size / 1024.0))
            note("""
                640x480 @ 10fps is what camera_publisher is hardcoded to.
                The sensor can do more, and resolution sets detection
                range. Revisit against the storage budget.
                """)
            RESULT.add(6, 'camera capture', 'PASS', '%.0f KB' % (size / 1024.0))
        else:
            bad('capture failed (rc=%d, %d bytes)' % (rc, size))
            note(out[:400])
            RESULT.add(6, 'camera capture', 'FAIL')
    return True


# ======================================================================
# STAGE 7: I2C / INA226
# ======================================================================

def stage7_i2c_power():
    hdr('STAGE 7  I2C: are the INA226 power monitors where we think?')
    note("""
        power_monitor.py assumes three INA226 boards at 0x40, 0x41 and
        0x44. The parts log is explicit that those addresses are a FIRST
        DRAFT GUESS against your boards' actual solder jumpers, and were
        never verified. This stage is how you find out.

        Remember the IMU is NOT on this bus -- it is on the Arduino's
        Qwiic port. Anything you find here should be an INA226.
        """)
    if not glob.glob('/dev/i2c-*'):
        bad('no I2C bus -- enable it via raspi-config, then reboot')
        RESULT.add(7, 'i2c bus', 'FAIL', 'no /dev/i2c-*')
        return False

    found = []
    if shutil.which('i2cdetect'):
        rc, out = run(['i2cdetect', '-y', '1'], timeout=10)
        if rc == 0:
            print()
            for line in out.strip().splitlines():
                note(line)
            for row in out.strip().splitlines()[1:]:
                base = row.split(':')[0].strip()
                if not base:
                    continue
                try:
                    row_base = int(base, 16)
                except ValueError:
                    continue
                cells = row.split(':', 1)[1].split()
                for idx, cell in enumerate(cells):
                    if cell not in ('--', 'UU'):
                        found.append(row_base + idx)
        else:
            warn('i2cdetect failed: %s' % out.strip()[:120])
    else:
        warn('i2cdetect not installed  (sudo apt install i2c-tools)')
        smbus = try_import('smbus2', 'pip install smbus2 --break-system-packages')
        if smbus is not None:
            try:
                with smbus.SMBus(1) as bus:
                    for addr in range(0x03, 0x78):
                        try:
                            bus.read_byte(addr)
                            found.append(addr)
                        except OSError:
                            pass
            except OSError as exc:
                bad('could not open I2C bus 1: %s' % exc)

    print()
    if not found:
        bad('no I2C devices found at all')
        note("""
            If the INA226s are wired: check SDA/SCL are not swapped, that
            each board has 3.3V and GND, and that pull-ups are present
            (most breakouts include them). If they are NOT wired yet, this
            is expected -- power_monitor simply publishes no channels and
            the dashboard shows NO MONITOR, which is honest.
            """)
        RESULT.add(7, 'ina226 devices', 'WARN', 'none found')
        return False

    ok('I2C devices found: %s' % ', '.join('0x%02X' % a for a in found))
    matched, unexpected = [], []
    for addr in found:
        if addr in INA226_EXPECTED:
            matched.append((addr, INA226_EXPECTED[addr]))
        else:
            unexpected.append(addr)

    for addr, name in matched:
        ok('0x%02X matches the expected %r channel' % (addr, name))
    for addr in unexpected:
        warn('0x%02X is present but NOT in power_monitor.py\'s channel list'
             % addr)
    missing = [a for a in INA226_EXPECTED if a not in found]
    for addr in missing:
        bad('0x%02X (%s) expected but NOT present'
            % (addr, INA226_EXPECTED[addr]))

    if unexpected or missing:
        note("""
            EDIT power_monitor.py's DEFAULT_CHANNELS to match what is
            actually on the bus, or move the boards' address jumpers to
            match the code. Either is fine; guessing is not. A channel
            configured at an address with nothing on it reads as
            ok=false, which the dashboard now greys out rather than
            showing as a healthy 0.0 A -- so you will see it, but you
            still have to fix it.
            """)
    RESULT.add(7, 'ina226 addresses', 'PASS' if not missing else 'WARN',
               '%d/%d expected present' % (len(matched), len(INA226_EXPECTED)))

    # Shunt resistance is the other unverified assumption.
    note("""
        Also not verified: power_monitor.py assumes a 0.1 ohm shunt
        (DEFAULT_SHUNT_OHMS). That is the common value for cheap INA226
        breakouts but was never confirmed for yours. Current is computed
        as V_shunt / R_shunt, so a wrong shunt value scales every current
        and power reading by a constant factor.

        HOW TO CHECK IT with the multimeter in your parts log: read the
        marking on the shunt resistor itself (usually "R100" = 0.1 ohm,
        "R010" = 0.01 ohm), or run a known load and compare this node's
        reading against the meter in series. Do that once, write the
        number down, and the battery telemetry becomes trustworthy.
        """)
    return True


# ======================================================================
# STAGE 8: ROS stack
# ======================================================================

def ros_topic_hz(topic, seconds=5):
    rc, out = run(['ros2', 'topic', 'hz', topic], timeout=seconds + 3)
    for line in out.splitlines():
        if 'average rate' in line:
            return line.strip()
    return None


def stage8_ros_stack():
    hdr('STAGE 8  ROS stack: are the topics live?')
    if not have_ros():
        warn('`ros2` not on PATH -- source your workspace first:')
        note('  source install/setup.bash   (from the repo root)')
        RESULT.add(8, 'ros stack', 'SKIP', 'ros2 not sourced')
        return False
    note("""
        Start the stack in another terminal first:
          ros2 launch boat_bringup boat_bringup.launch.py
        This stage then checks that the topics that matter are actually
        publishing at sensible rates.
        """)
    if not confirm('is the stack running'):
        RESULT.add(8, 'ros stack', 'SKIP', 'stack not running')
        return False

    rc, out = run(['ros2', 'node', 'list'], timeout=10)
    nodes = [n.strip() for n in out.splitlines() if n.strip().startswith('/')]
    print()
    info('nodes: %s' % (', '.join(nodes) or 'none'))
    expected_nodes = ['motor_driver', 'gps_publisher', 'camera_publisher',
                      'heading_estimator', 'safety_monitor', 'control_arbiter',
                      'mission_manager', 'waypoint_follower', 'web_server',
                      'system_monitor', 'power_monitor']
    for name in expected_nodes:
        if any(name in n for n in nodes):
            ok('node %s running' % name)
        else:
            warn('node %s NOT running' % name)

    print()
    checks = [
        ('/boat/imu', 'IMU relayed from the Arduino', 15.0),
        ('/boat/gps/fix', 'GPS fixes', 0.5),
        ('/boat/camera/image_raw', 'camera frames', 5.0),
        ('/boat/heading', 'fused heading estimate', 10.0),
        ('/boat/thrust_command', 'arbiter output', 10.0),
        ('/boat/failsafe_state', 'failsafe supervisor', 1.0),
        ('/boat/system_status', 'host health', 0.5),
    ]
    for topic, label, min_hz in checks:
        line = ros_topic_hz(topic)
        if line is None:
            bad('%-28s %s -- NOTHING PUBLISHING' % (topic, label))
            RESULT.add(8, topic, 'FAIL', 'silent')
            continue
        try:
            rate = float(line.split('average rate:')[1].split()[0])
        except (IndexError, ValueError):
            rate = 0.0
        if rate >= min_hz:
            ok('%-28s %.1f Hz  (expect >= %.1f)' % (topic, rate, min_hz))
            RESULT.add(8, topic, 'PASS', '%.1f Hz' % rate)
        else:
            warn('%-28s %.1f Hz  (expect >= %.1f)' % (topic, rate, min_hz))
            RESULT.add(8, topic, 'WARN', '%.1f Hz' % rate)

    # The one that catches a test-pattern run.
    print()
    rc, out = run(['ros2', 'topic', 'echo', '/boat/camera/status', '--once'],
                  timeout=8)
    if rc == 0 and out.strip():
        if 'mock_mode: true' in out:
            bad('CAMERA IS IN MOCK MODE -- the video is a synthetic test '
                'pattern, not the world')
            note('any run recorded now is worthless. Fix the camera before '
                 'collecting anything.')
            RESULT.add(8, 'camera not mock', 'FAIL', 'mock_mode true')
        else:
            ok('camera reports a real source')
            RESULT.add(8, 'camera not mock', 'PASS')
        for line in out.strip().splitlines()[:10]:
            note(line)
    else:
        warn('/boat/camera/status not published -- old build, or '
             'camera_publisher not running')
        RESULT.add(8, 'camera not mock', 'WARN', 'no status topic')
    return True


# ======================================================================
# STAGE 9: recording round trip
# ======================================================================

def stage9_recording():
    hdr('STAGE 9  Recording: does a run actually reach the disk?')
    if not have_ros():
        RESULT.add(9, 'recording', 'SKIP', 'ros2 not sourced')
        return False
    note("""
        This is the round trip that matters: start a run, confirm bytes
        are growing WHILE it runs, stop it cleanly, and check the archive
        on disk. data_recorder reports `writing` from actual byte growth.

        Start the recorder in another terminal first:
          ros2 run boat_data data_recorder
        """)
    if not confirm('is data_recorder running'):
        RESULT.add(9, 'recording', 'SKIP', 'recorder not running')
        return False

    site = ask('site name for this test run', 'benchtest')
    cmd = json.dumps({'action': 'start', 'site': site,
                      'purpose': 'hardware bring-up smoke test',
                      'source': 'real'})
    info('starting a run...')
    rc, out = run(['ros2', 'topic', 'pub', '--once', '/boat/data_command',
                   'std_msgs/msg/String', '{data: %s}' % json.dumps(cmd)],
                  timeout=15)
    if rc != 0:
        bad('could not publish the start command: %s' % out.strip()[:200])
        RESULT.add(9, 'recording start', 'FAIL')
        return False

    info('recording for 8 seconds...')
    time.sleep(4)
    rc, mid = run(['ros2', 'topic', 'echo', '/boat/recording_status', '--once'],
                  timeout=8)
    time.sleep(4)

    info('stopping...')
    run(['ros2', 'topic', 'pub', '--once', '/boat/data_command',
         'std_msgs/msg/String',
         '{data: %s}' % json.dumps(json.dumps({'action': 'stop'}))],
        timeout=15)
    time.sleep(3)

    # The mid-run status is the interesting one: `writing` must be true.
    if mid.strip():
        print()
        for line in mid.strip().splitlines()[:4]:
            note(line)
        if '"writing": true' in mid:
            ok('the recorder confirmed bytes were reaching the disk mid-run')
            RESULT.add(9, 'writing confirmed', 'PASS')
        elif '"writing": false' in mid:
            bad('recording was TRUE but writing was FALSE')
            note("""
                The recorder process is alive and nothing is reaching the
                disk. Almost always this means the topics it was told to
                record have no publishers -- i.e. the stack is not
                running, or is running under different topic names.
                """)
            RESULT.add(9, 'writing confirmed', 'FAIL', 'writing false')
        else:
            warn('could not parse the recording status')
            RESULT.add(9, 'writing confirmed', 'WARN')

    # Now check the disk.
    runs_dir = Path.home() / 'asv-boat' / 'data' / 'runs'
    if not runs_dir.exists():
        bad('no run archive at %s' % runs_dir)
        RESULT.add(9, 'run archive', 'FAIL', 'no directory')
        return False
    candidates = sorted((p for p in runs_dir.iterdir() if p.is_dir()),
                        key=lambda p: p.stat().st_mtime)
    if not candidates:
        bad('the archive directory is empty')
        RESULT.add(9, 'run archive', 'FAIL', 'no runs')
        return False

    latest = candidates[-1]
    ok('latest run: %s' % latest.name)
    total = sum(f.stat().st_size for f in latest.rglob('*') if f.is_file())
    mcaps = list(latest.rglob('*.mcap'))
    print()
    for name in ('manifest.json', 'run_brief.json', 'checksums.sha256'):
        if (latest / name).exists():
            ok('%s present' % name)
        else:
            warn('%s missing' % name)
    if mcaps and total > 10000:
        ok('bag: %d mcap file(s), %.2f MB total on disk'
           % (len(mcaps), total / 1e6))
        RESULT.add(9, 'run archive', 'PASS', '%.2f MB' % (total / 1e6))
    else:
        bad('bag is empty or tiny (%d mcap files, %d bytes)'
            % (len(mcaps), total))
        RESULT.add(9, 'run archive', 'FAIL', '%d bytes' % total)

    manifest = latest / 'manifest.json'
    if manifest.exists():
        try:
            data = json.loads(manifest.read_text())
            quality = data.get('quality', {})
            note('manifest quality: %s' % json.dumps(quality))
            if quality.get('write_confirmed'):
                ok('manifest records write_confirmed: true')
            else:
                warn('manifest says write_confirmed is not true')
        except (OSError, json.JSONDecodeError) as exc:
            warn('could not read the manifest: %s' % exc)

    note("""
        Finally, confirm the provenance topics are really in the bag:
          ros2 bag info %s/bag/rosbag2
        You are looking for /boat/mode, /boat/cmd/manual,
        /boat/cmd/waypoint and /boat/cmd/failsafe. Without them a thrust
        value in the bag has no attributable source and the data cannot
        support imitation learning.
        """ % latest)
    return True


# ======================================================================
# Menu
# ======================================================================

STAGES = [
    (0, 'Inventory -- what does the Pi see?', stage0_inventory, False),
    (1, 'Arduino link -- firmware alive?', stage1_arduino_link, False),
    (2, 'MOTORS -- do they turn, correctly?', stage2_motors, True),
    (3, 'WATCHDOG -- does it stop on its own?', stage3_watchdog, True),
    (4, 'IMU -- reporting and responding?', stage4_imu, False),
    (5, 'GPS -- sentences and a fix?', stage5_gps, False),
    (6, 'Camera -- real, and can it capture?', stage6_camera, False),
    (7, 'I2C -- INA226 power monitors', stage7_i2c_power, False),
    (8, 'ROS stack -- topics live?', stage8_ros_stack, False),
    (9, 'Recording -- does a run hit the disk?', stage9_recording, False),
]


def print_stages():
    print('\n%sStages%s' % (BOLD, RST))
    for num, title, _fn, dangerous in STAGES:
        flag = ' %s[MOTORS MOVE]%s' % (RED, RST) if dangerous else ''
        print('  %d  %s%s' % (num, title, flag))
    print('  a  run everything in order')
    print('  q  quit')


def run_stage(num):
    for stage_num, _title, fn, _d in STAGES:
        if stage_num == num:
            try:
                fn()
            except KeyboardInterrupt:
                print()
                warn('interrupted -- motors were commanded to zero')
                RESULT.add(num, 'stage %d' % num, 'WARN', 'interrupted')
            except Exception as exc:  # noqa: BLE001 - a failing check must
                # not stop the run; get through the list and see the summary.
                bad('stage %d raised: %r' % (num, exc))
                RESULT.add(num, 'stage %d' % num, 'FAIL', repr(exc))
            return
    bad('no such stage: %s' % num)


def main():
    parser = argparse.ArgumentParser(
        description='Interactive hardware bring-up and bench test for the '
                    'ASV boat. Run this ON THE PI.')
    parser.add_argument('--stage', type=int, action='append',
                        help='run one stage (repeatable)')
    parser.add_argument('--all', action='store_true',
                        help='run every stage in order')
    parser.add_argument('--list', action='store_true', help='list the stages')
    args = parser.parse_args()

    print(__doc__.split('READ THIS')[0])

    if args.list:
        print_stages()
        return 0

    try:
        if args.all:
            for num, _t, _f, _d in STAGES:
                run_stage(num)
        elif args.stage:
            for num in args.stage:
                run_stage(num)
        else:
            while True:
                print_stages()
                choice = ask('stage').lower()
                if choice in ('q', 'quit', 'exit'):
                    break
                if choice == 'a':
                    for num, _t, _f, _d in STAGES:
                        run_stage(num)
                    continue
                try:
                    run_stage(int(choice))
                except ValueError:
                    bad('enter a stage number, "a", or "q"')
    except KeyboardInterrupt:
        print()
        warn('interrupted')

    return RESULT.summary()


if __name__ == '__main__':
    sys.exit(main())
