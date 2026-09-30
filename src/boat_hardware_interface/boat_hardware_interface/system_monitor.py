"""Host health for the Pi: CPU temperature, load, throttle flags, disk space.

A Pi in a sealed enclosure in the sun throttles, and a throttled Pi drops
camera frames, which otherwise looks like a camera fault. Free disk decides
how long a recording can be.

Publishes JSON on boat/system_status (std_msgs/String) since the fields are
diagnostic only and will change. Values that can't be read are null, never
zero. Clock sync (gpsd + chrony) is not checked yet.
"""

import json
import os
import shutil
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# SoC temperature in millidegrees C. Missing on a laptop or in a container.
THERMAL_ZONE = '/sys/class/thermal/thermal_zone0/temp'

# vcgencmd get_throttled bits. The "since boot" bits are the useful ones,
# since live throttling has usually stopped by the time you look.
THROTTLE_BITS = {
    0: 'under-voltage NOW',
    1: 'ARM frequency capped NOW',
    2: 'THROTTLED NOW',
    3: 'soft temperature limit NOW',
    16: 'under-voltage occurred since boot',
    17: 'ARM frequency capping occurred since boot',
    18: 'throttling occurred since boot',
    19: 'soft temperature limit reached since boot',
}

# The Pi 4 soft-throttles around 80 C. Warn early, since cooling takes time.
TEMP_WARN_C = 70.0
TEMP_CRITICAL_C = 80.0


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')

        self.declare_parameter('rate_hz', 1.0)
        # Filesystem to report free space for: the data archive.
        self.declare_parameter(
            'disk_path', str(Path.home() / 'asv-boat' / 'data'))
        self.declare_parameter('temp_warn_c', TEMP_WARN_C)
        self.declare_parameter('temp_critical_c', TEMP_CRITICAL_C)

        self.publisher_ = self.create_publisher(
            String, 'boat/system_status', 10)

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate if rate > 0 else 1.0, self.timer_callback)

        self._boot_time = time.monotonic()
        self._warned_no_thermal = False
        self._last_temp_warn_level = None
        self._vcgencmd_available = shutil.which('vcgencmd') is not None

        self.get_logger().info(
            'System monitor started -- publishing boat/system_status at %.1f Hz '
            '(temperature %s, throttle flags %s, disk %s)'
            % (rate,
               'available' if os.path.exists(THERMAL_ZONE) else 'UNAVAILABLE',
               'available' if self._vcgencmd_available else 'UNAVAILABLE',
               self.get_parameter('disk_path').value))

    # Readers. Each returns None instead of a fake value.

    def _read_cpu_temp_c(self):
        try:
            with open(THERMAL_ZONE, 'r', encoding='utf-8') as handle:
                return round(int(handle.read().strip()) / 1000.0, 1)
        except (OSError, ValueError):
            if not self._warned_no_thermal:
                self.get_logger().warn(
                    'No CPU temperature available at %s -- this is normal on a '
                    'dev machine or in a container, and means the dashboard '
                    'will show "--" rather than a misleading 0.0 C.'
                    % THERMAL_ZONE)
                self._warned_no_thermal = True
            return None

    def _read_load(self):
        """1/5/15-minute load averages. Above ~4 on a 4-core Pi means work is queueing."""
        try:
            return [round(x, 2) for x in os.getloadavg()]
        except (OSError, AttributeError):
            return None

    def _read_throttle_flags(self):
        """Under-voltage and throttling flags from vcgencmd."""
        if not self._vcgencmd_available:
            return None
        try:
            out = subprocess.run(['vcgencmd', 'get_throttled'],
                                 capture_output=True, text=True, timeout=2)
            if out.returncode != 0:
                return None
            value = int(out.stdout.strip().split('=')[-1], 16)
        except Exception:  # noqa: BLE001 - a diagnostic read must not kill the node
            return None
        return {
            'raw': '0x%X' % value,
            'flags': [text for bit, text in sorted(THROTTLE_BITS.items())
                      if value & (1 << bit)],
        }

    def _read_disk(self):
        path = str(self.get_parameter('disk_path').value)
        for candidate in (path, str(Path.home()), '/'):
            try:
                usage = shutil.disk_usage(candidate)
            except OSError:
                continue
            return {
                'path': candidate,
                'free_gb': round(usage.free / 1e9, 1),
                'total_gb': round(usage.total / 1e9, 1),
                'used_pct': round(100.0 * usage.used / usage.total, 1),
                # Rough estimate at ~7 GB/hour for 640x480 10 fps imagery.
                'est_imagery_minutes': round(usage.free / 7e9 * 60.0, 0),
            }
        return None

    # Publish

    def timer_callback(self):
        temp_c = self._read_cpu_temp_c()

        payload = {
            'cpu_temp_c': temp_c,
            'load': self._read_load(),
            'throttle': self._read_throttle_flags(),
            'disk': self._read_disk(),
            'uptime_s': round(time.monotonic() - self._boot_time, 1),
            # Clock sync is not checked yet.
            'clock_disciplined': None,
            'clock_source': 'unchecked (gpsd+chrony not configured)',
            'temp_warn_c': float(self.get_parameter('temp_warn_c').value),
            'temp_critical_c': float(self.get_parameter('temp_critical_c').value),
        }
        self.publisher_.publish(String(data=json.dumps(payload)))

        self._warn_on_temperature(temp_c, payload['throttle'])

    def _warn_on_temperature(self, temp_c, throttle):
        """Log only when the temperature level changes, not every tick."""
        if temp_c is None:
            return
        critical = float(self.get_parameter('temp_critical_c').value)
        warn = float(self.get_parameter('temp_warn_c').value)
        level = ('critical' if temp_c >= critical
                 else 'warn' if temp_c >= warn else 'ok')
        if level == self._last_temp_warn_level:
            return
        self._last_temp_warn_level = level
        if level == 'critical':
            self.get_logger().error(
                'CPU at %.1f C -- the Pi throttles around 80-85 C, and a '
                'throttled Pi drops camera frames. Shade or ventilate the '
                'enclosure.' % temp_c)
        elif level == 'warn':
            self.get_logger().warn('CPU at %.1f C and rising' % temp_c)
        else:
            self.get_logger().info('CPU temperature back to %.1f C' % temp_c)

        if throttle and throttle.get('flags'):
            self.get_logger().warn(
                'Pi throttle flags: %s' % ', '.join(throttle['flags']))


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
