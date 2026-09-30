"""Power monitor for three INA226 current/voltage sensors on the Pi I2C bus.

Channels: motor_left, motor_right, battery. Not yet tested on hardware.

Current is computed from the raw shunt voltage register as
I = V_shunt / R_shunt instead of programming the chip's calibration
register, so it can be checked by hand against a multimeter.

To verify: the I2C addresses in DEFAULT_CHANNELS (set by the A0/A1
jumpers on each board, check with `i2cdetect -y 1`) and the shunt
resistance (0.1 ohm is typical for these breakouts).
"""

import rclpy
from rclpy.node import Node
from boat_msgs.msg import PowerStatus, PowerChannel

try:
    from smbus2 import SMBus
except ImportError:  # no smbus2: publish no channels
    SMBus = None

INA226_REG_SHUNT_VOLTAGE = 0x01
INA226_REG_BUS_VOLTAGE = 0x02
INA226_SHUNT_VOLTAGE_LSB = 2.5e-6   # volts per LSB, fixed by the chip
INA226_BUS_VOLTAGE_LSB = 1.25e-3    # volts per LSB, fixed by the chip

# Typical for INA226 breakouts. Not verified on these boards.
DEFAULT_SHUNT_OHMS = 0.1

# Not verified against the jumper settings.
DEFAULT_CHANNELS = [
    {'name': 'motor_left', 'address': 0x40, 'shunt_ohms': DEFAULT_SHUNT_OHMS},
    {'name': 'motor_right', 'address': 0x41, 'shunt_ohms': DEFAULT_SHUNT_OHMS},
    {'name': 'battery', 'address': 0x44, 'shunt_ohms': DEFAULT_SHUNT_OHMS},
]


def _to_signed16(msb, lsb):
    val = (msb << 8) | lsb
    if val >= 0x8000:
        val -= 0x10000
    return val


class Ina226Channel:
    """One INA226. read() returns (voltage_v, current_a, power_w) and raises
    on I2C errors; the node catches those per channel."""

    def __init__(self, bus, name, address, shunt_ohms):
        self.bus = bus
        self.name = name
        self.address = address
        self.shunt_ohms = shunt_ohms

    def read(self):
        shunt_data = self.bus.read_i2c_block_data(self.address, INA226_REG_SHUNT_VOLTAGE, 2)
        bus_data = self.bus.read_i2c_block_data(self.address, INA226_REG_BUS_VOLTAGE, 2)

        shunt_raw = _to_signed16(shunt_data[0], shunt_data[1])
        bus_raw = _to_signed16(bus_data[0], bus_data[1])

        shunt_voltage_v = shunt_raw * INA226_SHUNT_VOLTAGE_LSB
        bus_voltage_v = bus_raw * INA226_BUS_VOLTAGE_LSB

        current_a = shunt_voltage_v / self.shunt_ohms if self.shunt_ohms else 0.0
        power_w = bus_voltage_v * current_a
        return bus_voltage_v, current_a, power_w


class PowerMonitor(Node):
    def __init__(self):
        super().__init__('power_monitor')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('mock', False)

        i2c_bus_num = int(self.get_parameter('i2c_bus').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        force_mock = bool(self.get_parameter('mock').value)

        self.publisher_ = self.create_publisher(PowerStatus, 'boat/power_status', 10)

        self.channels = self._build_channels(i2c_bus_num, force_mock)

        period = 1.0 / rate_hz if rate_hz > 0 else 0.5
        self.timer = self.create_timer(period, self.timer_callback)

    def _build_channels(self, i2c_bus_num, force_mock):
        if force_mock:
            self.get_logger().info(
                'Power monitor mock mode forced via "mock" parameter -- '
                'no real INA226 reads will be attempted.')
            return []

        if SMBus is None:
            self.get_logger().warn(
                'smbus2 not installed (pip install smbus2 --break-system-packages) '
                '-- power_monitor will publish no channels until it is.')
            return []

        try:
            bus = SMBus(i2c_bus_num)
        except (FileNotFoundError, OSError) as e:
            self.get_logger().warn(
                f'Could not open I2C bus {i2c_bus_num} ({e}) -- power_monitor '
                f'will publish no channels until this is fixed.')
            return []

        channels = []
        for cfg in DEFAULT_CHANNELS:
            channels.append(Ina226Channel(bus, cfg['name'], cfg['address'], cfg['shunt_ohms']))
        self.get_logger().info(
            f'Power monitor started, {len(channels)} channel(s) configured '
            f'(addresses NOT verified against real hardware -- see module '
            f'docstring): '
            + ', '.join(f'{c.name}=0x{c.address:02X}' for c in channels))
        return channels

    def timer_callback(self):
        msg = PowerStatus()
        for channel in self.channels:
            entry = PowerChannel()
            entry.name = channel.name
            try:
                voltage_v, current_a, power_w = channel.read()
                entry.voltage_v = voltage_v
                entry.current_a = current_a
                entry.power_w = power_w
                entry.ok = True
            except Exception as e:  # noqa: BLE001 - one bad channel must not stop the others
                self.get_logger().warn(
                    f'Power channel {channel.name!r} (0x{channel.address:02X}) '
                    f'read failed: {e}', throttle_duration_sec=5.0)
                entry.voltage_v = 0.0
                entry.current_a = 0.0
                entry.power_w = 0.0
                entry.ok = False
            msg.channels.append(entry)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PowerMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
