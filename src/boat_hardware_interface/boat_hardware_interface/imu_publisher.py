"""IMU node for an IMU wired directly to the Pi's I2C bus.

Not used on the current boat and not in the bringup launch: the boat's IMU
(Modulino Movement) sits on the Arduino, and motor_driver publishes
boat/imu from the serial stream. Kept for a possible Pi-side IMU later.
Not yet tested on hardware.

imu_model:
  bno055   reads the chip's onboard fusion quaternion (NDOF mode)
  mpu6050  raw 6-axis. Roll/pitch from a complementary filter, yaw from gyro
           integration only, so yaw drifts without a heading reference.

Check the orientation sign conventions against a known rotation before
using it for navigation.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    from smbus2 import SMBus
except ImportError:  # no smbus2: the node falls back to mock mode
    SMBus = None

MPU6050_ADDR = 0x68
MPU6050_REG_PWR_MGMT_1 = 0x6B
MPU6050_REG_ACCEL_XOUT_H = 0x3B
MPU6050_REG_GYRO_XOUT_H = 0x43
MPU6050_ACCEL_SCALE = 16384.0  # LSB/g at +-2g full scale (default)
MPU6050_GYRO_SCALE = 131.0     # LSB/(deg/s) at +-250deg/s full scale (default)
GRAVITY_MPS2 = 9.80665

BNO055_ADDR = 0x28
BNO055_REG_PAGE_ID = 0x07
BNO055_REG_OPR_MODE = 0x3D
BNO055_REG_SYS_TRIGGER = 0x3F
BNO055_REG_QUATERNION_DATA_W_LSB = 0x20
BNO055_OPR_MODE_CONFIG = 0x00
BNO055_OPR_MODE_NDOF = 0x0C
BNO055_QUATERNION_SCALE = 1.0 / 16384.0


def euler_to_quaternion(roll, pitch, yaw):
    """Roll/pitch/yaw in radians to (x, y, z, w), ZYX convention."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)


class Mpu6050Backend:
    """MPU6050 read plus a complementary filter for orientation."""

    # Weight of the gyro-integrated angle vs the accelerometer angle for
    # roll/pitch. 0.98 is a common starting point, not tuned.
    COMPLEMENTARY_ALPHA = 0.98

    def __init__(self, bus, address=MPU6050_ADDR):
        self.bus = bus
        self.address = address
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._last_time = None

        # The MPU6050 powers up asleep.
        self.bus.write_byte_data(self.address, MPU6050_REG_PWR_MGMT_1, 0x00)
        time.sleep(0.1)

    @staticmethod
    def _read_word_signed(bus, address, reg):
        high = bus.read_byte_data(address, reg)
        low = bus.read_byte_data(address, reg + 1)
        val = (high << 8) | low
        if val >= 0x8000:
            val -= 0x10000
        return val

    def read(self):
        ax = self._read_word_signed(self.bus, self.address, MPU6050_REG_ACCEL_XOUT_H) / MPU6050_ACCEL_SCALE
        ay = self._read_word_signed(self.bus, self.address, MPU6050_REG_ACCEL_XOUT_H + 2) / MPU6050_ACCEL_SCALE
        az = self._read_word_signed(self.bus, self.address, MPU6050_REG_ACCEL_XOUT_H + 4) / MPU6050_ACCEL_SCALE

        gx = self._read_word_signed(self.bus, self.address, MPU6050_REG_GYRO_XOUT_H) / MPU6050_GYRO_SCALE
        gy = self._read_word_signed(self.bus, self.address, MPU6050_REG_GYRO_XOUT_H + 2) / MPU6050_GYRO_SCALE
        gz = self._read_word_signed(self.bus, self.address, MPU6050_REG_GYRO_XOUT_H + 4) / MPU6050_GYRO_SCALE
        gx, gy, gz = math.radians(gx), math.radians(gy), math.radians(gz)  # -> rad/s

        now = time.monotonic()
        dt = 0.0 if self._last_time is None else max(0.0, min(0.5, now - self._last_time))
        self._last_time = now

        # Accelerometer roll/pitch, only valid when not accelerating hard.
        accel_roll = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        if dt > 0.0:
            gyro_roll = self.roll + gx * dt
            gyro_pitch = self.pitch + gy * dt
            self.roll = self.COMPLEMENTARY_ALPHA * gyro_roll + (1 - self.COMPLEMENTARY_ALPHA) * accel_roll
            self.pitch = self.COMPLEMENTARY_ALPHA * gyro_pitch + (1 - self.COMPLEMENTARY_ALPHA) * accel_pitch
            # No magnetometer, so yaw is gyro integration only and drifts.
            self.yaw += gz * dt
        else:
            self.roll, self.pitch = accel_roll, accel_pitch

        orientation = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        angular_velocity = (gx, gy, gz)
        linear_acceleration = (ax * GRAVITY_MPS2, ay * GRAVITY_MPS2, az * GRAVITY_MPS2)
        # Derived orientation, so a loose covariance. Not measured.
        orientation_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.3]
        return orientation, angular_velocity, linear_acceleration, orientation_covariance


class Bno055Backend:
    """BNO055 read of the onboard fusion quaternion."""

    def __init__(self, bus, address=BNO055_ADDR):
        self.bus = bus
        self.address = address

        self.bus.write_byte_data(self.address, BNO055_REG_PAGE_ID, 0x00)
        self.bus.write_byte_data(self.address, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_CONFIG)
        time.sleep(0.02)
        # No SYS_TRIGGER reset here: it needs a long settle and retry loop.
        # Power-cycle the board instead if it needs a reset.
        self.bus.write_byte_data(self.address, BNO055_REG_OPR_MODE, BNO055_OPR_MODE_NDOF)
        time.sleep(0.02)

    def read(self):
        data = self.bus.read_i2c_block_data(self.address, BNO055_REG_QUATERNION_DATA_W_LSB, 8)
        w = self._to_signed16(data[0], data[1]) * BNO055_QUATERNION_SCALE
        x = self._to_signed16(data[2], data[3]) * BNO055_QUATERNION_SCALE
        y = self._to_signed16(data[4], data[5]) * BNO055_QUATERNION_SCALE
        z = self._to_signed16(data[6], data[7]) * BNO055_QUATERNION_SCALE
        orientation = (x, y, z, w)

        # Gyro (0x14) and linear accel (0x28) registers are not read yet.
        angular_velocity = (0.0, 0.0, 0.0)
        linear_acceleration = (0.0, 0.0, 0.0)
        # Tighter than the MPU6050 path. Not measured.
        orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02]
        return orientation, angular_velocity, linear_acceleration, orientation_covariance

    @staticmethod
    def _to_signed16(lsb, msb):
        val = (msb << 8) | lsb
        if val >= 0x8000:
            val -= 0x10000
        return val


class MockBackend:
    """Fixed level orientation for running the stack without an IMU."""

    def read(self):
        orientation = (0.0, 0.0, 0.0, 1.0)
        angular_velocity = (0.0, 0.0, 0.0)
        linear_acceleration = (0.0, 0.0, GRAVITY_MPS2)
        orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return orientation, angular_velocity, linear_acceleration, orientation_covariance


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameter('imu_model', 'mpu6050')  # 'mpu6050' | 'bno055'
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', 0)  # 0 = use the model's default address
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('frame_id', 'imu_frame')
        self.declare_parameter('mock', False)

        model = str(self.get_parameter('imu_model').value).lower()
        i2c_bus_num = int(self.get_parameter('i2c_bus').value)
        address_param = int(self.get_parameter('address').value)
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.frame_id = self.get_parameter('frame_id').value
        force_mock = bool(self.get_parameter('mock').value)

        self.publisher_ = self.create_publisher(Imu, 'boat/imu', 10)

        self.backend = self._build_backend(model, i2c_bus_num, address_param, force_mock)

        period = 1.0 / rate_hz if rate_hz > 0 else 0.05
        self.timer = self.create_timer(period, self.timer_callback)

    def _build_backend(self, model, i2c_bus_num, address_param, force_mock):
        if force_mock or SMBus is None:
            if force_mock:
                self.get_logger().info('IMU mock mode forced via "mock" parameter.')
            else:
                self.get_logger().warn(
                    'smbus2 not installed (pip install smbus2 --break-system-packages) '
                    '-- falling back to MOCK IMU so the rest of the stack can still be '
                    'developed/tested off-hardware.')
            return MockBackend()

        try:
            bus = SMBus(i2c_bus_num)
        except (FileNotFoundError, OSError) as e:
            self.get_logger().warn(
                f'Could not open I2C bus {i2c_bus_num} ({e}) -- falling back to MOCK IMU.')
            return MockBackend()

        try:
            if model == 'bno055':
                address = address_param or BNO055_ADDR
                backend = Bno055Backend(bus, address)
                self.get_logger().info(f'IMU publisher started (BNO055 @ 0x{address:02X})')
                return backend
            elif model == 'mpu6050':
                address = address_param or MPU6050_ADDR
                backend = Mpu6050Backend(bus, address)
                self.get_logger().info(
                    f'IMU publisher started (MPU6050 @ 0x{address:02X}) -- NOTE: yaw is '
                    f'gyro-integration only and will drift, see module docstring.')
                return backend
            else:
                self.get_logger().error(
                    f'Unknown imu_model {model!r} (expected "mpu6050" or "bno055") -- '
                    f'falling back to MOCK IMU.')
                return MockBackend()
        except Exception as e:  # noqa: BLE001 - any I2C/device error at init
            self.get_logger().error(
                f'Failed to initialise {model} on I2C bus {i2c_bus_num}: {e} -- '
                f'falling back to MOCK IMU. Check wiring/address before trusting '
                f'anything downstream of boat/imu.')
            return MockBackend()

    def timer_callback(self):
        try:
            orientation, angular_velocity, linear_acceleration, orientation_cov = self.backend.read()
        except Exception as e:  # noqa: BLE001 - one bad read must not kill the node
            self.get_logger().warn(f'IMU read failed: {e}', throttle_duration_sec=5.0)
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = orientation
        msg.orientation_covariance = orientation_cov

        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = angular_velocity
        # -1.0 marks the covariance as unknown (sensor_msgs/Imu convention).
        msg.angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = linear_acceleration
        msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
