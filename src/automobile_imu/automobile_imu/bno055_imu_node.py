import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from smbus2 import SMBus


BNO055_ADDRESS = 0x28
I2C_BUS = 7

BNO055_OPR_MODE_ADDR = 0x3D
BNO055_PWR_MODE_ADDR = 0x3E
BNO055_SYS_TRIGGER_ADDR = 0x3F
BNO055_UNIT_SEL_ADDR = 0x3B

CONFIG_MODE = 0x00
IMUPLUS_MODE = 0x08

QUATERNION_DATA_W_LSB_ADDR = 0x20
GYRO_DATA_X_LSB_ADDR = 0x14
LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28


def to_int16(lsb, msb):
    value = (msb << 8) | lsb
    if value & 0x8000:
        value -= 65536
    return value


class BNO055ImuNode(Node):

    def __init__(self):
        super().__init__('bno055_imu_node')

        self.publisher = self.create_publisher(
            Imu,
            '/automobile/imu/data',
            10
        )

        self.bus = SMBus(I2C_BUS)

        self.configure_bno055()

        self.timer = self.create_timer(0.02, self.publish_imu)

        self.get_logger().info('BNO055 IMU node publishing to /automobile/imu/data')

    def configure_bno055(self):
        self.bus.write_byte_data(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, CONFIG_MODE)
        time.sleep(0.05)

        self.bus.write_byte_data(BNO055_ADDRESS, BNO055_PWR_MODE_ADDR, 0x00)
        time.sleep(0.01)

        self.bus.write_byte_data(BNO055_ADDRESS, BNO055_SYS_TRIGGER_ADDR, 0x00)
        time.sleep(0.01)

        # Default units:
        # acceleration: m/s^2
        # gyro: rad/s
        # Euler: degrees
        self.bus.write_byte_data(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, 0x02)
        time.sleep(0.01)

        # IMUPLUS = accelerometer + gyro fusion, no magnetometer
        self.bus.write_byte_data(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, IMUPLUS_MODE)
        time.sleep(0.05)

    def read_vector(self, start_register, scale):
        data = self.bus.read_i2c_block_data(BNO055_ADDRESS, start_register, 6)

        x = to_int16(data[0], data[1]) / scale
        y = to_int16(data[2], data[3]) / scale
        z = to_int16(data[4], data[5]) / scale

        return x, y, z

    def read_quaternion(self):
        data = self.bus.read_i2c_block_data(BNO055_ADDRESS, QUATERNION_DATA_W_LSB_ADDR, 8)

        w = to_int16(data[0], data[1]) / 16384.0
        x = to_int16(data[2], data[3]) / 16384.0
        y = to_int16(data[4], data[5]) / 16384.0
        z = to_int16(data[6], data[7]) / 16384.0

        return w, x, y, z

    def publish_imu(self):
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        try:
            w, x, y, z = self.read_quaternion()
            gx, gy, gz = self.read_vector(GYRO_DATA_X_LSB_ADDR, 900.0)
            ax, ay, az = self.read_vector(LINEAR_ACCEL_DATA_X_LSB_ADDR, 100.0)

            msg.orientation.w = float(w)
            msg.orientation.x = float(x)
            msg.orientation.y = float(y)
            msg.orientation.z = float(z)

            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)

            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)

            msg.orientation_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.5
            ]

            msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]

            msg.linear_acceleration_covariance = [
                0.1, 0.0, 0.0,
                0.0, 0.1, 0.0,
                0.0, 0.0, 0.1
            ]

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'BNO055 read failed: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = BNO055ImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.bus.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()