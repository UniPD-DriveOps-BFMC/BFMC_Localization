import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

import board
import busio
import adafruit_bno055


class BNO055ImuNode(Node):

    def __init__(self):
        super().__init__('bno055_imu_node')

        self.publisher = self.create_publisher(
            Imu,
            '/automobile/imu/data',
            10
        )

        i2c = busio.I2C(board.SCL, board.SDA)

        self.sensor = adafruit_bno055.BNO055_I2C(
            i2c,
            address=0x28
        )

        # Near BLDC motor: avoid magnetometer fusion
        self.sensor.mode = adafruit_bno055.IMUPLUS_MODE

        self.frame_id = 'imu_link'

        self.timer = self.create_timer(0.02, self.publish_imu)

        self.get_logger().info('Publishing BNO055 IMU data to /automobile/imu/data')

    def publish_imu(self):
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        quat = self.sensor.quaternion
        gyro = self.sensor.gyro
        accel = self.sensor.linear_acceleration

        if quat is not None:
            w, x, y, z = quat

            msg.orientation.x = float(x)
            msg.orientation.y = float(y)
            msg.orientation.z = float(z)
            msg.orientation.w = float(w)
        else:
            msg.orientation.w = 1.0

        if gyro is not None:
            msg.angular_velocity.x = float(gyro[0])
            msg.angular_velocity.y = float(gyro[1])
            msg.angular_velocity.z = float(gyro[2])

        if accel is not None:
            msg.linear_acceleration.x = float(accel[0])
            msg.linear_acceleration.y = float(accel[1])
            msg.linear_acceleration.z = float(accel[2])

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


def main(args=None):
    rclpy.init(args=args)

    node = BNO055ImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
