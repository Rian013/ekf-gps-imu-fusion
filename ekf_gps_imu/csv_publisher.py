import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import pandas as pd
import numpy as np
import time

class CSVPublisher(Node):
    def __init__(self):
        super().__init__('csv_publisher')

        # ─────────────────────────────────────────
        # Publishers
        # ─────────────────────────────────────────
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # ─────────────────────────────────────────
        # Load CSV files
        # ─────────────────────────────────────────
        self.get_logger().info('Loading CSV files...')

        self.gps_data = pd.read_csv('/home/rian/ros2_ws/src/ekf_gps_imu/data/gpstopic.csv')
        self.imu_data = pd.read_csv('/home/rian/ros2_ws/src/ekf_gps_imu/data/imutopic.csv')

        self.get_logger().info(f'GPS rows: {len(self.gps_data)}')
        self.get_logger().info(f'IMU rows: {len(self.imu_data)}')

        # ─────────────────────────────────────────
        # Indices to track current position in CSV
        # ─────────────────────────────────────────
        self.gps_idx = 0
        self.imu_idx = 0

        # ─────────────────────────────────────────
        # Timer — publishes at 40Hz (IMU rate)
        # ─────────────────────────────────────────
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.get_logger().info('CSV Publisher started!')

    def timer_callback(self):
        # Publish IMU data
        if self.imu_idx < len(self.imu_data):
            row = self.imu_data.iloc[self.imu_idx]

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Linear acceleration
            imu_msg.linear_acceleration.x = row['linear_acceleration.x']
            imu_msg.linear_acceleration.y = row['linear_acceleration.y']
            imu_msg.linear_acceleration.z = row['linear_acceleration.z']

            # Angular velocity
            imu_msg.angular_velocity.x = row['angular_velocity.x']
            imu_msg.angular_velocity.y = row['angular_velocity.y']
            imu_msg.angular_velocity.z = row['angular_velocity.z']

            self.imu_pub.publish(imu_msg)
            self.imu_idx += 1

        # Publish GPS data — every ~40 IMU messages (1Hz GPS)
        if self.gps_idx < len(self.gps_data):
            gps_row = self.gps_data.iloc[self.gps_idx]
            imu_time = self.imu_data.iloc[min(self.imu_idx, len(self.imu_data)-1)]['Time']

            # Sync by timestamp
            if gps_row['Time'] <= imu_time:
                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = 'gps_link'

                gps_msg.latitude  = gps_row['latitude.data']
                gps_msg.longitude = gps_row['longitude.data']
                gps_msg.altitude  = gps_row['altitude.data']

                self.gps_pub.publish(gps_msg)
                self.gps_idx += 1

        # Stop when all data published
        if self.imu_idx >= len(self.imu_data) and self.gps_idx >= len(self.gps_data):
            self.get_logger().info('All data published!')
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = CSVPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()