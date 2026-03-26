import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
import numpy as np
from pyproj import Proj
import csv
import os


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # ─────────────────────────────────────────
        # EKF INITIALIZATION
        # ─────────────────────────────────────────
        self.x = np.zeros(5)        # state vector [x, y, vx, vy, yaw]
        self.P = np.eye(5) * 500    # initial covariance

        self.dt = 0.025             # 40Hz IMU rate

        # State transition matrix F
        self.F = np.array([[1, 0, self.dt, 0,       0],
                           [0, 1, 0,       self.dt, 0],
                           [0, 0, 1,       0,       0],
                           [0, 0, 0,       1,       0],
                           [0, 0, 0,       0,       1]])

        # Measurement matrix H — GPS sees x and y only
        self.H = np.array([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0]])

        # Process noise Q — how much we distrust IMU
        self.Q = np.diag([0.001, 0.001, 0.01, 0.01, 0.0001])

        # Measurement noise R — how much we distrust GPS
        self.R = np.diag([5.0, 5.0])

        self.I = np.eye(5)

        # UTM projection — converts lat/lon to meters
        self.proj = Proj(proj='utm', zone=19, ellps='WGS84')

        # Track if we initialized x with first GPS reading
        self.initialized = False

        # ─────────────────────────────────────────
        # CSV OUTPUT — save EKF results
        # ─────────────────────────────────────────
        self.output_file = open(
            '/home/rian/ros2_ws/src/ekf_gps_imu/data/ekf_output.csv', 'w')
        self.csv_writer = csv.writer(self.output_file)
        self.csv_writer.writerow(['time', 'ekf_x', 'ekf_y', 'vx', 'vy', 'yaw'])

        # ─────────────────────────────────────────
        # SUBSCRIBERS
        # ─────────────────────────────────────────
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)

        # ─────────────────────────────────────────
        # PUBLISHER — filtered position output
        # ─────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/ekf/odom', 10)

        self.get_logger().info('EKF Node started!')

    # ─────────────────────────────────────────
    # IMU CALLBACK — runs predict step
    # ─────────────────────────────────────────
    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        gz = msg.angular_velocity.z

        # Run predict step
        self.x, self.P = self.predict(self.x, self.P, ax, ay, gz)

        # Publish current estimate
        self.publish_odom()

    # ─────────────────────────────────────────
    # GPS CALLBACK — runs update step
    # ─────────────────────────────────────────
    def gps_callback(self, msg):
        # Convert lat/lon to UTM x,y meters
        utm_x, utm_y = self.proj(msg.longitude, msg.latitude)

        # Initialize state with first GPS reading
        if not self.initialized:
            self.x[0] = utm_x
            self.x[1] = utm_y
            self.initialized = True
            self.get_logger().info(
                f'EKF initialized at x={utm_x:.2f}, y={utm_y:.2f}')
            return

        # Run update step
        self.x, self.P = self.update(self.x, self.P, utm_x, utm_y)

    # ─────────────────────────────────────────
    # EKF PREDICT
    # ─────────────────────────────────────────
    def predict(self, x, P, ax, ay, gz):
        # Equation 1 — move state forward using physics
        x = self.F @ x

        # Add IMU input
        x[2] += ax * self.dt
        x[3] += ay * self.dt
        x[4] += gz * self.dt

        # Equation 2 — grow uncertainty
        P = self.F @ P @ self.F.T + self.Q

        return x, P

    # ─────────────────────────────────────────
    # EKF UPDATE
    # ─────────────────────────────────────────
    def update(self, x, P, gps_x, gps_y):
        # GPS measurement vector
        z = np.array([gps_x, gps_y])

        # Equation 3 — Kalman Gain
        S = self.H @ P @ self.H.T + self.R
        K = P @ self.H.T @ np.linalg.inv(S)

        # Equation 4 — correct state using GPS
        innovation = z - self.H @ x
        x = x + K @ innovation

        # Equation 5 — uncertainty shrinks
        P = (self.I - K @ self.H) @ P

        return x, P

    # ─────────────────────────────────────────
    # PUBLISH FILTERED POSITION
    # ─────────────────────────────────────────
    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        # Position
        msg.pose.pose.position.x = self.x[0]
        msg.pose.pose.position.y = self.x[1]
        msg.pose.pose.position.z = 0.0

        # Yaw → Quaternion
        yaw = self.x[4]
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = np.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = np.cos(yaw / 2.0)

        # Velocity
        msg.twist.twist.linear.x = self.x[2]
        msg.twist.twist.linear.y = self.x[3]

        # Save to CSV
        self.csv_writer.writerow([
            self.get_clock().now().nanoseconds,
            self.x[0], self.x[1],
            self.x[2], self.x[3],
            self.x[4]
        ])

        self.odom_pub.publish(msg)

    # ─────────────────────────────────────────
    # CLEANUP
    # ─────────────────────────────────────────
    def destroy_node(self):
        self.output_file.close()
        self.get_logger().info('EKF output saved!')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()