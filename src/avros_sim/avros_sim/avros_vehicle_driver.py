"""AVROS Webots driver for the native differential-drive vehicle.

The Webots model exposes one powered track on each side.  The driver converts
the commanded body twist into left/right track belt velocities, matching the
kinematics used by the real vehicle's actuator node.
"""

import rclpy
import rclpy.parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from math import cos, sin
from rclpy.qos import QoSProfile, ReliabilityPolicy

TRACK_WIDTH = 0.7366  # Production vehicle track gauge, in metres


class AvrosVehicleDriver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        timestep = int(self.__robot.getBasicTimeStep())

        # One independently controlled wheel on each side is the differential
        # drive pair; there is no steering subsystem in this model.
        self.__left_motor = self.__robot.getDevice('left_drive')
        self.__right_motor = self.__robot.getDevice('right_drive')

        # Track LinearMotors use belt speed in metres per second.  Position
        # infinity selects continuous velocity mode, just as for a HingeMotor.
        for motor in [self.__left_motor, self.__right_motor]:
            if motor:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)

        # IMU devices
        self.__inertial_unit = self.__robot.getDevice('imu_inertial')
        self.__gyro = self.__robot.getDevice('imu_gyro')
        self.__accel = self.__robot.getDevice('imu_accel')
        self.__inertial_unit.enable(timestep)
        self.__gyro.enable(timestep)
        self.__accel.enable(timestep)

        rclpy.init(args=None)
        self.__node = rclpy.create_node(
            'avros_vehicle_driver',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', value=True),
            ],
        )
        self.__node.create_subscription(
            Twist, 'cmd_vel', self.__cmd_vel_callback, 1
        )
        self.__imu_pub = self.__node.create_publisher(Imu, '/imu/data', 10)
        self.__odom_pub = self.__node.create_publisher(
            Odometry,
            '/wheel_odom',
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        self.__linear_velocity = 0.0
        self.__angular_velocity = 0.0
        self.__odom_x = 0.0
        self.__odom_y = 0.0
        self.__odom_yaw = 0.0
        self.__last_step_time = self.__robot.getTime()

    def __cmd_vel_callback(self, msg):
        self.__linear_velocity = msg.linear.x
        self.__angular_velocity = msg.angular.z

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        speed = self.__linear_velocity
        angular_velocity = self.__angular_velocity

        # Differential-drive inverse kinematics, matching actuator_node.py.
        left_speed = speed - angular_velocity * TRACK_WIDTH / 2.0
        right_speed = speed + angular_velocity * TRACK_WIDTH / 2.0
        if self.__left_motor:
            self.__left_motor.setVelocity(left_speed)
        if self.__right_motor:
            self.__right_motor.setVelocity(right_speed)

        # Publish combined IMU
        self.__publish_imu()
        self.__publish_odometry(speed, angular_velocity)

    def __publish_imu(self):
        # getQuaternion() returns [x, y, z, w] in Webots ENU frame (R2025a default)
        # which maps directly to ROS ENU convention — no frame conversion needed
        q = self.__inertial_unit.getQuaternion()
        gyro = self.__gyro.getValues()
        accel = self.__accel.getValues()

        msg = Imu()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01
        msg.angular_velocity_covariance[0] = 0.01
        msg.angular_velocity_covariance[4] = 0.01
        msg.angular_velocity_covariance[8] = 0.01
        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1

        self.__imu_pub.publish(msg)

    def __publish_odometry(self, speed, angular_velocity):
        """Publish the velocity odometry consumed by both EKF instances."""
        now = self.__robot.getTime()
        dt = max(0.0, now - self.__last_step_time)
        self.__last_step_time = now

        self.__odom_yaw += angular_velocity * dt
        self.__odom_x += speed * cos(self.__odom_yaw) * dt
        self.__odom_y += speed * sin(self.__odom_yaw) * dt

        stamp = self.__node.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.__odom_x
        msg.pose.pose.position.y = self.__odom_y
        msg.pose.pose.orientation.z = sin(self.__odom_yaw / 2.0)
        msg.pose.pose.orientation.w = cos(self.__odom_yaw / 2.0)
        msg.twist.twist.linear.x = speed
        msg.twist.twist.angular.z = angular_velocity
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.1
        msg.twist.covariance[0] = 0.05
        msg.twist.covariance[35] = 0.05
        self.__odom_pub.publish(msg)
