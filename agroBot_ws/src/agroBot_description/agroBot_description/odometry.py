import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from sensor_msgs.msg import NavSatFix

class ImuOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_odometry_node')
        self.odom_pub_ = self.create_publisher(Odometry, 'odom', 10)
        self.imu_sub = self.create_subscription(Imu, 'demo/imu', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, 'gps/data', self.gps_callback, 10)

        # Initial state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.u = 0.0
        self.v = 0.0
        self.r = 0.0
        self.az = 0.0
        self.u_ = 0.0
        self.v_ = 0.0
        self.r_ = 0.0

        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_link"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Fill the TF message
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_link"


    def imu_callback(self, msg):

        self.ax = msg.linear_acceleration.x
        self.ay = msg.linear_acceleration.y
        self.az = msg.orientation.z

    def gps_callback(self, msg):

        
        #self.get_logger().info("z imu %s" % self.az)


        # Get the linear acceleration from IMU
        self.y = msg.latitude
        self.x = msg.longitude
        #az = msg.linear_acceleration.z

        #self.get_logger().info("y %s" % self.y)
        self.get_logger().info("y %s" % (self.y*100000))
        self.get_logger().info("x %s" % (self.x*100000))
        
        self.r_ = self.az
        self.u_ = self.x
        self.v_ = self.y

        # Compose and publish the odom message
        q = quaternion_from_euler(0, 0, self.r_)
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.u_
        self.odom_msg_.pose.pose.position.y = self.v_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = self.u
        self.odom_msg_.twist.twist.angular.z = self.r
        self.odom_pub_.publish(self.odom_msg_)

        # TF
        self.transform_stamped_.transform.translation.x = self.u_
        self.transform_stamped_.transform.translation.y = self.v_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()