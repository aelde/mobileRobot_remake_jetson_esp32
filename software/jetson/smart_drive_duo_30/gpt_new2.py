import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose2D, TransformStamped
from std_msgs.msg import Float32, Int32
from nav_msgs.msg import Odometry
import tf2_ros
import math


class Robot(Node):
    def __init__(self):
        super().__init__('robot_core_node')
        self.wheel_radius = 0.04  # m
        # self.wheelbase = 0.223 #m
        self.wheelbase = 0.28

        self.max_rpm = 350.0  # rpm

        # use speed 0.39 turn 3.13 in cmd_vel rpm = 84 -> กำลังพอดี

        self.vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.vel_callback, 10)

        self.leftwheel_pub = self.create_publisher(Float32, 'cmd_rpm_FL', 10)
        self.rightwheel_pub = self.create_publisher(Float32, 'cmd_rpm_FR', 10)

        self.v = 0.0
        self.w = 0.0

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.cmd_l = Float32()
        self.cmd_r = Float32()

        self.FL_tick_sub = self.create_subscription(
            Int32, 'FL_tick', self.FR_tick_callback, 10)
        self.FR_tick_sub = self.create_subscription(
            Int32, 'FR_tick', self.FL_tick_callback, 10)

        self.FL_tick = {'bf': None, 'now': None}
        self.FR_tick = {'bf': None, 'now': None}
        self.FL_tick_ts = None
        self.FR_tick_ts = None

        self.last_time = self.get_clock().now()

        # odometry
        self.tick_per_rev = 330
        self.robot_pose = Pose2D()  # x , y , yaw
        self.dx = 0.0
        self.dy = 0.0
        self.dtheta = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF
        self.tf_boardcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("robot core is running")

    def FL_tick_callback(self, FL_tick_in):
        self.FL_tick['now'] = FL_tick_in.data
        print("FL_tick_in : ", FL_tick_in.data, end=' ')
        self.FL_tick_ts = self.get_clock().now()

    def FR_tick_callback(self, FR_tick_in):
        self.FR_tick['now'] = FR_tick_in.data
        print("FR_tick_in : ", FR_tick_in.data)
        self.FR_tick_ts = self.get_clock().now()

    def get_tick(self):
        tick = {}
        tick['FL'] = self.FL_tick['now']
        tick['FR'] = self.FR_tick['now']
        tick['FL_ts'] = self.FL_tick_ts
        tick['FR_ts'] = self.FR_tick_ts

        return tick

    def vel_callback(self, vel):
        print("callback function")
        self.v = vel.linear.x
        self.w = vel.angular.z

        L = self.wheelbase
        R = self.wheel_radius

        v_r = ((2.0 * self.v) + (self.w * L)) / (2.0 * R)
        v_l = ((2.0 * self.v) - (self.w * L)) / (2.0 * R)

        self.wheel_speed_setpoint(v_r, v_l)

    def timer_callback(self):
        self.leftwheel_pub.publish(self.cmd_l)
        self.rightwheel_pub.publish(self.cmd_r)
        self.odometry()

    def wheel_speed_setpoint(self, vr, vl):
        rpm_r = vr * 9.549297
        rpm_l = vl * 9.549297  # 1 rad/s = 9.549297 rpm

        rpm_r = max(min(rpm_r, self.max_rpm), -self.max_rpm)
        rpm_l = max(min(rpm_l, self.max_rpm), -self.max_rpm)

        if rpm_r == 0.0:
            self.cmd_r.data = 0.0
        if rpm_l == 0.0:
            self.cmd_l.data = 0.0

        self.cmd_r.data = rpm_r
        self.cmd_l.data = rpm_l

        self.get_logger().info('out left rpm: %s' % rpm_l)
        self.get_logger().info('out right rpm: %s' % rpm_r)

    def quanterion_to_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw*0.5)
        sy = math.sin(yaw*0.5)
        cp = math.cos(pitch*0.5)
        sp = math.sin(pitch*0.5)
        cr = math.cos(roll*0.5)
        sr = math.sin(roll*0.5)
        q = [0]*4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    def odometry(self):
        ts = self.get_clock().now()
        tick = self.get_tick()
        dt = ts - self.last_time  # nanosec
        dt = dt.nanoseconds * 1e-9  # sec

        if tick['FL'] == None or tick['FR'] == None:
            return

        if self.FL_tick['bf'] is None or self.FR_tick['bf'] is None:
            self.FL_tick['bf'] = tick['FL']
            self.FR_tick['bf'] = tick['FR']

        prev_robot_pose = self.robot_pose
        R = self.wheel_radius
        L = self.wheelbase
        TPR = self.tick_per_rev  # tick per rev

        DPT = (2 * math.pi * R) / TPR  # distance per tick

        DL = DPT * (tick['FL'] - self.FL_tick['bf'])
        DR = DPT * (tick['FR'] - self.FR_tick['bf'])

        DC = (DL + DR) * 0.5  # distance center

        self.dx = DC * math.cos(prev_robot_pose.theta)
        self.dy = DC * math.sin(prev_robot_pose.theta)
        self.dtheta = (DL - DR) / L

        new_robot_pose = Pose2D()
        new_robot_pose.x = prev_robot_pose.x + self.dx
        new_robot_pose.y = prev_robot_pose.y + self.dy
        new_robot_pose.theta = prev_robot_pose.theta + self.dtheta

        self.robot_pose.x = new_robot_pose.x
        self.robot_pose.y = new_robot_pose.y
        self.robot_pose.theta = new_robot_pose.theta

        self.FL_tick['bf'] = tick['FL']
        self.FR_tick['bf'] = tick['FR']

        # publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.robot_pose.x
        odom.pose.pose.position.y = self.robot_pose.y
        odom.pose.pose.position.z = 0.0

        quat = self.quanterion_to_euler(0.0, 0.0, self.robot_pose.theta)
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]

        # odom.pose.pose.covariance[0] = 0.01
        # odom.pose.pose.covariance[7] = 0.01
        # odom.pose.pose.covariance[35] = 0.01
        
        self.odom_pub.publish(odom)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.robot_pose.x
        t.transform.translation.y = self.robot_pose.y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = quat[0]
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]
        
        self.tf_boardcaster.sendTransform(t)
        
        self.last_time = ts

def main():
    rclpy.init()

    rb = Robot()
    rclpy.spin(rb)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
