#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf_conversions
from tf2_ros import TransformBroadcaster, TransformStamped

class SimpleController():

    def __init__(self, wheel_separation, wheel_radius):
        self.wheel_separation_ = wheel_separation
        self.wheel_radius_ = wheel_radius

        rospy.loginfo("Simple Controller Initialized")
        rospy.loginfo("Wheel Seperation: %f, Wheel radius: %f", self.wheel_separation_, self.wheel_radius_)

        self.wheel_radius_ = wheel_radius
        self.wheel_separation_ = wheel_separation

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = rospy.Time.now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
    

        rospy.loginfo("Simple Controller Initialization Complete")

        # Define the publishers for the wheel velocities
        self.fw_right_vel_pub_ = rospy.Publisher("/fw_left_motor/command", Float64, queue_size=10)
        self.fw_left_vel_pub_ = rospy.Publisher("/fw_right_motor/command", Float64, queue_size=10)
        self.rw_right_vel_pub_ = rospy.Publisher("/rw_left_motor/command", Float64, queue_size=10)
        self.rw_left_vel_pub_ = rospy.Publisher("/rw_right_motor/command", Float64, queue_size=10)

        # Define the subscribers for recieving the robot velocity from the joystick/keyboard
        self.vel_sub = rospy.Subscriber("/chromebot_controller/cmd_vel", Twist, self.cmd_vel_callback)

        #Define the subscriber for the joint states of the wheels
        self.joint_state_sub_ = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)    

        # Define a publisher for updating the odom topic
        self.odom_pub_ = rospy.Publisher("chromebot_controller/odom", Odometry, queue_size=10)

        #Define the odometry message
        self.odom_msg_ = Odometry()

        # Define the transform broadcaster
        self.br_ = TransformBroadcaster()
        self.br_msg_ = TransformStamped()
        self.br_msg_.header.frame_id = "odom"
        self.br_msg_.child_frame_id = "base_footprint"


        #Initialize the odometry message
        self.odom_msg_.header.stamp = rospy.Time.now()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        
        self.odom_msg_.pose.pose.position.x = 0.0
        self.odom_msg_.pose.pose.position.y = 0.0
        self.odom_msg_.pose.pose.position.z = 0.0

        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.odom_msg_.twist.twist.linear.x = 0.0
        self.odom_msg_.twist.twist.linear.y = 0.0
        self.odom_msg_.twist.twist.linear.z = 0.0

        self.odom_msg_.twist.twist.angular.x = 0.0
        self.odom_msg_.twist.twist.angular.y = 0.0
        self.odom_msg_.twist.twist.angular.z = 0.0



        #Define the jacobian matrix
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                            [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        
        rospy.loginfo("Speed conversion matrix: %s", self.speed_conversion_)

    def cmd_vel_callback(self, msg):
        # Extract the linear and angular velocities from the message
        robot_velocity = np.array([[msg.linear.x], [msg.angular.z]])

        # Compute the wheel velocities
        wheel_velocities = np.matmul(np.linalg.inv(self.speed_conversion_), robot_velocity)

        right_speed = Float64(wheel_velocities[0,0])
        left_speed = Float64(wheel_velocities[1,0])

        self.fw_right_vel_pub_.publish(right_speed/2)
        self.fw_left_vel_pub_.publish(left_speed/2)
        self.rw_right_vel_pub_.publish(right_speed/2)
        self.rw_left_vel_pub_.publish(left_speed/2)


    def joint_state_callback(self, msg):
        dp_left = msg.position[0] - self.left_wheel_prev_pos_
        dp_right = msg.position[1] - self.right_wheel_prev_pos_
        dt = (msg.header.stamp - self.prev_time_).to_sec()

        self.left_wheel_prev_pos_ = msg.position[0]
        self.right_wheel_prev_pos_ = msg.position[1]
        self.prev_time_ = msg.header.stamp

        phi_left = dp_left/dt
        phi_right = dp_right/dt

        linear_velocity = (phi_right + phi_left)*self.wheel_radius_/2
        angular_velocity = (phi_right - phi_left)*self.wheel_radius_/self.wheel_separation_

        rospy.loginfo("Linear velocity: %f, Angular velocity: %f", linear_velocity, angular_velocity)


        # Computing the pose of the robot
        d_s = self.wheel_radius_* (dp_left + dp_right)/2
        d_theta = self.wheel_radius_* (dp_right - dp_left)/self.wheel_separation_

        self.theta_ += d_theta
        self.x_ += d_s*np.cos(self.theta_)
        self.y_ += d_s*np.sin(self.theta_)
        rospy.loginfo("X: %f, Y: %f, Theta: %f", self.x_, self.y_, self.theta_)

        # Update the odometry message
        self.odom_msg_.header.stamp = msg.header.stamp
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.position.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear_velocity
        self.odom_msg_.twist.twist.linear.y = 0.0
        self.odom_msg_.twist.twist.linear.z = 0.0
        self.odom_msg_.twist.twist.angular.x = 0.0
        self.odom_msg_.twist.twist.angular.y = 0.0
        self.odom_msg_.twist.twist.angular.z = angular_velocity

        self.odom_pub_.publish(self.odom_msg_)

        # Update the transform
        self.br_msg_.transform.translation.x = self.x_
        self.br_msg_.transform.translation.y = self.y_
        self.br_msg_.transform.translation.z = 0.0
        self.br_msg_.transform.rotation.x = q[0]
        self.br_msg_.transform.rotation.y = q[1]
        self.br_msg_.transform.rotation.z = q[2]
        self.br_msg_.transform.rotation.w = q[3]
        self.br_msg_.header.stamp = rospy.Time.now()
        self.br_.sendTransform(self.br_msg_)








