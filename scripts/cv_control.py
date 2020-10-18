#!/usr/bin/env python
import rospy
from thales2020.msg import H_info
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool

from dynamic_reconfigure.server import Server

from simple_pid import PID

import time
import numpy as np

class VisCon():
    def __init__(self):
        # ROS setup
        rospy.init_node('control')
        self.rate = rospy.Rate(60)

        # ROS Parameters
        self.vel_topic = rospy.get_param("/mavros_velocity_pub")

        # Publishers
        self.vel_pub = rospy.Publisher(self.vel_topic, TwistStamped, queue_size=1)

        # Subscribers
        self.detection_sub = rospy.Subscriber('/cv_detection/detection', H_info, self.detection_callback)
        
        # Attributes
        self.velocity = TwistStamped()
        
        # PIDs
        self.pid_x = PID(-0.01, -0.008, -0.0001)         # size how close the drone is to the H
        self.pid_y = PID(0.01, 0.008, 0.0001)
        self.pid_z = PID(-0.1, -0.001, 0.0001) # Negative parameters (CV's -y -> Frame's +z)
        self.pid_w = PID(0, 0, 0) # Orientation

        self.pid_x.output_limits = self.pid_y.output_limits = (-0.5, 0.5) # output value will be between -0.3 and 0.3
       
        self.pid_z.output_limits = (-0.5, 0.5)  # output value will be between -0.8 and 0.8
    
    def set_goal_pose(self):
        self.pid_x.setpoint = -240.0/2 # y size
        self.pid_y.setpoint = 320.0/2 # x
        self.pid_z.setpoint = 0.01 # 1% of the image area
        self.pid_w.setpoint = 0 # orientation

    def set_goal_vel(self, vx, vy, vz, vw):
        self.velocity.twist.linear.x = vx
        self.velocity.twist.linear.y = vy
        self.velocity.twist.linear.z = vz
        self.velocity.twist.angular.z = vw # not volkswagen

    def detection_callback(self, vector_data):
        self.detection = vector_data
        self.last_time = time.time()

    def run(self):
        self.set_goal_pose()
        start_time = rospy.get_rostime().secs

        while not rospy.is_shutdown() or rospy.get_rostime().secs - start_time <= 100:
            
            try:

                self.velocity.twist.linear.x = self.pid_x(-self.detection.center_y)
                self.velocity.twist.linear.y = self.pid_y(self.detection.center_x)
                self.velocity.twist.linear.z = self.pid_z(-self.detection.area_ratio) # PID z must have negative parameters
                self.velocity.twist.angular.z = 0
            
                self.vel_pub.publish(self.velocity)
                # Assume velocity message will be treated
                    
                rospy.spin()
            
            except:
                rospy.logwarn("ERROR")
                continue

            self.rate.sleep()

        rospy.loginfo("DONE")

if __name__ == "__main__":
    c = VisCon()
    c.run()