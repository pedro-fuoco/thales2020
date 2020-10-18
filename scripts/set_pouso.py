#!/usr/bin/env python
import rospy
from MAV import MAV
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped

def run():

    rospy.init_node("head")
    cv_control_publisher = rospy.Publisher("/cv_detection/set_running_state", Bool, queue_size=10)
    mav = MAV("1")
    height = 1

    """ ARM AND TAKE OFF"""
    mav.takeoff(height)
    height = 1
    mav.set_position(0, 0, height)
    while not mav.chegou():
        rospy.logwarn("going to the center")
        mav.set_position(0, 0, height)

    """ INITIALIZE CV_CONTROL """
    for i in range (10):
        cv_control_publisher.publish(Bool(True))
        mav.rate.sleep()

if __name__ == "__main__":
    run()
    