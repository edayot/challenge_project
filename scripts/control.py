#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np


LAST_ANGLE = 0
LAST_DISTANCE = np.inf

LAST_LINE_DETECTION = None


def callback(msg : Float32):
    global LAST_ANGLE
    LAST_ANGLE = msg.data

def callback_stop(msg : Float32):
    global LAST_DISTANCE
    LAST_DISTANCE = msg.data

def callback_line_detection(msg : Float32):
    global LAST_LINE_DETECTION
    LAST_LINE_DETECTION = msg.data




def main():
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.init_node("control", anonymous=True)

    rospy.Subscriber("/distance", Float32, callback)
    rospy.Subscriber("/stop_robot", Float32, callback_stop)
    rospy.Subscriber("/line_detection", Float32, callback_line_detection)

    v_angular_factor = 1/25

    while not rospy.is_shutdown():
        msg = Twist()
        
        if LAST_LINE_DETECTION is None:
            v_angular = LAST_ANGLE * v_angular_factor

            v_linear = 0.2
            if LAST_DISTANCE < 0.2:
                v_linear = 0
                v_angular = 0
            msg.linear.x = v_linear
            msg.angular.z = v_angular

            pub.publish(msg)
        else:
            msg.linear.x = 0.4
            msg.angular.z = - LAST_LINE_DETECTION * v_angular_factor / 2
            pub.publish(msg)



if __name__ == "__main__":
    main()


