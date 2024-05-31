#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np


LAST_ANGLE = 0
LAST_DISTANCE = np.inf


def callback(msg : Float32):
    global LAST_ANGLE
    LAST_ANGLE = msg.data

def callback_stop(msg : Float32):
    global LAST_DISTANCE
    LAST_DISTANCE = msg.data




def main():
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.init_node("control", anonymous=True)

    rospy.Subscriber("/distance", Float32, callback)
    rospy.Subscriber("/stop_robot", Float32, callback_stop)

    v_angular_factor = 1/25

    while not rospy.is_shutdown():
        msg = Twist()
        
        
        v_angular = LAST_ANGLE * v_angular_factor

        v_linear = 0.2
        if LAST_DISTANCE < 0.2:
            v_linear = 0
            v_angular = 0
        msg.linear.x = v_linear
        msg.angular.z = v_angular

        pub.publish(msg)



if __name__ == "__main__":
    main()


