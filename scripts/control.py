#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np


LAST_ANGLE = 0



def callback(msg : Float32):
    global LAST_ANGLE
    LAST_ANGLE = msg.data





def main():
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.init_node("control", anonymous=True)

    rospy.Subscriber("/distance", Float32, callback)

    v_linear = 0.2
    v_angular_factor = 1/25

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = v_linear
        
        v_angular = LAST_ANGLE * v_angular_factor

        msg.angular.z = v_angular

        pub.publish(msg)



if __name__ == "__main__":
    main()


