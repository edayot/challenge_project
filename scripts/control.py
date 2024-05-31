#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float32MultiArray

from math import sqrt

OBJECTIVE_ANGLE = None
DIFF_G_D = None
SIGNED_MIN_G_D = None
DISTANCE_FRONT = None
FIRST_MSG_RECEIVED = False



def callback(msg : Float32MultiArray):
    global OBJECTIVE_ANGLE, DIFF_G_D, SIGNED_MIN_G_D, DISTANCE_FRONT
    OBJECTIVE_ANGLE, DIFF_G_D, SIGNED_MIN_G_D, DISTANCE_FRONT = msg.data
    global FIRST_MSG_RECEIVED
    FIRST_MSG_RECEIVED = True


def signe(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0


def main():
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.init_node("control", anonymous=True)

    rospy.Subscriber("/distance", Float32MultiArray, callback)

    v_linear = 0.1
    v_angular_factor = 1/20

    DISTANCE_FRONT_DIM = 0.2
    DISTANCE_FRONT_MIN = 0.05
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rate.sleep()
        msg = Twist()
        msg.linear.x = v_linear

        if FIRST_MSG_RECEIVED:
            if DISTANCE_FRONT < DISTANCE_FRONT_DIM:
                msg.linear.x = v_linear * DISTANCE_FRONT / DISTANCE_FRONT_DIM
            if DISTANCE_FRONT < DISTANCE_FRONT_MIN:
                msg.linear.x = 0
            
            mur_coef = DIFF_G_D * 3
            obj_coef = 2*sqrt(abs(OBJECTIVE_ANGLE)) * signe(OBJECTIVE_ANGLE)
            coef = 0
            if abs(mur_coef) > abs(obj_coef):
                coef = mur_coef
            else:
                coef = obj_coef + mur_coef
            if abs(SIGNED_MIN_G_D) < 0.15:
                coef = 1/-SIGNED_MIN_G_D /2
            msg.angular.z = v_angular_factor * (coef)
            pub.publish(msg)

            



if __name__ == "__main__":
    main()


