#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import click
from std_msgs.msg import Float32MultiArray, String



def callback(msg : Float32MultiArray):
    min_distance = rospy.get_param("distance_before_stop", 1.0)


    if msg.data[0] != -1 and msg.data[0] < min_distance:
        msg = String("front")
        pub.publish(msg)
    elif msg.data[1] != -1 and msg.data[1] < min_distance:
        msg = String("back")
        pub.publish(msg)
    elif msg.data[2] != -1 and msg.data[2] < min_distance:
        msg = String("left")
        pub.publish(msg)
    elif msg.data[3] != -1 and msg.data[3] < min_distance:
        msg = String("right")
        pub.publish(msg)
    
    





def main():

    global pub
    pub = rospy.Publisher("/stop_robot", String, queue_size=10)

    rospy.Subscriber("/distance", Float32MultiArray, callback)

    rospy.init_node("stop_robot", anonymous=True)
    rospy.spin()



if __name__ == "__main__":
    main()