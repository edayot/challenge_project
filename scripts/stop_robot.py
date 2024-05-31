#!/usr/bin/env python3


import rospy

from sensor_msgs.msg import LaserScan
import numpy as np

from std_msgs.msg import Float32MultiArray, Float32

def callback(msg : LaserScan):
    beam_angle = 2

    front_list : list = msg.ranges[360-(beam_angle+1):] + msg.ranges[:beam_angle]


    msg = Float32(min(front_list))
    pub.publish(msg)

    


def main():
    rospy.init_node("stop_robot", anonymous=True)

    global pub
    pub = rospy.Publisher("/stop_robot", Float32, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()



if __name__ == "__main__":
    main()