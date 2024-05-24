#!/usr/bin/env python3


import rospy

from sensor_msgs.msg import LaserScan
import numpy as np

from std_msgs.msg import Float32MultiArray, Float32

def callback(msg : LaserScan):
    beam_angle = 30
    beam_offset = 2

    left : list = msg.ranges[360-(beam_angle-beam_offset+1):360-beam_offset] 
    right : list =  msg.ranges[beam_offset:beam_angle + beam_offset]

    left_min = left.index(min(left))
    left_max = left.index(max(left))

    right_min = right.index(min(right))
    right_max = right.index(min(right))


    angle = 0

    msg = Float32(angle)
    pub.publish(msg)

    


def main():
    rospy.init_node("lds_distance", anonymous=True)

    global pub
    pub = rospy.Publisher("/distance", Float32, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()



if __name__ == "__main__":
    main()