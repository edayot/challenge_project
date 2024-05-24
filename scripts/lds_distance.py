#!/usr/bin/env python3


import rospy

from sensor_msgs.msg import LaserScan
import numpy as np

from std_msgs.msg import Float32MultiArray, Float32

def callback(msg : LaserScan):
    beam_angle = 30

    front_list : list = msg.ranges[360-(beam_angle+1):] + msg.ranges[:beam_angle]

    front_list = [x if x < 0.8 else 0 for x in front_list]

    # set 0 to the middle of the list
    n = len(front_list)
    offset = 5
    for i in range(n):
        if n/2 - offset < i < n/2 + offset:
            front_list[i] = 0
        


    

    max_front_index = front_list.index(max(front_list))

    angle = 0 - len(front_list) // 2 + max_front_index

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