#!/usr/bin/env python3


import rospy

from sensor_msgs.msg import LaserScan
import numpy as np

from std_msgs.msg import Float32MultiArray, Float32

def callback(msg : LaserScan):
    beam_angle = 30

    front_list : list = msg.ranges[360-(beam_angle+1):] + msg.ranges[:beam_angle]

    beam_angle = 10
    back_list : list = msg.ranges[180-beam_angle:180+beam_angle]
    left_list : list = msg.ranges[90-beam_angle:90+beam_angle]
    right_list : list = msg.ranges[270-beam_angle:270+beam_angle]

    min_left = min(left_list)
    min_right = min(right_list)

    n_infinity =  0
    for i in range(len(front_list)):
        if front_list[i] == np.inf:
            n_infinity += 1
    front_list = [x if x < 0.8 else 0 for x in front_list]
    front_list = list(front_list)

    # set 0 to the middle of the list
    n = len(front_list)
    offset = 1
    for i in range(n):
        if n/2 - offset < i < n/2 + offset:
            front_list[i] = 0

    max_front_index = front_list.index(max(front_list))

    angle = 0 - len(front_list) // 2 + max_front_index
    angle *= 2

    if max(front_list) == 0:
        angle = 0

    diff = min_left - min_right
    if diff != np.inf:
        angle = angle + diff * 200
    if abs(min_left) < 0.15:
        angle = - min_left * 30
    if abs(min_right) < 0.15:
        angle = min_right * 30

    if np.isnan(angle):
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