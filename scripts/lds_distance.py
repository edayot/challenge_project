#!/usr/bin/env python3


import rospy

from sensor_msgs.msg import LaserScan
import numpy as np

from std_msgs.msg import Float32MultiArray

def callback(msg : LaserScan):
    objective_angle = calc_objective_angle(msg)
    diff_g_d, signed_min_g_d = calc_diff_g_d(msg)
    distance_front = calc_distance_front(msg)
    msg = Float32MultiArray()
    msg.data = [objective_angle, diff_g_d, signed_min_g_d, distance_front]
    pub.publish(msg)

def calc_diff_g_d(msg : LaserScan):
    beam_angle = 10
    left_list : list = msg.ranges[90-beam_angle:90+beam_angle]
    right_list : list = msg.ranges[270-beam_angle:270+beam_angle]

    min_left = min(left_list)
    min_right = min(right_list)

    diff = min_left - min_right
    signed_min_g_d = diff / abs(diff) * min(abs(min_left), abs(min_right))
    return diff, - signed_min_g_d

def calc_distance_front(msg : LaserScan):
    beam_angle = 10
    front_list : list = msg.ranges[360-(beam_angle+1):] + msg.ranges[:beam_angle]

    return min(front_list)



def calc_objective_angle(msg : LaserScan):
    beam_angle = 30
    front_list : list = msg.ranges[360-(beam_angle+1):] + msg.ranges[:beam_angle]

    n_inf = 0
    for i in range(len(front_list)):
        if front_list[i] == np.inf:
            n_inf += 1
    if n_inf > beam_angle:
        return 0    
    front_list = [x if x < 0.8 else 0 for x in front_list]
    front_list = list(front_list)
    n = len(front_list)
    offset = 10
    for i in range(n):
        if n/2 - offset < i < n/2 + offset:
            front_list[i] = 0
    max_front_index = front_list.index(max(front_list))
    angle = 0 - len(front_list) // 2 + max_front_index
    return angle



def main():
    rospy.init_node("lds_distance", anonymous=True)

    global pub
    pub = rospy.Publisher("/distance", Float32MultiArray, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()



if __name__ == "__main__":
    main()