#!/usr/bin/env python3


import rospy

from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Float32MultiArray

# print the list of all accessible packages
import os


from utils import Point, distance_points, r_angle_to_xy, xy_to_r_angle




def publish_middle_points(points_grouped : list[list[Point]], msg : LaserScan):
    # find the middle point of each group
    middle_points : list[Point] = []
    for i,group in enumerate(points_grouped):
        x = np.mean([p["x"] for p in group])
        y = np.mean([p["y"] for p in group])
        distance, angle = xy_to_r_angle(x, y)
        angle = np.rad2deg(angle)
        angle = int(angle)
        angle = angle % 360
        middle_points.append({
            "x": x,
            "y": y,
            "distance": distance,
            "angle": angle
        })
    # publish the middle points as a LaserScan message
    msg.ranges = []
    for i in range(360):
        # get the middle point if it exists
        for point in middle_points:
            if int(point["angle"]) == i:
                msg.ranges.append(point["distance"])
                break
        else:
            msg.ranges.append(np.inf)
    pub.publish(msg)
    



def callback(msg : LaserScan):
    """
    Goal : Filter the laser scan data by grouping points that are close to each other

    Behavior:
    - Group each point by proximity to each other 
    - Merge groups that have points close to each other 
        (depending on the order that the points are processed, they may have two groups that are close to each other)
    - Delete empty groups (groups that have been merged with another group)

    Note:
        Also publish the middle points of each group as a LaserScan message (for debugging purposes)
    """
    DISTANCE_THRESHOLD = 0.2
    # group each point by proximity to the robot
    points_grouped : list[list[Point]] = []
    for angle, distance in enumerate(msg.ranges):
        if distance == np.inf:
            continue
        x, y = r_angle_to_xy(distance, np.deg2rad(angle))
        point : Point = {
            "x": x,
            "y": y,
            "distance": distance,
            "angle": angle
        }
        found = False
        for group in points_grouped:
            min_distance = distance_points(group[0], point)
            for p in group[1:]:
                d = distance_points(p, point)
                if d < min_distance:
                    min_distance = d
            if min_distance < DISTANCE_THRESHOLD:
                group.append(point)
                found = True
                break
        if not found:
            points_grouped.append([point])
    # merge groups that have points close to each other
    for i,group in enumerate(points_grouped):
        for j in range(i+1, len(points_grouped)):
            for p1 in group:
                for p2 in points_grouped[j]:
                    if distance_points(p1, p2) < DISTANCE_THRESHOLD:
                        group.extend(points_grouped[j])
                        points_grouped[j] = []
                        break
                else:
                    continue
                break
            else:
                continue
            break
    # remove empty groups
    points_grouped = [group for group in points_grouped if group]
    # publish the middle points
    publish_middle_points(points_grouped, msg)
    



def main():
    rospy.init_node("lds_distance", anonymous=True)

    global pub
    pub = rospy.Publisher("/scan_filtered", LaserScan, queue_size=10)

    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()



if __name__ == "__main__":
    main()