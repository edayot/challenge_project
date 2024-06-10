#!/usr/bin/env python3


import rospy

from geometry_msgs.msg import Twist
import pandas as pd
import pathlib

DATAFRAME = pd.DataFrame(columns=["time","lx","ly","lz","ax","ay","az"])

def callback(msg : Twist):
    global DATAFRAME
    t = rospy.get_time()
    lx = msg.linear.x
    ly = msg.linear.y
    lz = msg.linear.z
    ax = msg.angular.x
    ay = msg.angular.y
    az = msg.angular.z
    DATAFRAME = pd.concat([DATAFRAME,pd.DataFrame([[t,lx,ly,lz,ax,ay,az]],columns=["time","lx","ly","lz","ax","ay","az"])])


    
    


def main():
    rospy.init_node("recording", anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, callback)
    while not rospy.is_shutdown():
        pass
    DATAFRAME.to_csv(pathlib.Path(__file__).parent.parent/"data.csv",index=False)




if __name__ == "__main__":
    main()