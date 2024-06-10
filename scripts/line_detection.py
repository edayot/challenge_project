#!/usr/bin/env python3


import rospy

from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Float32


AS_BEEN_ON_LINE = False

def apply_color_filter(color : tuple[float,float,float]) -> bool:
    r,g,b = color
    if r == b == g:
        return 0
    if g > 200 and r < 20 and b < 180:
        return 1
    return 0
    
    

def callback(msg : Image):
    global AS_BEEN_ON_LINE
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    a = False
    for i in range(img.shape[0]-1,0,-1):
        for j in range(img.shape[1]-1,0,-1):
            if apply_color_filter(img[i][j]) == 1:
                a = True
                break
        if a:
            break
    if a:
        index = j - img.shape[1]//2
        h = img.shape[0] - i
        if h < 10:
            AS_BEEN_ON_LINE = True
        if h >= 105 and not AS_BEEN_ON_LINE:
            return
        msg = Float32(index / h **(1/3))
        pub.publish(msg)
    if not a and AS_BEEN_ON_LINE:
        AS_BEEN_ON_LINE = False
        msg = Float32(0)
        pub.publish(msg)


    


def main():
    rospy.init_node("line_detection", anonymous=True)

    global pub
    pub = rospy.Publisher("/wanted_angle_line_detection", Float32, queue_size=10)

    rospy.Subscriber("/camera/image", Image, callback)
    rospy.spin()




if __name__ == "__main__":
    main()