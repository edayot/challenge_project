#!/usr/bin/env python3


import rospy

from sensor_msgs.msg import Image
import numpy as np
from rich import inspect
from PIL import Image as PILImage
from std_msgs.msg import Float32


def apply_color_filter(color : tuple[float,float,float]) -> bool:
    if color[0] == color[1] == color[2]:
        return 0
    if color[1] > 30 and color[0] < 20:
        return 1
    return 0
    
    
    
N=0
def callback(msg : Image):
    img = PILImage.frombytes("RGB", (msg.width, msg.height), msg.data)
    img = img.convert("RGB")
    # remove red, blue channel
    img = np.array(img)

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
        msg = Float32(index)
        pub.publish(msg)


    


def main():
    rospy.init_node("camera_analysis", anonymous=True)

    global pub
    pub = rospy.Publisher("/line_detection", Float32, queue_size=10)

    rospy.Subscriber("/camera/image", Image, callback)
    rospy.spin()




if __name__ == "__main__":
    main()