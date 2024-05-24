from typing import TypedDict
import numpy as np


class Point(TypedDict):
    x: float
    y: float
    distance: float
    angle: float

def distance_points(p1 : Point, p2 : Point):
    return np.sqrt((p1["x"] - p2["x"])**2 + (p1["y"] - p2["y"])**2)



def r_angle_to_xy(r, angle):
    x = r * np.cos(angle)
    y = r * np.sin(angle)
    return x, y

def xy_to_r_angle(x, y):
    r = np.sqrt(x**2 + y**2)
    angle = np.arctan2(y, x)
    return r, angle
