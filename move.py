import rospy, pickle, time
import robot
from geometry_msgs.msg import Pose
import numpy as np
from rigid_transformation import *
import PyKDL
import cv2
import matplotlib.pyplot as plt
import sys

"""
Test script for moving the robot/verifying camera calibration accuracy.
"""

def move_to(pt):
    pos = [pt[0], pt[1], pt[2], 0.904224639361, 0.153280574192, -0.180361018743, -0.355461348203]
    rotation = PyKDL.Rotation.Quaternion(pos[3], pos[4], pos[5], pos[6])
    position = PyKDL.Vector(pos[0], pos[1], pos[2])
    fr = PyKDL.Frame(rotation, position)
    # psm1.open_gripper(-30)
    psm1.move_cartesian_frame(fr)
 
def load_images():
    return cv2.imread('images/left0.jpg'), cv2.imread('images/right0.jpg')

def load_camera_points():
    lst = []
    f3 = open("calibration_data/endoscope_points.p", "rb")
    pos1 = pickle.load(f3)
    lst.append(pos1)
    while True:
        try:
            pos1 = pickle.load(f3)
            lst.append(pos1)
        except EOFError:
            f3.close()
            return np.matrix(lst[0])

def home_robot():
    pos = [0.023580864372, 0.00699340564912, -0.0485527311586]
    move_to(pos)
    time.sleep(2)


if __name__ == '__main__':

    # left, right = load_images()
    # plt.imshow(np.asarray(left))
    # plt.show()
    # plt.imshow(np.asarray(right))
    # plt.show()
    # sys.exit()



    # Initialize everything    
    psm1 = robot.robot("PSM1")
    # psm1.close_gripper()

    f3 = open("calibration_data/camera_matrix.p", "rb")
    cmat = pickle.load(f3)
    print "camera matrix", cmat
    cpts = load_camera_points()

    #Camera 3d Point
    cpoint = (-0.018274445810247588, -0.0028771206338445974, 0.088643687702081211)
    cpoint = (0.0027797521195478464, 0.010301199936929009, 0.12342119489234789)
    cpoint = (-0.0024282700696836501, -0.01000298049768312, 0.13110752475247525)
    # cpoint = cpts[12]
    cpoint = (-0.0110894, 0.0184677, 0.170577595)
    print cpts
    print "camera point", cpoint
    pt = np.ones(4)
    pt[:3] = cpoint
    pred = cmat * np.matrix(pt).T
    print pred
    # move_to(pred)
    sys.exit()
    #Rigid transformation -> Move to point in Robot Frame


    for i in range(25):
        home_robot()
        cpoint = cpts[i]
        pt = np.ones(4)
        pt[:3] = cpoint
        pred = cmat * np.matrix(pt).T
        print pred
        move_to(pred)
        time.sleep(2)

    # print pred
    # move_to(pred)
