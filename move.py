import rospy, pickle, time
import robot
from geometry_msgs.msg import Pose
import numpy as np
from rigid_transformation import *
import PyKDL
import cv2
import matplotlib.pyplot as plt

"""
Test script for moving the robot/verifying camera calibration accuracy.
"""

def move_to(pt):
    pos = [pt[0], pt[1], pt[2], 0.904224639361, 0.153280574192, -0.180361018743, -0.355461348203]
    rotation = PyKDL.Rotation.Quaternion(pos[3], pos[4], pos[5], pos[6])
    position = PyKDL.Vector(pos[0], pos[1], pos[2])
    fr = PyKDL.Frame(rotation, position)
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


if __name__ == '__main__':

    # left, right = load_images()
    # plt.imshow(np.asarray(left))
    # plt.show()
    # plt.imshow(np.asarray(right))
    # plt.show()
    # import sys
    # sys.exit()



    # Initialize everything    
    psm1 = robot.robot("PSM1")
    psm1.close_gripper()

    f3 = open("calibration_data/camera_matrix.p", "rb")
    cmat = pickle.load(f3)
    print "camera matrix", cmat
    cpts = load_camera_points()

    #Camera 3d Point
    cpoint = (-0.028559456491671593, -0.020578925928518597, 0.12828773493509008) # top, second from left
    cpoint = (-0.038262278312046948, -0.020344382769890135, 0.1234131429583306) # top left
    cpoint = (-0.039982414769834149, 0.00078298402327594879, 0.12663153868222243) # third row, left

    cpoint = cpts[20]
    print "camera point", cpoint

    #Rigid transformation -> Move to point in Robot Frame
    pt = np.ones(4)
    pt[:3] = cpoint
    pred = cmat * np.matrix(pt).T

    print pred
    # move_to(pred)
