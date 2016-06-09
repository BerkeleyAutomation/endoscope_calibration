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
    pos = [pt[0], pt[1], pt[2], 0.704583065311, 0.590342398526, 0.387353243821, 0.0708238736684]
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

    #Initialize everything    
    psm1 = robot.robot("PSM1")
    f3 = open("calibration_data/camera_matrix.p", "rb")
    cmat = pickle.load(f3)
    testpts = load_camera_points()
    cpoint = testpts[24,:]

    #Camera 3d Point
    cpoint = (-0.05529828068368612, -0.0065449124598436024, 0.13959772667195353)

    #Rigid transformation -> Move to point in Robot Frame
    pt = np.ones(4)
    pt[:3] = cpoint
    pred = cmat * np.matrix(pt).T
    # move_to(pred)
    time.sleep(2)

    left, right = load_images()
    plt.imshow(left)
    plt.show()
    plt.imshow(right)
    plt.show()

