import image_geometry
import rospy
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
import cv2
import cv_bridge
import numpy as np
import rospy, scipy.misc
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import pickle
import sys
import time
from math import *

def load_robot_points():
    lst = []
    f3 = open("calibration_data/psm1_calibration.p", "rb")
    pos1 = pickle.load(f3)
    lst.append(pos1)
    while True:
        try:
            pos2 = pickle.load(f3)
            lst.append(pos2)
        except EOFError:
            f3.close()
            return np.matrix(lst)

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


def get_points_3d(left_points, right_points, info):
    """ this method assumes that corresponding points are in the right order
        and returns a list of 3d points """

    # both lists must be of the same lenghth otherwise return None
    if len(left_points) != len(right_points):
        rospy.logerror("The number of left points and the number of right points is not the same")
        return None

    points_3d = []
    for i in range(len(left_points)):
        a = left_points[i]
        b = right_points[i]
        disparity = sqrt((a[0]-b[0]) ** 2 + (a[1] - b[1]) ** 2)
        pt = convertStereo(a[0], a[1], disparity, info)
        points_3d.append(pt)
    return points_3d


def convertStereo(u, v, disparity, info):
    """
    Converts two pixel coordinates u and v along with the disparity to give PointStamped       
    """
    stereoModel = image_geometry.StereoCameraModel()
    stereoModel.fromCameraInfo(info['l'], info['r'])
    (x,y,z) = stereoModel.projectPixelTo3d((u,v), disparity)

    cameraPoint = PointStamped()
    cameraPoint.header.frame_id = info['l'].header.frame_id
    cameraPoint.header.stamp = time.time()
    cameraPoint.point = Point(x,y,z)
    return cameraPoint

def solve_for_camera_matrix():
    """
    Returns Camera -> Robot frame matrix
    """
    good_pts = get_good_indices()
    robot_points = load_robot_points()[good_pts,:]
    camera_points = load_camera_points()[good_pts,:]
    camera_mean = camera_points.mean(axis=0)
    robot_mean = robot_points.mean(axis=0)
    for i in range(robot_points.shape[0]):
        robot_points[i,:] -= robot_mean
        camera_points[i,:] -= camera_mean
    X = camera_points.T
    Y = robot_points.T
    covariance = X * Y.T
    U, Sigma, V = np.linalg.svd(covariance)
    V = V.T
    idmatrix = np.identity(3)
    idmatrix[2, 2] = np.linalg.det(V * U.T)
    R = V * idmatrix * U.T
    t = robot_mean.T - R * camera_mean.T
    return np.concatenate((R, t), axis=1)



def solve_for_robot_matrix():
    """
    Returns Robot -> Camera frame matrix
    """
    robot_points = load_robot_points()
    camera_points = load_camera_points()
    #needs to be implemented

def write_mat_to_file(filename, matrix):
    f = open("calibration_data/" + filename, "w")
    pickle.dump(matrix, f)
    f.close()

def pixelto3d(left, right):
    info = {}
    f = open("calibration_data/camera_right.p")
    info['r'] = pickle.load(f)
    f.close()

    f = open("calibration_data/camera_left.p")
    info['l'] = pickle.load(f)
    f.close() 
    pts3d = get_points_3d(left, right, info)
    return np.matrix([(p.point.x, p.point.y, p.point.z) for p in pts3d])

def least_squares_plane_normal(points_3d):
    x_list = points_3d[:,0]
    y_list = points_3d[:,1]
    z_list = points_3d[:,2]

    A = np.concatenate((x_list, y_list, np.ones((len(x_list), 1))), axis=1)
    plane = np.matrix(np.linalg.lstsq(A, z_list)[0]).T

    return plane

def distance_to_plane(m, point):
    A = m[0,0]
    B = m[0,1]
    C = -1
    D = m[0,2]
    p0 = np.array([0,0,D])
    p1 = np.array(point)
    n = np.array([A,B,C])/np.linalg.norm(np.array([A,B,C]))
    return np.dot(np.absolute(p0 - p1),n)

def get_good_indices(thresh=0.022):
    camera_points = load_camera_points()
    plane = least_squares_plane_normal(camera_points)
    good_pts = []
    for i in range(camera_points.shape[0]):
        p = camera_points[i,:]
        dist = distance_to_plane(plane, p)
        if abs(dist) > thresh:
            continue
        else:
            good_pts.append(i)
    return good_pts

def plot_camera_points(camera_points):
    """
    Plots points in camera_frame. Axes may need to be edited.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(np.array(camera_points[:,0]), np.array(camera_points[:,1]), np.array(camera_points[:,2]),c='r')

    ax.set_xlim3d(-0.1, 0)
    ax.set_ylim3d(-0.05, 0.05)
    ax.set_zlim3d(0.1,0.2)
    plt.show()

def plot_training_error():
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    cmat = solve_for_camera_matrix()
    camera_points = load_camera_points()
    robot_points = load_robot_points()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(np.array(robot_points[:,0]), np.array(robot_points[:,1]), np.array(robot_points[:,2]),c='r')
    ax.set_xlim3d(0, 0.1)
    ax.set_ylim3d(-0.05,0.05)
    ax.set_zlim3d(-0.192,-0.092)

    t = np.ones((4, 30))
    t[:3,:] = camera_points.T
    rnew = (cmat * t).T

    ax.scatter(np.array(rnew[:,0]), np.array(rnew[:,1]), np.array(rnew[:,2]))
    plt.show()

if __name__ == '__main__':

    camera_points = load_camera_points()
    robot_points = load_robot_points()

    cmat = solve_for_camera_matrix()
    print cmat

    # write_mat_to_file("camera_matrix.p", cmat)

    plot_camera_points(camera_points)
    plot_training_error()
