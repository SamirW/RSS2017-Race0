import rospy
import numpy as np

from std_msgs.msg import String, Header, Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
import tf.transformations
import tf
import matplotlib.pyplot as plt
import range_libc
import time
import math

from threading import Lock

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from scipy.misc import imread
# particles = np.zeros((10,3))
# action = np.array([3,2,1])

# particles[:,0] += action[0]*np.cos(particles[:,2]) - action[1]*np.sin(particles[:,2])

# print particles

# n_ranges, x, y, theta = int, float, float, float

n_ranges = 2
x = 300
y = 500
theta = np.pi/2.
max_scan_angle = 90*(np.pi/180)
testMap = range_libc.PyOMap("../maps/basement_fixed_more_dilated.png",1)

bl = range_libc.PyBresenhamsLine(testMap, 500)
queries = np.zeros((n_ranges, 3), dtype=np.float32)
ranges = np.zeros(n_ranges, dtype=np.float32)
queries[:,0] = x
queries[:,1] = y
queries[:,2] = theta + np.linspace(-max_scan_angle, max_scan_angle, n_ranges)
points = bl.calc_range_many(queries,ranges)
bl.saveTrace("./test.png")
print imread("./test.png")[500][600]
map_img = imread("./test.png")

cells = set()
for i in range(len(map_img[0])):
    for j in range(len(map_img)):
        if map_img[i][j][2] == 200:
            cells.add((j,i))
print cells

