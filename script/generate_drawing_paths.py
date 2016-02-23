#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import yaml
import rospkg
import rospy
from denso_openrave.utils import TextColors
from numpy.linalg import norm
import time
import argparse

def generate_circles_path():
    ''' Generate a numpy file containing the coordiantes for
    printing.

    Dir: (($find_pkg light_drawing)/data/drawings/circle_demo.npy)
    '''

    # Generate the path
    x = 0.7
    y = 0
    z = 1.4
    r = 0.2
    delta_t = 1e-1
    omega = 0.314
    total_time = 30
    delta_t_array = []
    path_array = []
    cur_time = 0
    for i in range(int(total_time / delta_t)):
        if i == 0:
            delta_t_array.append(0)
        else:
            delta_t_array.append(delta_t)

        path_array.append([x + r * np.cos(omega * cur_time),
                           y + r * np.sin(omega * cur_time),
                           z])

        cur_time += delta_t

    # Write it down
    rospack = rospkg.RosPack()
    path = rospack.get_path('light_drawing')+'/data/drawings/circle_demo.npy'
    data = zip(delta_t_array, path_array)
    data = np.array([])
    np.save(path, data)

    # Test
    test_arr = np.load(path)

    np.testing.assert_allclose(test_arr, data)


def generate_straight_path():
    ''' Generate a numpy file containing the coordinates for printing.

    Dir: (($find_pkg light_drawing)/data/drawings/straight_line.npy) 
    '''

    # data = [[0, 0.5, 0, 0.115], [5, 0.5, 0.0, 0.1135]]
    # 2D data

    data = [[0, 0.6, -0.2, 0.4], [1, 0.6, 0.2, 0.4]]
    msg = TextColors()
    msg.blue(str(data))

    # Write it down
    rospack = rospkg.RosPack()
    path = rospack.get_path('light_drawing')+'/data/drawings/straight_line.npy'
    data = np.array([data])
    np.save(path, data)

    rospy.loginfo("Generated a straight line at: %s" % path)

def generate_camel():
    import numpy as np
    import matplotlib.pyplot as plt
    camel = np.array([131.61278,133.2582, -9.59722,19.19444, -34.898987,3.4899, -43.62374,21.81187, -13.08712,21.81187, 5.23485,9.59722 ,12.21464,0.87247, 41.878786,-7.85227, 17.449501,3.4899, 8.72474,21.81187 ,-22.684341,63.69066 ,-5.23485,41.00631, 14.832071,41.00631, 18.32197,13.08712, 24.4293,4.36238 ,40.13383,-22.68435, 39.26137,-75.9053, 21.81187,-38.38889 ,28.79166,-22.68434, 30.53662,-5.23485, 19.19444,23.55682, 11.34218,3.4899, 29.66414,-13.9596, 47.11363,9.59722, 30.53662,40.13384, 4.36237,28.79167 ,28.79167,53.22096 ,11.34217,38.38889, 0.0,48.85858, 0.0,38.38889, 12.21465,25.30177, 15.70455,13.9596, -6.9798,46.24116, -18.32197,65.43561, -5.23485,6.97979, -40.13384,1.74495 ,-5.23485,-6.10732, 7.85227,-12.21465, 34.89899,-21.81187, 5.23485,-43.62373, -8.72474,-29.66414, -41.00632,-69.79798, -27.91919,-63.69066, -27.04672,-36.64394, -21.81187,-10.4697, -79.3952,8.72475, -68.9255,32.28157, -6.9798,31.40909, 17.44949,22.68434, 21.81187,0.87247, 21.81187,-14.83207, 8.72475,-47.11363, -16.57702,-31.40909, -21.81187,-4.36238, -19.19445,11.34217, -24.42929,34.89899, -3.4899,46.24117, 17.4495,126.50883, 2.61742,33.15405])

    l = len(camel)
    points = []
    x = []
    y = []
    z = []
    points.append(np.array([camel[0], camel[1]]))
    for i in range(l/2-1):
        p = np.array([camel[2*(i+1)], camel[2*(i+1)+1]])
        p = points[-1] + p
        points.append(p)

    for point in points:
        y.append((point[0]-300)/1200.)
        z.append((1000-point[1])/1200.)
        x.append(0.5)
    totaldist = 0
    for i in range(len(y)-1):
        totaldist += np.sqrt((y[i+1]-y[i])**2 + (z[i+1] - z[i])**2)
    totaltime = 5
    vel = totaldist/totaltime
    data = []
    data.append([0., x[0], y[0], z[0]])
    for i in range(len(y)-1):
        sumdist = np.sqrt((y[i+1]-y[i])**2 + (z[i+1] - z[i])**2)
        data.append([sumdist/vel, x[i+1], y[i+1], z[i+1]])
    # plt.scatter(y,z)
    # plt.show()    
    # Write it down
    rospack = rospkg.RosPack()
    path = rospack.get_path('light_drawing')+'/data/drawings/camel.npy'
    data = np.array([data])
    np.save(path, data)

    rospy.loginfo("Generated a camel at: %s" % path)


def generate_dancer():

    dancer = np.array([204.69525,767.47706, 7.38591,-31.65391, 21.10261,-51.70137, 18.99234,-40.09495, 3.16539,-27.43338, -5.27565,-37.98469, 2.11026,-16.88208, 2.11026,-6.33078, -4.22052,-39.03982, 0,-30.59878, 7.38591,-35.87442, 13.71669,-31.65391, 21.10261,-28.48851, 26.37825,-23.21286, 9.49617,-6.33079, -5.27565,-7.38591, -27.43338,-16.88208, -16.88209,-11.60643, -10.5513,-8.44104,-6.33078,-7.38591, -12.66156,-1.05513, -16.88208,8.44104, -7.38591,4.22052, -3.16539,-2.11026, -6.33079,2.11026, -5.27565,5.27565, -3.16539,1.05513, -3.16539,-3.16539, 14.77182,-18.99234, -18.99234,6.33078, -2.11026,-3.16539, 29.54365,-14.77183 ,10.5513,-1.05513 ,17.93721,0, 10.5513,2.11026, 15.82695,9.49618, 37.98469,21.1026, 9.49617,7.38591, 23.21287,18.99234, 20.04747,-5.27565, 17.93721,-1.05513, 14.77182,5.27565, 18.99235,13.71669, 13.71669,27.43339, 8.44104,11.60643, 16.88208,15.82695, 5.27565,8.44104, 5.27566,5.27566, 10.5513,4.22052, 2.11026,3.16539, 0,10.5513, -3.16539,6.33078, 4.22052,1.05513, -4.22052,9.49617, 4.22052,5.27565, -8.44104,5.27565, -4.22052,2.11026, -8.44105,14.77183 ,-9.49617,7.38591, -13.71669,4.22052, -20.04747,-3.16539, -22.15774,-12.66157, -15.82695,-13.71669, 5.27565,-12.66156 ,9.49617,-7.38591, 2.11026,-12.66156, 24.268,-13.71669, -1.05513,-2.11026, -18.99235,0, -12.66156,2.11026, -13.71669,-2.11026, -6.33078,-8.44105, -3.16539,-12.66156, 6.33078,-23.21286, 17.93721,-11.60643, 20.04748,-8.44105, 29.54364,-29.54364, 28.48852,-36.92956, 28.48851,-37.98468 ,13.71669,-12.66156, 5.27565,-13.7167, -6.33078,-15.82695, -4.22052,-10.5513, 0,-9.49617, 7.38591,4.22052, -1.05513,-8.44104, -8.44104,-9.49617, 1.05513,-4.22052, 5.27565,4.22052, 4.22052,-6.33079, 7.38591,8.44105, 3.16539,13.71669, 7.38592,42.20521, -15.82696,23.21286, -14.77182,28.48851, -12.66156,16.88209, -18.99234,21.1026, -17.93722,23.21286, -4.22052,18.99235, -7.38591,11.60643 ,-4.22052,9.49617, -11.60643,21.1026 ,-15.82695,9.49617, -21.10261,4.22053, -26.37825,-8.44105 ,-8.44104,-4.22052, -15.82696,1.05513, -10.5513,6.33078, 9.49617,13.7167, 6.33079,18.99234, -4.22053,16.88208, -5.27565,7.38591, -16.88208,9.49617, -8.44104,12.66157, -4.22052,31.6539, 4.22052,17.93721, 14.77182,16.88209, 6.33078,15.82695, 10.5513,15.82695, 8.44105,7.38591, 9.49617,30.59878, 13.71669,62.25268, 10.5513,78.07963, 8.44104,11.60643])

    # l = len(camel)
    # points = []
    # x =[]
    # y = []
    # z = []
    # points.append(np.array([camel[0],camel[1]]))
    # for i in range(l/2-1):
    #     p = np.array([camel[2*(i+1)],camel[2*(i+1)+1]])
    #     p = points[-1] + p
    #     points.append(p)
    l = len(dancer)
    points = []
    x =[]
    y = []
    z = []
    points.append(np.array([dancer[0],dancer[1]]))
    for i in range(l/2-1):
        p = np.array([dancer[2*(i+1)],dancer[2*(i+1)+1]])
        p = points[-1] + p
        points.append(p)
        
    for point in points:
        y.append((point[0]-300)/1200.)
        z.append((1100-point[1])/1200.)
        x.append(0.5)
    totaldist = 0
    for i in range(len(y)-1):
        totaldist += np.sqrt((y[i+1]-y[i])**2 + (z[i+1] - z[i])**2)
    totaltime = 10
    data = []
    sumdist = 0
    vel = totaldist/totaltime
    data.append([0.,x[0],y[0],z[0]])
    for i in range(len(y)-1):
        sumdist = np.sqrt((y[i+1]-y[i])**2 + (z[i+1] - z[i])**2)
        data.append([sumdist/vel,x[i+1],y[i+1],z[i+1]])        
    # plt.scatter(y,z)
    # plt.show()
    # Write it down
    rospack = rospkg.RosPack()
    path = rospack.get_path('light_drawing')+'/data/drawings/dancer.npy'
    data = np.array([data])
    np.save(path, data)

    rospy.loginfo("Generated a dancer at: %s" % path)




if __name__ == "__main__":
    # Init the node
    n = rospy.init_node('generate_drawing', log_level=rospy.DEBUG)
    generate_straight_path()
    generate_camel()
    generate_dancer()
    