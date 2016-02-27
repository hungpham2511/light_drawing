#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import yaml

from denso_openrave.utils import TextColors
from numpy.linalg import norm

import rospkg
import rospy
import time
import argparse

# svg.path
from svg.path import Path, Line, Arc, CubicBezier, QuadraticBezier
from svg.path import parse_path

# xml
from xml.dom import minidom

def generate_path_from_svg(svg_file):
    """ Generate npy array from svg file

    Args:
        svg_file:   path to .svg file
    """
    # File IO
    rospy.loginfo("Getting svg from %s" % svg_file)

    doc = minidom.parse(svg_file)  # Load svg file from path
    path_strings = [path.getAttribute('d') for path
                    in doc.getElementsByTagName('path')]
    doc.unlink()

    rospy.loginfo("Exist %d path(s) in the SVG file" % len(path_strings))
    parsed_svg = parse_path(path_strings[0])  # Select the path to extract

    # Frame setting
    framedimension = np.array([[-0.2, 0.2], [0.4, 0.8]])  # y,z in meter (square region is recommended)

    # Extract data
    data = []
    numpoint = 1000  # Number of waypoints
    X = []
    Y = []
    totaltime = 19
    totaldist = 0
    scale = 1
    for i in range(numpoint):
        t = i/(numpoint+1e-10)
        X.append(parsed_svg.point(t).real)
        Y.append(parsed_svg.point(t).imag)
    maxX = np.max(X)
    minX = np.min(X)
    maxY = np.max(Y)
    minY = np.min(Y)
    scale = framedimension[0][1]-framedimension[0][0]
    if maxX -minX > maxY - minY:
        max = maxX-minX
    else:
        max = maxY-minY
    for i in range(numpoint):
        x = 0.5
        y = scale*((X[i]-minX)/max)+framedimension[0][0]
        z = scale*((-Y[i]+maxY)/max)+framedimension[1][0]
        data.append(np.array([x,y,z]))
        if i != 0:
            totaldist += np.linalg.norm(data[i] - data[i-1])

    # Retime trajectory to uniform velocity
    vel = totaldist / totaltime
    data_with_time = []
    for i, d in enumerate(data):
        if i == 0:
            data_with_time.append([0, d[0], d[1], d[2]])
        delta_t = np.linalg.norm(data[i] - data[i - 1]) / vel
        data_with_time.append([delta_t, d[0], d[1], d[2]])

    # Save trajectory to numpy array
    data_with_time = np.array([data_with_time])
    # for d in data_with_time:
    #     print d
    path = svg_file.replace('svgs', 'drawings').replace('svg', 'npy')
    np.save(path, data_with_time)

    rospy.loginfo("Generated data at: %s" % path) 


if __name__ == "__main__":
    # Create the parse
    parser = argparse.ArgumentParser(description="Input SVG file for robotic light drawing")
    parser.add_argument('--svg', help='Svg file name. File should be stored in /data/svgs/')
    args = parser.parse_args(rospy.myargv()[1:])

    # Init the node
    n = rospy.init_node('generate_drawing', log_level=rospy.DEBUG)
    rospack = rospkg.RosPack()
    svg = rospack.get_path('light_drawing') + "/data/svgs/%s.svg" % args.svg
    generate_path_from_svg(svg)
