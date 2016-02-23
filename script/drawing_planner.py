#!/usr/bin/env python
import numpy as np

from printx_controller.printingLib import PrintPlanner
import rospkg
import rospy
from light_drawing.srv import DrawingRequest
from denso_openrave.interfaces import DensoRobot
from denso_openrave.utils import read_parameter
import time
import argparse

from openravepy import Environment
from openravepy import RaveSetDebugLevel, DebugLevel
def convert_to_zipped_form(paths):
    """ Converter.

    From this form
    [
        [[t1, p1, p2, p3], [t2, p4, p5, p6]]
        ]
    to this form
    [   [[t1, [p1, p2, p3]], [t2, [p4, p5, p6]]]
        ]
    """
    # TODO: Fix this, there is no need to have the second form.
    zipped_paths = []
    for path in paths:
        dt = path[:, 0]
        pts = path[:, 1:4]
        zipped_path = zip(dt, pts)
        zipped_paths.append(zipped_path)
    return zipped_paths


class Controller(object):
    """ A controller for Denso-VS060.

    Contains both i) planner class and ii) executor class.
    Planning and Execution are triggerred by a service.
    Receive a request containing the directory of the numpy array
    which specifies the drawing trajectory, plan the path and execute it.

    Attributes:
        env: OpenRAVE environment
        robot: OpenRAVE robot
        pl: Planner class
        ex: Executor class (DensoRobot), responsible for executing the actual
    Denso
    """
    def __init__(self, sim=False):
        """ Constructor

        Construct all 3D world's information. Then constructs both planner and
        executor object.
        """
        # Get parameters
        robot_name = read_parameter('~robot_name', 'left')
        table_id = int(read_parameter('~table_id', 2))
        # Init openrave env
        self.env = Environment()
        self.env.SetViewer('qtcoin')
        # Get the robot
        rospack = rospkg.RosPack()
        path = rospack.get_path('light_drawing') + '/robots/denso_light.robot.xml'
        self.robot = self.env.ReadRobotXMLFile(path)
        self.robot.SetName(robot_name)
        manip_name = 'light'
        # Get the objects
        table_path = rospack.get_path('denso_openrave') + '/objects/table.kinbody.xml'
        table = self.env.ReadKinBodyXMLFile(table_path)
        Ttable = np.eye(4)
        Ttable[:3, 3] = np.array([0.7, 0.3, 0.126])
        # Ttable[2, 3] += 9.0e-3  # Uncomment if using the acrylic printing board
        table.SetTransform(Ttable)
        # Add the objects to the environment.
        self.env.AddRobot(self.robot)
        self.env.AddKinBody(table)
        self.sim = sim

        # Init planner and executor
        self.pl = PrintPlanner(self.env, robot_name, manip_name, hardware=False)
        self.ex = DensoRobot(self.env, robot_name, manip_name)  

        # Advertise the service
        srv_name = 'draw_light'
        s = rospy.Service(srv_name, DrawingRequest, self.handle_request)
        rospy.loginfo("Service %s is ready" % srv_name)

    def handle_request(self, req):
        """ Chain of command after receiving a request

        Try to load path's coordinate data from the request.
        Plan a trajectory using Planner
        Execute it using DensoRobot class.
        Args:
            req: ROS's request. Type: PrintRequest
        """
        # Load printing path data
        rospack = rospkg.RosPack()
        repo_dir = rospack.get_path('light_drawing')
        dir = repo_dir + "/data/drawings/" + req.drawing_dir
        rospy.loginfo("Planning started! Looking for drawings at: %s" % dir)
        try:
            printing_paths = convert_to_zipped_form(np.load(dir))

            # Plan to get robot's trajectories
            
            t0 = time.time()
            # trajectories = self.pl.plan_paths(paths=printing_paths, step=0.01, sim=False)
            traj = self.pl._plan_printing_path(printing_paths[0],
                                               desired_pose=[1, 0, 0],
                                               step=0.005,
                                               sim=self.sim,
                                               interpolation='linear')
            duration = time.time() - t0
            rospy.loginfo("Planning took %0.1f seconds" % duration)

            # Execute the trajectory
            # 1 - Move to the beginning of the trajectory
            qstart = self.ex._GetJointValuesFromTraj(traj, 0)
            rospy.loginfo("Moving robot to starting point at: %s!" % qstart)
            self.ex.MoveToGoal(qstart, execute=True)
            # 2 - Execute the trajectory
            rospy.loginfo("Executing the trajectory!")
            self.ex.ExecuteRavePath(traj)
            rospy.loginfo("Done!")
        except IOError:
            rospy.logerr('Printing paths data not found!')

if __name__ == "__main__":
    # Create the parse
    parser = argparse.ArgumentParser(description='Generate OpenRAVE databases')
    parser.add_argument('--sim', action='store_true',
                        help='If flagged run simulation of planning')
    args = parser.parse_args(rospy.myargv()[1:])

    # Init node
    node_name = 'printing_controller_server_demo'
    rospy.init_node(node_name, log_level=rospy.DEBUG)
    rospy.loginfo('Starting [%s] node' % node_name)

    # Or debug
    RaveSetDebugLevel(DebugLevel.Info)
    controller = Controller(sim=args.sim)  # main function

    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)