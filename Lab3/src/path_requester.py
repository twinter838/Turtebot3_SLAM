#!/usr/bin/env python3

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
class PathRequester:




    def __init__(self):
        requestPath = rospy.ServiceProxy('plan_path', GetPlan)
        rospy.Subscriber('/odom', Odometry , self.update_odometry)
    


    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        # TODO
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll , pitch , yaw) = math.euler_from_quaternion(quat_list)
        self.pth = yaw


    def run(self):
        rospy.sleep(0.5)
        rospy.spin()