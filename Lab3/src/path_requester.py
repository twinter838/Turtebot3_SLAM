#!/usr/bin/env python3

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
class PathRequester:

    def __init__(self):
        rospy.init_node("path_requester")
        rospy.loginfo("Waiting For Path Planning Service")
        rospy.wait_for_service('plan_path')
        rospy.loginfo("Path Planning Service Recieved")
        self.requestPath = rospy.ServiceProxy('plan_path', GetPlan)
        rospy.Subscriber('/odom', Odometry , self.update_odometry)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped,self.request_path)

    def request_path(self,msg):
        """
        Requests a Path From the Path Planner
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The goal of the path
        """
        rospy.loginfo("Generating path request")
        plan=GetPlan()
        path=Path()
        plan.goal=msg
        plan.start=PoseStamped()
        plan.start.pose.position.x=self.px
        plan.start.pose.position.y=self.py
        plan.start.pose.position.z=0
        plan.start.header.frame_id=('map')
        plan.start.pose.orientation=self.quat_orig
        plan.tolerance=0.05
        path=self.requestPath(plan.start,plan.goal,plan.tolerance)
        
    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.

        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.quat_orig = msg.pose.pose.orientation



    def run(self):
        rospy.sleep(0.5)
        rospy.spin()

if __name__ == '__main__':
    PathRequester().run()
