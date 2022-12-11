#!/usr/bin/env python3

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty,SetBool

class PathRequester:

    def __init__(self):
        rospy.init_node("path_requester")
        rospy.loginfo("Waiting For Path Planning Service")
        request_new_path=rospy.Service('request_new_path',SetBool,self.request_path)
        rospy.wait_for_service('plan_path')
        

        frontier_detector = rospy.Service('frontier_detector',PoseStamped,self.update_path_goal)
        
        rospy.loginfo("Path Planning Service Recieved")
        self.requestPath = rospy.ServiceProxy('plan_path', GetPlan)
        rospy.Subscriber('/odom', Odometry , self.update_odometry)

        rospy.Subscriber('/move_base_simple/goal', PoseStamped,self.update_path_goal)
        rospy.sleep(0.5)


    def update_path_goal(self,msg):
        self.goal=msg
        self.request_path(msg)

    def request_path(self,msg):
        """
        Requests a Path From the Path Planner
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The goal of the path
        """
 
        rospy.loginfo("Generating path request")
        plan=GetPlan()
        path=Path()
        plan.goal=self.goal
        plan.start=PoseStamped()
        plan.start.pose.position.x=self.px
        plan.start.pose.position.y=self.py
        plan.start.pose.position.z=0
        plan.start.header.frame_id=('map')
        plan.start.pose.orientation=self.quat_orig
        plan.tolerance=0.05
        path=self.requestPath(plan.start,plan.goal,plan.tolerance)
        return True,"Path Planned"
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
