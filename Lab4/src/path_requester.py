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
        rospy.loginfo("Path Planning Service Recieved")
        self.requestPath = rospy.ServiceProxy('plan_path', GetPlan)

        rospy.Subscriber('/move_base_simple/goal', PoseStamped,self.update_path_goal)
        rospy.Subscriber('/frontier_detector/frontier_waypoint', PoseStamped,self.update_path_goal)
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
        curr_pose = rospy.wait_for_message('/drive/currpose',PoseStamped)
        plan=GetPlan()
        path=Path()
        plan.goal = self.goal
        plan.start = PoseStamped()
        plan.start.pose.position.x = curr_pose.pose.position.x
        plan.start.pose.position.y = curr_pose.pose.position.y
        plan.start.pose.position.z=0
        plan.start.header.frame_id=('map')
        plan.start.pose.orientation=curr_pose.pose.orientation
        plan.tolerance = 0.05
        path=self.requestPath(plan.start,plan.goal,plan.tolerance)
        return True,"Path Planned"





    def run(self):
        rospy.sleep(0.5)
        rospy.spin()

if __name__ == '__main__':
    PathRequester().run()
