#!/usr/bin/env python3

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Empty
from path_requester import PathRequester
class exploration_controller:

    def __init__(self):
        rospy.init_node("exploration_controller")
        rospy.Subscriber('robot_state',String, self.update_robot_state())
        rospy.Subscriber('get_frontiers',String, self.update_robot_state())
        self.pub_robot_state=rospy.Publisher('robot_state',String, queue_size=10)
        
    def explore(self):
        self.pub_robot_state.publish("Exploration")
        while(self.robot_state=="Exploration"):
            frontierCentroids=self.get_frontiers()
            path=PathRequester.request_path(frontierCentroids)


    def update_robot_state(self,msg):
        self.robot_state=msg
        if(self.robot_state=="Exploration"):
            self.explore()

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.sleep(0.5)
        self.pub_robot_state.publish("Exploration")
        rospy.spin()


        
if __name__ == '__main__':
    exploration_controller().run()
