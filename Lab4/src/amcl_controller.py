#!/usr/bin/env python3

import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty,String
from std_srvs.srv import Empty,SetBool

class amcl_controller:


    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        rospy.init_node("amcl_controller")
        rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, self.update_AMCL_odom)
        self.runLocalizaiton = rospy.ServiceProxy('/global_localization', Empty)
        self.pub_robot_state=rospy.Publisher('/robot_state',String, queue_size=10)

        rospy.loginfo("AMCL controller node ready")      
        rospy.sleep(1.0)

    def update_AMCL_odom(self,msg):
        """
        Saves the current AMCL odom as a variable for easy access
        """
        self.AMCL_odom=msg
    
    def calculate_covariance(self):
        """
        Calculates the average covariance from a PoseWithCovrienceStamped message
        """
        rospy.loginfo("######## Calculating Covariance...")
        cov_x = self.AMCL_odom.pose.covariance[0]
        cov_y = self.AMCL_odom.pose.covariance[7]
        cov_z = self.AMCL_odom.pose.covariance[35]
        rospy.loginfo("## Cov X: " + str(cov_x) + " ## Cov Y: " + str(cov_y) + " ## Cov Z: " + str(cov_z))
        cov = (cov_x+cov_y)/2

        return cov
    def run(self):
        # Block until 
        rospy.wait_for_message('/amcl_pose',PoseWithCovarianceStamped)   
        self.pub_robot_state.publish("Localization")
        
        rospy.loginfo("Initalizing Localization")
        self.pub_robot_state.publish("Localization")
        # AMCL service call
        self.runLocalizaiton()
        rospy.sleep(0.5)
        # Spin the robot until covariance drops below a specific value
        while(self.calculate_covariance()>0.01):  
            rospy.sleep(0.1)
        self.pub_robot_state.publish("Localized")
        rospy.loginfo("Robot Localized")
        rospy.spin()

if __name__ == '__main__':
    amcl_controller().run()