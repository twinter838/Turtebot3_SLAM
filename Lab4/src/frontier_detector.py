#!/usr/bin/env python3

import math
import rospy
import numpy
import string
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from path_planner import PathPlanner

class frontier_detector:



    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("frontier_detector")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        # TODO
        ## Initialize the request counter
        # TODO
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Frontier detector node ready")
        self.pub_Frontier=rospy.Publisher('/frontier_detector/frontier',OccupancyGrid, queue_size=10)

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        mapdata=rospy.wait_for_message('/map',OccupancyGrid)
        return mapdata

    def find_frontiers(self):
        mapdata = frontier_detector.request_map()
        rospy.loginfo("Finding Frontiers")

        frontierData=self.find_boundary(mapdata)
        self.pub_Frontier.publish(frontierData)
        rospy.loginfo("Pubhishing Frontier Map")

        """"
        for each defined boundary cell
            Check if there is another boundary cell in it's vicinity
                

        """

    #threshold map consider only walkable regions between known and unknown 

    #edge detection


    #frontier centroid calculation
    def find_centroid(self,mapdata,frontier_data):
        
        ## go through each of the points addding them together and then take the average
        c_x = 0
        c_y = 0
        n = frontier_data.data.length

        for i in frontier_data.data:
            point = PathPlanner.index_to_grid(mapdata,i)
            c_x = c_x + point[0]
            c_y = c_y + point[1]


        c_x = c_x / n
        c_y = c_y / n

        return (c_x,c_y)
        
        
    # for 


    def find_boundary(self,mapdata):    

        """
        for each of the known cells that are unvisited, 
        check if it fits the boundary condition and is unoccupied
        If so, add it to a boundary and denote the cell as visited
        
        """
        
        frontierData = [0] * len(mapdata.data)

        padding = 1
        for i, j in enumerate(mapdata.data):
           
            if(j>=0 and j<50):
                point = PathPlanner.index_to_grid(mapdata, i)
                
                for k in range(-padding, padding + 1, 1): ## If k == padding, it would end up being false in k < padding
                        
                        workingY = point[1] + k
                        for l in range(-padding, padding + 1, 1):
                            
                            workingX = point[0] + l
                            ##Stil working here, building a helper, give me a bit
                            
                            if workingY in range(0, mapdata.info.height - 1 ):

                                if workingX in range(0, mapdata.info.width - 1 ):
                                    index = PathPlanner.grid_to_index(mapdata, workingX, workingY) 
                                    if(mapdata.data[index] == -1): 
                                        frontierData[index] = 100
                                    
                                    # printThis = str(workingX) + ' ' + str(working)
                                    # rospy.loginfo(printThis)
                        ## iterate
                    ## iterate 

        """
        Frontier lonely check

        isLonely = true
        for each occupied cell,
            Go through each of its surrounding cells,
            if a second cell is there, isLonely = false            

        Apply C space

        For each occupied cell, dialate it

        Use wavefront to find frontiers and their centroids
        
        Check if it's reachable?
        """
        frontierMap=OccupancyGrid()
        frontierMap.header=mapdata.header
        frontierMap.info=mapdata.info
        frontierMap.data=list(frontierData)
        return frontierMap


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        while True:
            self.find_frontiers()
            rospy.sleep(1)
        rospy.spin()


        
if __name__ == '__main__':
    frontier_detector().run()
