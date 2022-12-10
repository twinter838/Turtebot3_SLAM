#!/usr/bin/env python3

import math
import rospy
import numpy
import string
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from path_planner import PathPlanner
from std_msgs.msg import String,Empty
from priority_queue import PriorityQueue

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

        rospy.loginfo("Frontier detector node ready")
        self.pub_Frontier=rospy.Publisher('/frontier_detector/frontier',OccupancyGrid, queue_size=10)
        self.pub_frontier_centroid = rospy.Publisher('/frontier_detector/frontier_centroid', GridCells, queue_size= 10)
        rospy.Subscriber('/path_planner/cspace_occgrid',OccupancyGrid,self.find_frontiers)
        # request_frontier_centroid = rospy.service('frontier_centroid_service')
        rospy.sleep(1.0)
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

    def find_frontiers(self,mapdata):
        """
        Finds and returns frontiers
        This method is a callback bound to a service
        :return 
        """
        rospy.loginfo("Finding Frontiers")

        frontierData=self.find_boundary(mapdata)
        self.pub_Frontier.publish(frontierData)
        rospy.loginfo("Publishing Frontier Map")

        # frontierCentroid = self.find_centroid(mapdata,frontierData)
        # self.pub_frontier_centroid.publish(frontierCentroid)
        rospy.loginfo("Publishing Frontier Centroid")
        

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
        n = len(frontier_data.data)

        for i in frontier_data.data:
            point = PathPlanner.index_to_grid(mapdata,i)
            c_x = c_x + point[0]
            c_y = c_y + point[1]


        c_x = c_x / n
        c_y = c_y / n

        return (c_x,c_y)
        
        
    def find_centroid_and_members(self, frontierPoints, mapdata):

        """
        For each point that is a frontier and had not been used, 
            use the wave front algorithm to find all the points that are in a block of frontier,
            add these points to a temp list starting at the 1st index and append this list to the list of lists later on

            Calculate the centroid and put it as the 0th index

        return the list of lists
        
        """

        width = mapdata.info.width
        height = mapdata.info.height

        usedPoints = [0] * len(frontierPoints)
        pointsToVisit = [] ## queue
        currentFrontierIndex = 0

        frontiers = [] ## List of lists

        for i in frontierPoints:

            ## If it's a frontier point and unused
            if (i == 100) and usedPoints[i] == 0:

                pointsToVisit.append[i]

                ## For each point surrounding i, that is in range of the grid
                    ## if it is also a frontier point, add it to the priority queue

                index
                pointNumber = 1
                while not (pointsToVisit == []):
                    index = pointsToVisit.pop
                    usedPoints[index] = 1
                    # pointConversion = 
                    frontiers[currentFrontierIndex][pointNumber] = index

                    workingPoint = PathPlanner.index_to_grid(i)
                    pointX = workingPoint(0)
                    pointY = workingPoint(1)

                    ##Iterates through Y
                    for k in range(-1, 2, 1): ## If k == padding, it would end up being false in k < padding

                        workingY = pointY + k

                        ##Iterates through X
                        for l in range(-1, 2, 1):
                            
                            workingX = pointX + l
                            
                            ## If within the grid
                            if (workingY in range(0, height - 1 )) and (workingX in range(0, width - 1)):

                                    workingIndex = PathPlanner.grid_to_index(mapdata, workingX, workingY)

                                    if(index == i):
                                        continue ##don't consider the cell itself or you will end up in a loop

                                    if(frontierPoints[workingIndex] == 100):
                                        pointsToVisit.append[workingIndex]
                                        


                    
                    pass

                
                ## Iterates through frontier numbers
                if (pointsToVisit == []):
                    currentFrontierIndex += 1

    def find_boundary(self, mapdata):    

        """
        for each of the known cells that are unvisited, 
        check if it fits the boundary condition and is unoccupied
        If so, add it to a boundary and denote the cell as visited
        
        """
        
        frontierData = [0] * len(mapdata.data)
        for i, j in enumerate(mapdata.data):
           
            if(j>=0 and j<50):
                point = PathPlanner.index_to_grid(mapdata, i)
                
                for k in range(-1, 2, 1): ## If k == padding, it would end up being false in k < padding
                        
                        workingY = point[1] + k
                        for l in range(-1, 2, 1):
                            
                            workingX = point[0] + l
                            ##Stil working here, building a helper, give me a bit
                            
                            if workingY in range(0, mapdata.info.height - 1 ):

                                if workingX in range(0, mapdata.info.width - 1 ):
                                    index = PathPlanner.grid_to_index(mapdata, workingX, workingY) 
                                    if(mapdata.data[index] == -1): 
                                        frontierData[i] = 100
                                    
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
        isLonely = True
        """
        
        for i in frontierData:
            
            #if the point is considered a frontier do the lonely check
            if(frontierData[i] == 100):

                for k in range(-1, 2, 1): ## If k == padding, it would end up being false in k < padding
                        
                        workingY = point[1] + k
                        for l in range(-1, 2, 1):
                            
                            workingX = point[0] + l
                            ##Stil working here, building a helper, give me a bit
                            
                            if workingY in range(0, mapdata.info.height - 1 ):

                                if workingX in range(0, mapdata.info.width - 1 ):
                                    index = PathPlanner.grid_to_index(mapdata, workingX, workingY) 
                                    if(frontierData[index] == -1 and not(index == i)): 
                                        isLonely = False
                                    
                                    # printThis = str(workingX) + ' ' + str(working)
                                    # rospy.loginfo(printThis)
                        ## iterate
                    ## iterate 
                ## has gone theough Lonely check
                
                if(isLonely):
                    frontierData[i] = 0

        """      
        For each occupied cell, dialate it

        Use wavefront to find frontiers and their centroids
        
        Check if it's reachable?
        """
        frontierMap = OccupancyGrid()
        frontierMap.header=mapdata.header
        frontierMap.info=mapdata.info
        frontierMap.data=list(frontierData)
        return frontierMap


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    frontier_detector().run()
