#!/usr/bin/env python3

import rospy
import math
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from path_planner import PathPlanner
from std_msgs.msg import String,Empty
from priority_queue import PriorityQueue
from std_srvs.srv import Empty,SetBool
from std_msgs.msg import Bool,Empty

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

         #holds used centroid indices, gets reset during calculation
        
        self.pub_Frontier = rospy.Publisher('/frontier_detector/frontier',OccupancyGrid, queue_size=10)
        self.pub_frontier_waypoint = rospy.Publisher('/frontier_detector/frontier_waypoint', PoseStamped, queue_size= 10)
        self.pub_robot_state=rospy.Publisher('robot_state',String, queue_size=10)

        rospy.Subscriber('/path_planner/cspace_occgrid',OccupancyGrid,self.find_frontiers)
        rospy.Subscriber('/drive/awaitingPath',Empty,self.detect_closest_centroid)
        
        # request_frontier_centroid = rospy.service('frontier_centroid_service')
        self.centroidOld=(0,0)
        self.initialPosition=rospy.wait_for_message('/drive/currpose',PoseStamped)
        rospy.loginfo("Frontier detector node ready")      
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
        self.map=mapdata
        self.frontierData=self.find_boundary(mapdata)
        self.pub_Frontier.publish(self.frontierData)
        rospy.loginfo("Publishing Frontier Map")

        # rospy.loginfo(self.frontierList)
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
    def find_centroid(self,frontier_data):
        
        ## go through each of the points addding them together and then take the average
        c_x = 0
        c_y = 0
        n = len(frontier_data)
        avgX=0
        avgY=0
        distOld=100000
        for i in frontier_data:
            avgX+=i[0]
            avgY+=i[1]
        avgX=avgX//n
        avgY=avgY//n
        ##just finds roughly the centroid based off of the given points found
        for i in frontier_data:
            dist=PathPlanner.euclidean_distance(i[0],i[1],avgX,avgY)
            if(distOld>dist):
                distOld=dist
                point=i
        # # rospy.loginfo(frontier_data)
        # point = frontier_data[n//2]
        rospy.loginfo("Centroid Calc")
        rospy.loginfo(frontier_data)
        rospy.loginfo(avgX)
        rospy.loginfo(avgY)
        rospy.loginfo(point)
        c_x = point[0]
        c_y = point[1]

        return (c_x,c_y)
        
    

    def find_centroid_and_members(self, frontierPoints, mapdata):

        usedcentroids = []

        """
        For each point that is a frontier and had not been used, 
            use the wave front algorithm to find all the points that are in a block of frontier,
            add these points to a temp list starting at the 1st index and append this list to the list of lists later on

            Calculate the centroid and put it as the 0th index

        return the list of lists 
        
        """

        width = mapdata.info.width
        height = mapdata.info.height

        usedPoints = [0] * len(frontierPoints.data)
        pointsToVisit = [] ## queue
        currentFrontierIndex = 0
        centroids=[]

        for i, pointVal in enumerate(frontierPoints.data):
            
            
            ## If it's a frontier point and unused
            if (pointVal == 100) and usedPoints[i] == 0:

                pointsToVisit.append(i)

                ## For each point surrounding i, that is in range of the grid
                    ## if it is also a frontier point, add it to the priority queue
                frontierTemp=[]
                while(len(pointsToVisit)>0):
                    # rospy.loginfo(pointsToVisit)
                    
                    index = pointsToVisit.pop()
                    usedPoints[index] = 1

                    workingPoint = PathPlanner.index_to_grid(mapdata, index)

                    frontierTemp.append(workingPoint)
                    pointX = workingPoint[0]
                    pointY = workingPoint[1]

                    ##Iterates through Y
                    for k in range(-1, 2, 1): ## If k == padding, it would end up being false in k < padding

                        workingY = pointY + k

                        ##Iterates through X
                        for l in range(-1, 2, 1):
                            
                            workingX = pointX + l
                            
                            ## If within the grid
                            if (workingY in range(0, height - 1 )) and (workingX in range(0, width - 1)):

                                workingIndex = PathPlanner.grid_to_index(mapdata, workingX, workingY)

                            # if(index == workingIndex):
                            #     continue ##don't consider the cell itself or you will end up in a loop
                                if(frontierPoints.data[workingIndex] == 100 and usedPoints[workingIndex]== 0):
                                    pointsToVisit.append(workingIndex)
                                    # rospy.loginfo("Point to visit")
                                    # rospy.loginfo(pointsToVisit)
                                
                        #end of l loop        
                    #end of k loop
                ##end of While loop
                

                # rospy.loginfo('frontiers')
                # rospy.loginfo(frontierTemp)
                ##done looping through a group of frontier points
                centroids.append(self.find_centroid(frontierTemp))

                # frontierFinal=[]
                # frontierFinal.append(frontierCentroid)
                # frontierFinal.append(frontierTemp)
                # frontiers.append(frontierFinal)

                ## Iterates through frontier numbers
                
            currentFrontierIndex += 1


            ##end of if frontier cell

        ##end of i loop

        return centroids

    def detect_closest_centroid(self, msg):
        curr_pose = rospy.wait_for_message('/drive/currpose',PoseStamped)
        rospy.loginfo("Goal Request Recieved")
        current = PathPlanner.world_to_grid(self.map, curr_pose.pose.position)
        centroids=[]
        centroids=self.find_centroid_and_members(self.frontierData,self.map)


        centroids_and_distances = {}
        #making a list of all the centroids

        # for i in frontierList:
        #         firstFrontier = i
        #         centroid=firstFrontier[0]    
        #         centroids.append(centroid)
 

        indexOfClosestCentroid = 0 
        firstCentroid = centroids[0] 
        min = PathPlanner.euclidean_distance(current[0], current[1], firstCentroid[0], firstCentroid[1])
    
        # Robot perfers to go to a frontier near the one it last visited
        for centroid in centroids:
            dist = PathPlanner.euclidean_distance(self.centroidOld[0],self.centroidOld[1],centroid[0],centroid[1])
            centroids_and_distances[centroid]=dist
            rospy.loginfo(dist)
            
            if(dist<10):
                closestCentroidPoint = centroid
                closestCentroid = PoseStamped()
                centroidPoint=PathPlanner.grid_to_world(self.map,closestCentroidPoint[0],closestCentroidPoint[1])
                closestCentroid.header=curr_pose.header
                closestCentroid.pose.position=centroidPoint
                closestCentroid.pose.orientation=curr_pose.pose.orientation
                self.pub_frontier_waypoint.publish(closestCentroid)
                success = rospy.wait_for_message('/path_planner/success', Bool)
                rospy.loginfo(success)
                if success.data==True:
                    rospy.loginfo("Perfering to visit most reently visited frontier")
                    self.centroidOld=centroid
                    return True
                        
        
                 


        # distances=centroids_and_distances.get()
        # distances.sort

        # sorted_centroids = dict(sorted(centroids_and_distances.items(), key=lambda item: item[1]))
        for centroid in centroids:
            closestCentroidPoint = centroid
            closestCentroid = PoseStamped()
            centroidPoint=PathPlanner.grid_to_world(self.map,closestCentroidPoint[0],closestCentroidPoint[1])
            closestCentroid.header=curr_pose.header
            closestCentroid.pose.position=centroidPoint
            closestCentroid.pose.orientation=curr_pose.pose.orientation
            self.pub_frontier_waypoint.publish(closestCentroid)
            success = rospy.wait_for_message('/path_planner/success', Bool)
            rospy.loginfo(success)
            if success.data==True:
                rospy.loginfo("Valid path to a frontier found")
                self.centroidOld=centroid
                return True
        
        # if(not success):
        self.pub_robot_state.publish("Return")
        self.pub_frontier_waypoint.publish(self.initialPosition)

               
 
 
    # def isNotUsed(self, index):

    #     for item in self.usedcentroids:
    #          if(index == item):
    #             return False

    #     return True

    

    def find_boundary(self, mapdata):    

        """
        for each of the known cells that are unvisited, 
        check if it fits the boundary condition and is unoccupied
        If so, add it to a boundary and denote the cell as visited
        
        """
        
        frontierData = [0] * len(mapdata.data)
        for i, j in enumerate(mapdata.data):
           
            if(j >= 0 and j < 50):
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
