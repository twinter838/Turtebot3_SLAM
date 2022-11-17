#!/usr/bin/env python3

import math
import rospy
import string
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue


class PathPlanner:


    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        # TODO
        plan_path=rospy.Service('plan_path',GetPlan,self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        # TODO
        self.pub_CSpace = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        # TODO
        self.pub_Frontier = rospy.Publisher('/path_planner/frontier', GridCells, queue_size=10)
        self.pub_Path = rospy.Publisher('/path_planner/path', Path, queue_size=10)
        self.pub_Visited=rospy.Publisher('/path_planner/visited', GridCells, queue_size=10)

        ## Initialize the request counter
        # TODO
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

    """
    NOTES: from the lab document

    Consider that the origin (0, 0) point of the map is likely different from the robot
    Rotations will not be needed in this lab

    We need to look in the yaml files for the resolution of a cell in the real world
    (Pass the message?)
    MAPData is passed

    Message type of OccupancyMap has field info that holds the nav_msgs/MapMetaData, with the width height and position of origin 
    (how does it describe where it's origin is)
    Go through the psuedocode

    For Cell coordinate to World coordinates be aware the the translation error is liekly going to be (Cell resolution) / 2 due to... 
    ... look at the sketchbook example

    World to cell with also have the issue of translation with cellres/2 being the error 

    For Calculating the C space, the initial occupancy grid is passed by the map server and then we calculate the C space
    we may need to ceiling some of the calculations due to if we round down we may not create the proper buffer.

    Implementing A star will take in our current occupancy map after adding C space and use wave method (confirmed that's what a star is)
    Possibly implement early exit? (The code ends and returns a path when it finds the end)

    


    """

    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        ### REQUIRED CREDIT
           
        return (y * mapdata.info.width) + x
   
    @staticmethod
    def index_to_grid(mapdata, index):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param  [int] The index.
        :return x [int] The cell X coordinate.
        :return y [int] The cell Y coordinate.
        """
        ### REQUIRED CREDIT
        x = index % mapdata.info.width
        y = index // mapdata.info.width
        return x,y


    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        ### REQUIRED CREDIT
        return(math.sqrt(math.pow(x1-x2,2)+math.pow(y1-y2,2)))
        


    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        ### REQUIRED CREDIT
        wx = (x + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        wy = (y + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y

        return Point(wx,wy,0)

        
    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        ### REQUIRED CREDIT
        
        ## we do have a template, and the translation will be cellred/2 likely
        x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
        y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
        return(x,y)

        
    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        pass

    

    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        ### REQUIRED CREDIT
        
        if(mapdata.data[PathPlanner.grid_to_index(mapdata,x,y)]==100 or x > mapdata.info.width or y > mapdata.info.height):
            return False
        return True
         

               

    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT
        walkableCells=[]
        neighbors_of_4 = [ (x,y+1),(x,y-1),(x+1,y),(x-1,y)]
        
        for neighbor in neighbors_of_4 :

            if(PathPlanner.is_cell_walkable(mapdata,neighbor[0],neighbor[1])):
                walkableCells.append(neighbor)

        return walkableCells
    
    
    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        walkableCells=[]
        neighbors_of_8 = [(x,y+1),(x,y-1),(x+1,y),(x-1,y),(x+1,y+1),(x-1,y-1),(x+1,y-1),(x-1,y+1)]
        
        for neighbor in neighbors_of_8 :
            if(PathPlanner.is_cell_walkable(mapdata,neighbor[0],neighbor[1])):
                walkableCells.append(neighbor)

        return walkableCells
    
    
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


    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        ## Inflatcopy.deepcopy(e the obstac)les where necessary     
        CSpace = mapdata.data
        CSpace = list(CSpace)

        mapWidth = mapdata.info.width
        mapHeight = mapdata.info.height

        for i, j in enumerate(mapdata.data):
            if(j == 100): ## If it's occupied

                point = PathPlanner.index_to_grid(mapdata, i)
                pointX = point[0]
                pointY = point[1]
                for k in range(-padding, padding + 1, 1): ## If k == padding, it would end up being false in k < padding
                    
                    workingY = pointY + k
                    for l in range(-padding, padding + 1, 1):
                        
                        workingX = pointX + l
                        ##Stil working here, building a helper, give me a bit
                        
                        if workingY in range(0, mapHeight - 1 ):

                            if workingX in range(0, mapWidth - 1):
                                index = PathPlanner.grid_to_index(mapdata, workingX, workingY)
                                CSpace[index] = 100
                                # printThis = str(workingX) + ' ' + str(workingY)
                                # rospy.loginfo(printThis)
                    ## iterate
                ## iterate

                
        rospy.loginfo("THIS BABY IS THE ENTIRE AGENDA")
        CSpace = tuple(CSpace)

        ## Create a GridCells message and publish it
        Msg_GridCells = GridCells()
        Msg_GridCells.cell_height = mapdata.info.resolution
        Msg_GridCells.cell_width = mapdata.info.resolution
        GridCell_Coord = []
        ## For occupied grids in the Cspace array, convert them to world coordnates and then add them to the array of points to be passed to the gridcells message
        for i, j in enumerate(CSpace):
                if(j == 100):
                    gridCoord = PathPlanner.index_to_grid(mapdata, i)
                    pointWC = PathPlanner.grid_to_world(mapdata, gridCoord[0], gridCoord[1])
                    GridCell_Coord.append(pointWC)     

        Msg_GridCells.cells = GridCell_Coord
        Msg_GridCells.header.frame_id = ('map')
        self.pub_CSpace.publish(Msg_GridCells)        
        ## Return the C-space
        CspaceOG = OccupancyGrid()
        CspaceOG.data = CSpace
        CspaceOG.info = mapdata.info
        
        return CspaceOG

    
    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        msgVisted=GridCells()
        msgVisted.header.frame_id=('map')
        msgVisted.cell_height = mapdata.info.resolution
        msgVisted.cell_width = mapdata.info.resolution
        frontier = PriorityQueue()
        frontier.put(start,0)
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        # came_from.append(start)
        cost_so_far[start] = 0
        visited=[]
        # cost_so_far.append(0)

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break
            neighbors=PathPlanner.neighbors_of_8(mapdata,current[0], current[1])
            for  next in neighbors:
                ###TODO Add cost penalty for making turns
                new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(current[0], current[1], next[0], next[1])
                # rospy.loginfo(new_cost)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(goal[0],goal[1],next[0],next[1])
                    frontier.put(next, priority)
                    came_from[next] = current
                    visited.append(PathPlanner.grid_to_world(mapdata,next[0],next[1]))
                    # rospy.loginfo(visited)
                    msgVisted.cells=visited
                    self.pub_Visited.publish(msgVisted)
        path=[]
        while current != start:
            path.append(current)
            current = came_from[current]
        path.reverse()
        rospy.loginfo("Finished A*")
        rospy.loginfo(path)



        return path


    
    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        
        pathOpt=[]
        previousX = -1
        previousY = -1

        for index, point in enumerate(path):
            if(index+1<len(path)):
                future=path[index + 1]
            else:
                pathOpt.append(point)
                rospy.loginfo(pathOpt)
                return pathOpt
            if (previousX == point[0]) and (point[0] == future[0]):
                # rospy.loginfo("Redundant X Point")
                pass
            elif (previousY == point[1]) and (point[1] == future[1]):
                # rospy.loginfo("Redundant Y Point")
                pass
            elif((abs(previousX-point[0])==1) and (abs(previousY-point[1])==1) and (abs(future[0]-point[0])==1) and (abs(future[1]-point[1])==1)):
                # rospy.loginfo("Redundant Diagonal Point")
                pass
            else:    
                pathOpt.append(point)
                previousX = point[0]
                previousY = point[1]







        rospy.loginfo("Optimizing path")

        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        pathMsg=Path()
        pathMsg.header.frame_id=('map')

        poses=[]
        for gridPoint in path:
            pose=PoseStamped()
            pose.header.frame_id=('map')
            waypoint=PathPlanner.grid_to_world(mapdata,gridPoint[0],gridPoint[1])
            pose.pose.position=waypoint
            poses.append(pose)
        pathMsg.poses=poses
        self.pub_Path.publish(pathMsg)
        rospy.loginfo("Returning a Path message")
        return pathMsg

    
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, 1)
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)


    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.sleep(0.5)
        # mapdata=PathPlanner.request_map()
        # while(1==1):
        #     self.calc_cspace(mapdata,1)
        #     rospy.sleep(0.5)
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
