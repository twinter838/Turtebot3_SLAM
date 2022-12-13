#!/usr/bin/env python3

import math
import rospy
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import tf.transformations
from std_msgs.msg import Empty,String

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ### Initialize node, name it 'lab2'
        # TODO
        self.py=0
        self.px=0
        self.pth=0
        rospy.init_node('Drive')
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_awaitingPath=rospy.Publisher('/drive/awaitingPath',Empty, queue_size= 10)
        self.pub_currpose=rospy.Publisher('/drive/currpose',PoseStamped, queue_size= 10)
        self.pub_robot_state=rospy.Publisher('/robot_state',String, queue_size=10)
        rospy.Subscriber('/robot_state',String,self.handleRobotState)

        # TODO
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry , self.update_odometry)
        # TODO
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        # TODO
        rospy.Subscriber('/path_planner/pathOld',Path,self.update_path)
        rospy.Subscriber('/path_planner/path', Path,self.drive_path)
        self.pathRunning=False
        self.listener=tf.TransformListener()
        self.newPathRecieved=False
        # delete this when you implement your code
        
    def handleRobotState(self,msg):
        self.robotState=msg.data

    def spin(self):
        while(self.robotState=="Localization"):
            self.send_speed(0,0.6)
        self.send_speed(0,0)
        
    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        msg_cmd_vel = Twist()
        # Linear velocity
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        # Angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed
        # Send command
        self.pub_cmd_vel.publish(msg_cmd_vel)

    def update_path(self,msg):
        rospy.loginfo("New Path Recieved")
        self.newPathRecieved=True
        self.path=msg
        rospy.sleep(0.1)
        self.newPathRecieved=False

    def drive_path(self,msg):
        rospy.sleep(0.11)
        if(self.robotState=="Exploration"):
            dist=0
            driveList=[]
            if(len(msg.poses)>1):
                driveList.append(msg.poses[0],0.08)
                for index,point in enumerate(msg.poses):
                    if(index<(len(msg.poses)-2)):
                        point1=msg.poses[index+1]
                        point2=msg.poses[index+2]
                        dist+=math.sqrt(math.pow(point1.pose.position.x-point2.pose.position.x,2) + math.pow(point1.pose.position.x-point2.pose.position.y,2))
                        if(dist<.1):
                            driveList.append(point1)
                        else:
                            break

            if(not(self.path.poses==msg.poses)):
                rospy.loginfo("Aborting Path")
                return None
            else:
                for i in driveList:
                    self.arc_to(i,0.2)
            rospy.sleep(3)
            if(self.path.poses==msg.poses):
                self.pub_awaitingPath.publish()
        elif(self.robotState=="Return" or self.robotState=="Localized"):
            for i in msg.poses:
                if(not(self.path.poses==msg.poses)):
                    rospy.loginfo("Aborting Path")
                    return None
                else:
                    self.arc_to(i,0.2)





        # point=msg.poses[0]
        # waypointX = point.pose.position.x
        # waypointY = point.pose.position.y
        # yawAngle=math.atan2(waypointY-self.py,waypointX-self.px)
        # pointGoal=msg.poses[len(msg.poses)-1]
        # waypointXGoal = pointGoal.pose.position.x
        # waypointYGoal = pointGoal.pose.position.y
        # rospy.loginfo("Driving too")
        # rospy.loginfo(waypointX)
        # rospy.loginfo(self.px)
        # rospy.loginfo(waypointY)
        # rospy.loginfo(self.py)
        # rospy.loginfo(msg.poses)
        
        # errorTheta = self.normalize_angle(self.normalize_angle_positive(yawAngle) - self.normalize_angle_positive(self.pth))
        # errorDistance = math.sqrt(math.pow(waypointX - self.px,2) + math.pow(waypointY - self.py,2))
        # errorDistanceGoal = math.sqrt(math.pow(waypointXGoal - self.px,2) + math.pow(waypointYGoal - self.py,2))

        # if(abs(errorTheta)>1 and errorDistance>0.1 and self.newPathRecieved==False):
        #     rospy.loginfo("Correcting Orientation")
        #     self.rotate(self.normalize_angle(yawAngle-self.pth),1)
        # if(self.newPathRecieved==False and errorDistance>0.1):
        #     self.arc_to(msg.poses[0])
        # # errorDistanceGoal = math.sqrt(math.pow(waypointXGoal - self.px,2) + math.pow(waypointYGoal - self.py,2))
        # # if(len(msg.poses)>1 and errorDistanceGoal>0.1 and self.newPathRecieved==False):
        # #     point=msg.poses[1]
        # #     waypointX = point.pose.position.x
        # #     waypointY = point.pose.position.y
        # #     errorDistance = math.sqrt(math.pow(waypointX - self.px,2) + math.pow(waypointY - self.py,2))
        # #     if(self.newPathRecieved==False and errorDistance>0.1):
        # #         rospy.loginfo("Moving to next point")
        # #         self.arc_to(msg.poses[1])
        # #     elif(errorDistance<0.1):
        # #         rospy.loginfo("Reached Destination")
        # #         self.send_speed(0,0)        # rospy.sleep(2)
        # # if(self.newPathRecieved==False):
        # #     self.send_speed(0,0)
     
        



        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        ### REQUIRED CREDIT
        distanceOld=math.sqrt(math.pow(self.px,2)+math.pow(self.py,2))
        oldX=self.px
        oldY=self.py
        print("Starting Movement")
        print(abs(math.sqrt(math.pow(self.px,2)+math.pow(self.py,2))))
        print(distance)
        while(abs(math.sqrt(math.pow(self.px-oldX,2)+math.pow(self.py-oldY,2)))<distance):
            self.send_speed(linear_speed,0)
            # print(abs(math.sqrt(math.pow(self.px,2)+math.pow(self.py,2))-distanceOld))
            rospy.sleep(0.1)
        print("Stopping Movement")
        self.send_speed(0,0)



    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        KpHeading = 1.5
        KiHeading = 0.02
        errorThetaAcc = 0
        errorTheta = self.normalize_angle(self.normalize_angle_positive(angle) - self.normalize_angle_positive(self.pth))
        ### REQUIRED CREDIT
        angleOld=self.pth
        angle=self.normalize_angle(angle+angleOld)
        while abs(errorTheta) > 0.03 and self.newPathRecieved==False:
            errorTheta = self.normalize_angle(self.normalize_angle_positive(angle) - self.normalize_angle_positive(self.pth))
            effortTheta = (errorTheta * KpHeading) + (KiHeading * errorThetaAcc)
            errorThetaAcc=errorTheta+errorThetaAcc
            self.send_speed(0,effortTheta)
            rospy.sleep(0.05)

        

    def normalize_angle_positive(self,angle):
        """ Normalizes the angle to be 0 to 2*pi
        It takes and returns radians. """
        return math.fmod(math.fmod(angle, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi)


    def normalize_angle(self,angle):
        """ Normalizes the angle to be -pi to +pi
        It takes and returns radians."""
        a = self.normalize_angle_positive(angle)
        if a > math.pi:
            a -= 2.0 *math.pi
        return a

    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT
        ### Take in the pose from move_base_simple/goal and save it in the function
        waypointX=  msg.pose.position.x
        waypointY = msg.pose.position.y
        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll , pitch , yaw) = euler_from_quaternion(quat_list)
        waypointYaw = yaw
        print(waypointX)
        print(waypointY)
        print(waypointYaw)
        ### Calculate distance and angle to waypoint
        distance=math.sqrt(math.pow(self.px-waypointX,2)+math.pow(self.py-waypointY,2))
        print(distance)
        yawAngle=math.atan2(waypointY-self.py,waypointX-self.px)
        ### Rotate to target
        print("Rotating")
        print(yawAngle)
        self.rotate(self.normalize_angle(yawAngle-self.pth),1)
        rospy.sleep(0.1)
        ### Drive to target
        print('Driving')
        print(distance)
        #self.drive(distance,0.5)
        self.smooth_drive(distance,2)
        ### Rotate to face the waypoints direction        
        # self.rotate(self.normalize_angle(waypointYaw-self.pth),1)
        ### Correct Error if it is too large
    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        # TODO

        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.px = trans[0]
            self.py = trans[1]
            quat_orig = rot
            quat_list = [rot[0], rot[1], rot[2], rot[3]]
            (roll , pitch , yaw) = euler_from_quaternion(quat_list)
            self.pth = yaw
            pose=PoseStamped()
            pose.pose.position.x=self.px
            pose.pose.position.y=self.py
            pose.pose.position.z=0
            pose.pose.orientation.x=rot[0]
            pose.pose.orientation.y=rot[1]
            pose.pose.orientation.z=rot[2]
            pose.pose.orientation.w=rot[3]
            pose.header.frame_id=('map')
            self.pub_currpose.publish(pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass



    def arc_to(self, position,maxSpeed):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        print("Starting Arc to")
        #PID values
        KpHeading = 1
        KiHeading = 0.015
        KpDistance = 1.1
        KiDistance = 0.001

        errorThetaAcc = 0
        errorDistanceAcc = 0
       

        waypointX = position.pose.position.x
        waypointY = position.pose.position.y

        quat_orig = position.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        errorDistance = math.sqrt(math.pow(waypointX - self.px,2) + math.pow(waypointY - self.py,2))
        (roll , pitch , yaw) = euler_from_quaternion(quat_list)
        waypointYaw = yaw
        yawAngle=math.atan2(waypointY-self.py,waypointX-self.px)
        ## Rotate to target
        print("Rotating")
        print(yawAngle)
        errorTheta = self.normalize_angle(self.normalize_angle_positive(math.atan2((waypointY - self.py),(waypointX - self.px))) - self.normalize_angle_positive(self.pth))


        self.rotate(self.normalize_angle(yawAngle-self.pth),1)
        while(errorDistance) > 0.03 and self.newPathRecieved==False:
            #Errors
            errorTheta = self.normalize_angle(self.normalize_angle_positive(math.atan2((waypointY - self.py),(waypointX - self.px))) - self.normalize_angle_positive(self.pth))

            errorDistance = math.sqrt(math.pow(waypointX - self.px,2) + math.pow(waypointY - self.py,2))

            #Efforts
            effortTheta = (errorTheta * KpHeading) + (KiHeading * errorThetaAcc)
            effortDistance = (errorDistance * KpDistance) + (KiDistance * errorDistanceAcc)

            errorThetaAcc=errorTheta+errorThetaAcc
            errorDistanceAcc=errorDistance+errorDistanceAcc
            if(effortDistance>maxSpeed):
                effortDistance=maxSpeed

            self.send_speed(effortDistance,effortTheta)

            rospy.sleep(0.05)
        if(self.newPathRecieved==True):
            rospy.loginfo("Aborting Arc as new path recieved")
        self.send_speed(0,0)

    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """

        ### EXTRA CREDIT
        oldX=self.px
        oldY=self.py
        error=distance
        kp=0.3
        ki=0.005
        errorACC=0

        while(abs(error>0.05) or effort>0.01):
            error=distance-math.sqrt(math.pow(self.px-oldX,2)+math.pow(self.py-oldY,2))
            effort=kp*error+ki*errorACC
            print("Effort is")
            print(effort)
            print(error)
            errorACC=error+errorACC
            if effort>linear_speed:
                effort=linear_speed
            self.send_speed(effort,0)
            rospy.sleep(0.05)
        self.send_speed(0,0)
        print("Finished Smooth Drive")

    def run(self):
        rospy.wait_for_message('/odom',Odometry)
        # self.pub_robot_state.publish("Exploration")
        rospy.sleep(5)
        self.spin()
        self.pub_awaitingPath.publish()
        rospy.spin()
if __name__ == '__main__':
    Lab2().run()
