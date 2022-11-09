#!/usr/bin/env python3

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

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
        rospy.init_node('Lab2')
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # TODO
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry , self.update_odometry)
        # TODO
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        # TODO
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)
        # delete this when you implement your code



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
        ### REQUIRED CREDIT
        angleOld=self.pth
        angle=self.normalize_angle(angle+angleOld)
        while abs(self.pth-angle) > 0.1:
            angle=self.normalize_angle(angle)
            if(angle>angleOld):
                self.send_speed(0,aspeed)
            if(angle<angleOld):
                self.send_speed(0,-aspeed)
            rospy.sleep(0.1)
        self.send_speed(0,0)

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
        rospy.sleep(2)
        ### Drive to target
        print('Driving')
        print(distance)
        self.drive(distance,0.5)
        #self.smooth_drive(distance,2)
        ### Rotate to face the waypoints direction        
        self.rotate(self.normalize_angle(waypointYaw-self.pth),1)
        ### Correct Error if it is too large
    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        # TODO
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll , pitch , yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw



    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



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
        kp=0.01
        ki=0.001
        errorACC=0

        while(error>0.05 or effort>0.1):
            error=math.sqrt(math.pow(self.px-oldX,2)+math.pow(self.py-oldY,2))
            effort=kp*error+ki*errorACC
            errorACC=error+errorACC
            if effort>linear_speed:
                effort=linear_speed
            self.send_speed(effort,0)
            print(effort)
            rospy.sleep(0.05)
        self.send_speed(0,0)


    def run(self):
        rospy.sleep(0.5)
        # while not rospy.is_shutdown():
            
        #     rospy.sleep(0.1)

        #     print("Pose X="+self.px)
        #     print("Pose Y="+self.py)
        rospy.spin()
if __name__ == '__main__':
    Lab2().run()
