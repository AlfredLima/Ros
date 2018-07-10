#!/usr/bin/env python

# Imports
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import *
from tf import transformations

# Comands
'''
source devel/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch
rosrun init_pb [talker.py]
'''

class Robot():
    def __init__(self):

        #initialize
        rospy.init_node('GoForward', anonymous=False) # Init
        rospy.loginfo("To stop TurtleBot CTRL + C") # Inform stop TurtleBot
        rospy.on_shutdown(self.shutdown) # Stop TurtleBot
        # Values default
        self.distEPS = 1e-2
        self.angleEPS = pi/5
        self.communication = Communication()

        # 
        self.position = Point()
        self.diretion = 0
        self.goal = Point()
        self.angleRel = 0

    def setGoal(self, x, y, z = 0):
        self.goal = Point(x,y,z)

    def run(self):

        r = rospy.Rate(10);
        while not rospy.is_shutdown():
            self.aling_and_go()
            r.sleep()
        pass

    def aling(self):

        print('Alinhar')
        # Rotate
        turn_cmd = Twist()
        if self.angleRel > pi :
            turn_cmd.angular.z = -self.angleEPS/5
        else :
            turn_cmd.angular.z = self.angleEPS/5
        self.communication.cmd_vel.publish(turn_cmd)
        
    def go(self):

        print('Andar')
        # Move
        vel = self.distance()
        move_cmd = Twist()
        move_cmd.linear.x = max(min(vel,1),0.1)
        self.communication.cmd_vel.publish(move_cmd)

    def stop(self):
        rospy.loginfo("Chegou:" , self.position.x , self.position.y)
        self.communication.cmd_vel.publish(Twist())
        
    def distance(self):
        return hypot(self.goal.y - self.position.y, self.goal.x - self.position.x)

    def angleRelative(self):
        goalDiretion = atan2( -self.position.y + self.goal.y , -self.position.x + self.goal.x )
        self.angleRel = goalDiretion - self.diretion
        if self.angleRel < 0 :
            self.angleRel += 2*pi
            
    def aling_and_go(self):
        self.position, self.diretion = self.communication.getPosition()
        self.angleRelative()
        
        print( self.position.x , self.position.y )

        if self.distance() > self.distEPS:
            if self.angleRel > self.angleEPS:
                self.aling()
            else :
                self.go()
        else :
            self.stop()


    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        self.communication.cmd_vel.publish(Twist())
        rospy.sleep(1)

class Communication():
    def __init__(self):

        # initiliaze    
        # Create channel of control velocity
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        # Creare channel of position
        self.cmd_pos = rospy.Subscriber('/odom', Odometry, self.getOdometry)
        self.position = Point()
        self.diretion = 0


    def getOdometry(self,msg) :
        self.position = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.diretion = transformations.euler_from_quaternion([ ori.x , ori.y , ori.z , ori.w ])[2]
        
    def getPosition(self):
        return self.position, self.diretion
        

if __name__ == '__main__':
    try:
        robot = Robot()
        robot.setGoal(7,-3)
        robot.run()
    except Exception as e:
        print e
        rospy.loginfo("GoForward node terminated.")
