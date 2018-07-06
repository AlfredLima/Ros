#!/usr/bin/env python

# Imports
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from math import radians

class Ros():
    def __init__(self):

        #initialize
        rospy.init_node('GoForward', anonymous=False) # Init
        rospy.loginfo("To stop TurtleBot CTRL + C") # Inform stop TurtleBot
        rospy.on_shutdown(self.shutdown) # Stop TurtleBot

        self.comunication = Comunication()
        

    def run(self):

        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);
        # Twist is a datatype for velocity
        move_cmd = Twist()
        move_cmd.linear.x = 0.2

        turn_cmd = Twist()
        turn_cmd.angular.z = radians(45)


        while not rospy.is_shutdown():
            for _ in range(30):
                self.comunication.cmd_vel.publish(move_cmd)
                r.sleep()
            for _ in range(5):
                self.comunication.cmd_vel.publish(turn_cmd)
                r.sleep()
        pass

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.comunication.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

class Comunication():
    def __init__(self):

        # initiliaze    
        # Create channel of control velocity
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)



if __name__ == '__main__':
    try:
        ros = Ros()
        ros.run()
    except:
        rospy.loginfo("GoForward node terminated.")