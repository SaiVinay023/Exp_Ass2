#!/usr/bin/env python

## @file move_ball_around.py
# @brief This node permits to the ball to move around, wait for a while and disappear.
#

#ros + python library
import rospy
import roslib
import time
import math
import random

#actionlib
import actionlib
import actionlib.msg
import exp_assignment2.msg

#message ros
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry


## client of actionlib server for moving the ball
def move_ball(target):

    ## Creates the SimpleActionClient, passing the type of the action (PlanningAction) to the constructor
    client = actionlib.SimpleActionClient(
        '/reaching_goal', exp_assignment2.msg.PlanningAction)

    ## Waits until the action serve has started up and started listening for goal
    client.wait_for_server()

    ## Create a goal to sent to action server
    goal = exp_assignment2.msg.PlanningGoal()
    goal.target_pose.pose.position.x = target[0]
    goal.target_pose.pose.position.y = target[1]
    goal.target_pose.pose.position.z = target[2]

    ## Send the goal to the action server
    client.send_goal(goal)

    ## waits for the serve to finish performing the action.
    client.wait_for_result()

    return client.get_result()

## Main function
def main():
    ## initialization node
    rospy.init_node('move_ball_around')
    ## loop
    while True:
        ## move for two times in the arena
        move_ball([random.randrange(-8,8,1), random.randrange(-8,8,1), 0 ])
        ## wait for a while
        time.sleep(50)
        move_ball([random.randrange(-8,8,1), random.randrange(-8,8,1), 0 ])
        time.sleep(50)

        ## move along z axis to disappear
        move_ball([0,0,20])
        time.sleep(50)
if __name__ == '__main__':
    main()