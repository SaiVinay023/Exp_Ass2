#!/usr/bin/env python

## @file state_machine.py
# @brief This node implements a state machine which permits to move around and to search for a ball, go to sleep, and play with the ball when the last one is found.
#


#ros + python library
import rospy
import roslib
import time
import math
import random

#smach library
import smach
import smach_ros

#message ros
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float64
from tf import transformations
from gazebo_msgs.msg import LinkState
from nav_msgs.msg import Odometry

#import numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

#Open CV
import cv2
from sensor_msgs.msg import CompressedImage

#actionlib
import actionlib
import actionlib.msg
import exp_assignment2.msg

VERBOSE = False

vel = Twist()
angle_camera = Float64()


global count
global camera_rotate
global count2
global DetectedBall


## User Action function
def user_action():
    ## random choice between search for the ball or go to sleep
    return random.choice(['searchball', 'sleep'])



## client of actionlib server for moving the dog robot
def move_dog(target):

    ## Creates the SimpleActionClient, passing the type of the action (PlanningAction) to the constructor
    client = actionlib.SimpleActionClient(
        '/robot_reaching_goal', exp_assignment2.msg.PlanningAction)

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




## Publisher
image_pub = rospy.Publisher("/output/image_raw/compressed",
                            CompressedImage, queue_size=1)

vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)


## Callback for Normal State for subscribe to a Camera1 : it checks if the ball is on the arena, if it is, a parameter (DetectedBall) becomes 1 and the robot goes to play, otherwise the parameter becomes 0 and the robot moves randomly.
def Normal_clbk(ros_data):

    global count2
    global DetectedBall
    global SearchBallSub

    while rospy.get_param('count2') == 360:
        time.sleep(1)


        ## direct conversion to CV2
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
    ## define color
    greenLower = (50, 50, 20)
    greenUpper = (70, 255, 255)
    ## create a mask
    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    ## find color
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None



    ## only proceed if at least one contour was found

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        ## only proceed if the radius meets a minimum size
        if radius > 10:


            cv2.circle(image_np, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(image_np, center, 5, (0, 0, 255), -1)

            ## the ball is found, set parameter to 1
            rospy.set_param('DetectedBall',1)

            ## the ball is not found
    else:

        vel_ = Twist()
        ## set an angular velocity to search around
        vel_.angular.z = 0.5
        vel_pub.publish(vel_)
        count2 = rospy.get_param('count2')
        count2 = count2 + 1
        rospy.set_param('count2', count2)


        ## show window
    cv2.imshow('window', image_np)
    cv2.waitKey(2)



## Istance called in play state to track the ball
class image_feature:
    ## initialization
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''

        ## publisher
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)

        self.vel_pub = rospy.Publisher("/robot/cmd_vel",
                                       Twist, queue_size=1)


        ## subscriber
        self.subscriber = rospy.Subscriber("/robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

    ## callback to subscribe to camera1 when the state is Play: when the ball is found, a parameter (camera_rotate) becomes 1 and the robot starts to rotate the head, otherwise, a parameter becomes 0 and the robot search around for a bit time.
    def callback(self, ros_data):
        global count
        global camera_rotate
        count = rospy.get_param('count')

        while count == 360:
            time.sleep(1)




        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        ## direct conversion to CV2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        ## define color
        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)
        ## create a mask
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        ## find color
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        ##  only proceed if at least one contour was found
        if len(cnts) > 0:

            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid




            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:

                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                vel = Twist()
                ## define a velocity to track the ball
                vel.angular.z = 0.002*(center[0]-400)
                vel.linear.x = -0.01*(radius-100)

                self.vel_pub.publish(vel)
                ## if the robot reaches the ball then it wanna rotate the head: thus the parameter "rotate_camera"  is setted to 1
                if (vel.linear.x < 0.01) & (vel.angular.z < 0.01):
                    vel.angular.z = 0
                    vel.linear.x = 0

                    self.vel_pub.publish(vel)
                    rospy.set_param('camera_rotate', 1)


        ## if the ball is not found, searches around
        else:

            vel = Twist()
            vel.angular.z = 0.5
            self.vel_pub.publish(vel)
            count = count + 1

            rospy.set_param('count', count)

            ## after a bit time in which the ball is not found, the robot go to normal state.
            if rospy.get_param('count') == 259:

                vel.angular.z = 0.0
                self.vel_pub.publish(vel)
                self.subscriber.unregister()





                ## show the window
        cv2.imshow('window', image_np)
        cv2.waitKey(2)




    ## Normal state definition
class Normal(smach.State):

    ## inizialization
    def __init__(self):
        ## 2 outcomes defined
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.command = String()




        ## execution
    def execute(self, userdata):


        global SearchBallSub, count2, DetectedBall

        rospy.set_param('count',0)

        while True:

            time.sleep(5)
            ## read command (choice random between move random or go to sleep)
            self.command = user_action()

            rospy.loginfo('command received is %s', self.command)

            ## command is sleep
            if self.command == "sleep":
                ## switch in sleep state
                return 'outcome1'

            ## command is searchball
            if self.command == "searchball":

                ## subscriber to camera to search the ball
                SearchBallSub = rospy.Subscriber("camera1/image_raw/compressed",
                                                 CompressedImage, Normal_clbk,  queue_size=1)


                while rospy.get_param('count2') < 360:
                    time.sleep(1)
                    ## if the ball is found, switch to play state
                    if rospy.get_param('DetectedBall') == 1:
                        rospy.loginfo('The ball is here, Play!')
                        rospy.set_param('DetectedBall', 0)
                        rospy.set_param('count2',0)
                        SearchBallSub.unregister()
                        return 'outcome2'
                ## the ball is not found
                SearchBallSub.unregister()
                rospy.set_param('count2',0)
                rospy.loginfo('The ball is not here')
                ## call the function "move_dog" to move random in the arena
                move_dog([random.randrange(-8,8,1), random.randrange(-8,8,1), 0])



            ## Sleep State definition
class Sleep(smach.State):
    ## Initialization
    def __init__(self):
        ## 1 outcome defined : Normal
        smach.State.__init__(self, outcomes=['outcome1'])


    ## Execution
    def execute(self, userdata):
        rospy.loginfo('Go to sleep')
        move_dog([7,7,0])

        ## sleep for a while
        time.sleep(14)

        return 'outcome1'


## Play state definition
class Play(smach.State):
    ## initialization
    def __init__(self):
        ## 1 outcome defined : Normal
        smach.State.__init__(self, outcomes=['outcome2'])
        ## publisher for move the head
        self.angle_pub = rospy.Publisher("/robot/joint_position_controller/command",
                                         Float64, queue_size=1)



        ## Execution
    def execute(self, userdata):


        ## called the instance to track the ball
        ic = image_feature()



        while rospy.get_param('count') < 259:

            time.sleep(1)
            ## look for the "camera_rotate" parameter
            replay = rospy.get_param('camera_rotate')

            if replay == 1:
                rospy.set_param('count', 0)
                rospy.loginfo('rotating head')
                angle_camera = Float64()
                angle_camera.data = 0.0

                ## rotate head
                while angle_camera.data < 0.5: #head turns left
                    angle_camera.data = angle_camera.data + 0.1
                    self.angle_pub.publish(angle_camera)
                    time.sleep(1)
                time.sleep(3)

                while angle_camera.data > 0.01: #head turns right
                    angle_camera.data = angle_camera.data - 0.1
                    self.angle_pub.publish(angle_camera)
                    time.sleep(1)

                while angle_camera.data > -0.5:
                    angle_camera.data = angle_camera.data - 0.1
                    self.angle_pub.publish(angle_camera)
                    time.sleep(1)
                time.sleep(3)

                while angle_camera.data < -0.01:
                    angle_camera.data = angle_camera.data + 0.1
                    self.angle_pub.publish(angle_camera)
                    time.sleep(1)



                rospy.set_param('camera_rotate', 0)
                time.sleep(30)


        return 'outcome2'
















## Main Function definition
def main():
    ## init the ros node
    rospy.init_node('state_machine')
    rospy.set_param('count2',0)
    rospy.set_param('count',0)
    rospy.set_param('camera_rotate',0)
    rospy.set_param('DetectedBall',0)

    ## Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    ## Open the container
    with sm:
        ## Add states to the container
        smach.StateMachine.add('NORMAL', Normal(),
                               transitions={'outcome1':'SLEEP',
                                            'outcome2':'PLAY'})

        smach.StateMachine.add('SLEEP', Sleep(),
                               transitions={'outcome1':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(),
                               transitions={'outcome2':'NORMAL'})

    ## Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    ## Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

