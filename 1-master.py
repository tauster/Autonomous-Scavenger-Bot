#!/usr/bin/env python

"""
Copyright (c) 2015, Mark Silliman
All rights reserved.

Changes made by Ryegroup, 2017
Last update: Apr. 7, 2017
"""

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

import numpy as np

from std_msgs.msg import Empty, String

import roslaunch
import time
import os
import subprocess
import pyttsx

class GoToPose():
    def __init__(self):
        # Subscriber to the QR code messages.
        self.sub_qr_read = rospy.Subscriber('visp_auto_tracker/code_message', String, self.update_word_log)

        # Publisher for the costmap clearing.
        self.pub_clear_costmap = rospy.Publisher('move_base/clear_costmaps', Empty, queue_size=10)

        # Initial variables
        self.goal_sent = False
        self.current_time = 0
        self.start_time = rospy.get_time()
        self.old_qr_time = 0

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)

        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))


    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 2400 seconds (40 mins) to complete task
        success = self.move_base.wait_for_result(rospy.Duration(2400)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            #self.move_base.cancel_goal()
            self.pub_clear_costmap.publish(Empty())

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
        sys.exit()

    def update_word_log(self, qr_msg):
        if qr_msg.data != "":
            new_qr_time = rospy.get_time()
            if np.absolute(self.old_qr_time - new_qr_time) >= 1:
                word_log = np.genfromtxt('qr_log.csv', delimiter=", ", dtype = "|S")
                N, M = word_log.shape
                new_word = np.array([[N-1, qr_msg.data]], dtype = "|S")
                new_log = np.append(word_log, new_word, axis = 0)
                np.savetxt('qr_log.csv', new_log, delimiter = ', ', fmt = "%s")
                self.old_qr_time = new_qr_time
        else:
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

    # Launching minimal launch to cancel after
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/turtlebot/turtlebot_ws/src/turtlebot/turtlebot_bringup/launch/minimal.launch"])

    launch.start()

    time.sleep(5)

    # Waiting for user input of 1 to start hunt. START OTHER SERVICES FISRT
    start_ready = 0
    while start_ready == 0:
        start_ready = input("Run bash scripts 2-5, THEN input 1: ")
        if start_ready != 1:
            start_ready = 0

    waypoints = np.genfromtxt ('waypoints.csv', delimiter=",")
    num_of_waypoints, M = waypoints.shape

    nonunix_time = 0

    for i in range(0, num_of_waypoints):
        x_position = waypoints[i, 0]
        y_position = waypoints[i, 1]
        
        try:
            navigator = GoToPose()

            # Customize the following values so they are appropriate for your location
            position = {'x': x_position, 'y' : y_position}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = navigator.goto(position, quaternion)

            if success:
                rospy.loginfo("Reached current pose")
            else:
                rospy.loginfo("Failed to reach the current pose, adding a little movement.")
                position = {'x': (x_position + 0.001), 'y' : (y_position + 0.001)}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
                success = navigator.goto(position, quaternion)

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)
            
            navigator.current_time = rospy.get_time()
            nonunix_time = navigator.current_time - navigator.start_time

        except rospy.ROSInterruptException:
            rospy.loginfo("Ctrl-C caught. Quitting")

        # rospy.sleep(1)

    rospy.loginfo("All waypoints completed.")
    rospy.sleep(1)

    # Shutdown Turtlebot's minimal launch.
    launch.shutdown()
    time.sleep(10)

    # Start kobuki's minimal launch in the background.
    subprocess.Popen(["roslaunch","kobuki_auto_docking","compact.launch"])
    time.sleep(10)

    # Process QR code data.
    data_log = np.genfromtxt('qr_log.csv', delimiter=", ", dtype = "|S")

    word_log = data_log[2:, 1]

    word_log_cond = str(word_log[0])

    for i in range(1, len(word_log)):
        if word_log[i - 1] != word_log[i]:
            word_log_cond = word_log_cond + " " + str(word_log[i])

    print(word_log_cond)

    os.system("roslaunch kobuki_auto_docking activate.launch --screen")


    engine = pyttsx.init()
    for i in range(0, 3):
        print(word_log_cond)
        engine.say(word_log_cond)
        engine.runAndWait()
        time.sleep(2)

    while True:
        print(word_log_cond)
        time.sleep(2)

