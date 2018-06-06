#!/usr/bin/env python

"""
Copyright (c) 2015, Mark Silliman
All rights reserved.

Changes made by Ryegroup, 2017
Last update: Apr. 5, 2017
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

import sys
import roslaunch
import time
import os
import subprocess

class GoToPose():
    def __init__(self):

        self.sub_qr_read = rospy.Subscriber('visp_auto_tracker/code_message', String, self.update_word_log)

        self.pub_clear_costmap = rospy.Publisher('move_base/clear_costmaps', Empty, queue_size=10)

        self.goal_sent = False
        self.current_time = 0
        self.start_time = rospy.get_time()

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

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(900)) 

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
            word_log = np.genfromtxt('qr_log.csv', delimiter=", ", dtype = "|S")
            N, M = word_log.shape
            new_word = np.array([[N-1, qr_msg.data]], dtype = "|S")
            new_log = np.append(word_log, new_word, axis = 0)
            np.savetxt('qr_log.csv', new_log, delimiter = ', ', fmt = "%s")
        else:
            pass


if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/turtlebot/turtlebot_ws/src/turtlebot/turtlebot_bringup/launch/minimal.launch"])

    launch.start()

    start_ready = 0
    while start_ready == 0:
        start_ready = input("Run bash scripts 2-5, then input 0: ")
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
                rospy.loginfo("Failed to reach the current pose")

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)
            
            navigator.current_time = rospy.get_time()
            nonunix_time = navigator.current_time - navigator.start_time

        except rospy.ROSInterruptException:
            rospy.loginfo("Ctrl-C caught. Quitting")

        # rospy.sleep(1)

    rospy.loginfo("All waypoints completed.")
    rospy.sleep(1)

    launch.shutdown()
    time.sleep(10)

    subprocess.Popen(["roslaunch","kobuki_auto_docking","compact.launch"])

    time.sleep(10)

    os.system("roslaunch kobuki_auto_docking activate.launch --screen")

