#! /usr/bin/env python3

from __future__ import print_function
import sys
import rospy
# Brings in the SimpleActionClient
import actionlib
import sciroc_msgs.msg


def image_action_client():
    # Creates the SimpleActionClient
    client = actionlib.SimpleActionClient('image_action', sciroc_msgs.msg.PerceptionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("Waiting for server")
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = sciroc_msgs.msg.PerceptionGoal(mode=2)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('image_action_client_py')
        result = image_action_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
