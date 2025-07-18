#!/usr/bin/env python3
import rospy
import actionlib
from wormhole_navigation.msg import NavigateToGoalAction, NavigateToGoalGoal

def send_goal():
    client = actionlib.SimpleActionClient('navigate_to_goal', NavigateToGoalAction)
    client.wait_for_server()
    
    goal = NavigateToGoalGoal()
    goal.target_map = "map1"
    goal.target_x = 8.0
    goal.target_y = -7.0
    
    client.send_goal(goal)
    client.wait_for_result()
    
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('navigation_client')
    result = send_goal()
    print("Result:", result.success, result.message)
