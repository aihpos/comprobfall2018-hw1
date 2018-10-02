#!/usr/bin/env python

import rospy
from turtlebot_ctrl.msg import TurtleBotState
from gazebo_msgs.srv import GetModelState

'''Subscriber that subscribes to /turtlebot_state topic and relays the position
	of the robot (as x, y coordinates) and also indicates if the goal_position 
	has been reached or not.'''


def callback(msg):
    x = msg.x.data
    y = msg.y.data
    reached = msg.goal_reached.data
    rospy.loginfo('x: {}, y: {}, reached: {}'.format(x, y, reached))


def main():
    rospy.init_node('location')
    rospy.Subscriber("turtlebot_state", TurtleBotState, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
