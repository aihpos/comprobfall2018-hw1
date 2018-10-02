#!/usr/bin/env python

import sys
import rospy
from turtlebot_ctrl.srv import TurtleBotControl
from geometry_msgs.msg import Point

'''Sends a request to the turtlebot_control ros service provided. Sends a 3-D
	coordiante value to the service that correspond to the location the robot is to move'''


def sendRequest(req):
    rospy.wait_for_service('turtlebot_control')

    try:
        turtlebot_control = rospy.ServiceProxy("turtlebot_control", TurtleBotControl)
        print (req)
        resp = turtlebot_control(Point(req[0], req[1], req[2]))
        print (type(resp))
    except rospy.ServiceException, e:
        print ("Service call failed: %s", e)


if __name__ == "__main__":
    if len(sys.argv) == 3:
        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            z = 0.0
        except:
            print("error taking args")

    sendRequest([x, y, z])
