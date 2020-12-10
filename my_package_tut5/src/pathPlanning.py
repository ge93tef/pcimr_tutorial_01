#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pcimr_simulation.srv import InitPos
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.msg import Path
from geometry_msgs.msg import Point,PoseStamped
import numpy as np


#Get the Map
def callback_map(data):
    height = data.info.height
    width = data.info.width
    map = data.data

#get the Position
def callback_Pos(data):
    pos = data
    #print("Position: \n", data)

#get the Goal check if it is valid
def callback_goal(data):
    print("Goal")

def pathPlanning():
    #Subscriber
    robotPosition = rospy.Subscriber('/robot_pos', Point, callback_Pos)
    map = rospy.Subscriber('/map', OccupancyGrid, callback_map)
    goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_goal)
    #Publisher
    #navigate = rospy.Publisher('/global_path', Path, queue_size=10)
    #visualGoal = rospy.Publisher('/visualization/goal', Marker, queue_size=10)
    #visualPath = rospy.Publisher('/visualization/path', Marker, queue_size=10)

    rospy.init_node('pathPlanning', anonymous=True)
    rate = rospy.Rate(2)


    subrate = rospy.Rate(1)
    pubrate = rospy.Rate(0.5)

    while (not rospy.is_shutdown()):
        rospy.spin()


if __name__ == '__main__':
    pathPlanning()