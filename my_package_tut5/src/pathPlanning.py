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
    global map
    global height
    global width
    height = data.info.height
    width = data.info.width
    map = np.array(data.data)

#get the Position
def callback_Pos(data):
    global pos
    pos = data
    #print("Position: \n", data)

#get the Goal discretize and check if it is valid 
def callback_goal(data):
    global map
    global height
    global goalDisc
    goal= np.array([data.pose.position.x,data.pose.position.y])
    bins = np.arange(1, 20, 1)
    goalDisc = np.digitize(goal,bins, right=False)
    print("Goal", goalDisc)
    x = goalDisc[0]
    y = goalDisc[1]
    if(map[y*height+x]== 100 or map[y*height+x]== -1):
        print('Goal is not valid')

def alreadyInList(list, id, cost):
    if(np.any(list[:, 0] == id)):

#real cost per movement = 1
#Estimated cost = distance between a and b
def algo():
    global pos #MyPosition
    global map #MyMap
    global height
    global goalDisc #My Discretized Goal
    costMap = np.zeros([map.shape[0]]) # copy map for cost estimates
    for i in range(0,map.shape[0]):
        costMap[i]=abs((pos.y*height+pos.x)-i) #get estimated cost and don't care about obstacles
    costMap[pos.y*height+pos.x]=0 #Start has no costs
    nextToCheck = np.array([pos.y*height+pos.x, 0]) #init nextToCheck id, cost
    checked = np.array([])#GAlready checked
    searching = True
    while(searching):
        #Get minimum from nextToCheck, are all 4 points existing? And Are they valid?Is the min our goal?
        row = np.argmin(nextToCheck[:,1], axis=1)
        i=nextToCheck[row][0] #get row id
        if(i==pos.y*height+pos.x):
            print("Done")
            searching=False
        if((i+1)<map.shape[0]):
            if(map[i+1]!= 100 and map[i+1] != -1 and np.any(nextToCheck[:, 0] != i+1)):
                np.insert(nextToCheck,0, [i+1, 1+costMap[i+1]+costMap[i]], axis=0) # insert id and costs
        if(i-1>=0):
            if(map[i-1]!= 100 and map[i-1] != -1 and np.any(nextToCheck[:, 0] != i-1)):
                np.insert(nextToCheck,0, [i-1, 1+costMap[i-1]+costMap[i]], axis=0)
        if(i+10<map.shape[0]):
            if(map[i+20]!= 100 and map[i+20] != -1 and np.any(nextToCheck[:, 0] != i+20)):
                np.insert(nextToCheck,0, [i+20, 1+costMap[i+20]+costMap[i]], axis=0)
        if(i-20>=0):
            if(map[i-20]!= 100 and map[i-20] != -1 and np.any(nextToCheck[:, 0] != i-20)):
                np.insert(nextToCheck,0, [i-20, 1+costMap[i+-20]+costMap[i]], axis=0)
        np.insert(checked, 0,[i-20, 1+costMap[i+-20], axis=0)


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