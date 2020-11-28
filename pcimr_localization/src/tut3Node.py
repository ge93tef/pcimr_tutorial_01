#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from pcimr_simulation.srv import InitPos
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

#same,left,right,backwards,stand Still
#global prob = [0.1,0.5,0.1,0.1,0.2]

#Map Callback
def callback_map(data):
    global map
    map = data.data

#Sensor Callback
def callback_sensor(data):
    global sensor
    sensor = data.ranges

#Move Callback
def callback_move(data):
    global move
    #N,S,W,E
    if(move=='N'):
        moveProbs[0]=initMoveProbs[0]#North
        moveProbs[1]=initMoveProbs[3]#South
        moveProbs[2]=initMoveProbs[1]#West
        moveProbs[3]=initMoveProbs[2]#East
        moveProbs[4]=initMoveProbs[4]#Stay
    elif(move=='S'):
        moveProbs[0]=initMoveProbs[3]#North
        moveProbs[1]=initMoveProbs[0]#South
        moveProbs[2]=initMoveProbs[2]#West
        moveProbs[3]=initMoveProbs[1]#East
        moveProbs[4]=initMoveProbs[4]#Stay
    elif(move=='W'):
        moveProbs[0]=initMoveProbs[2]#North
        moveProbs[1]=initMoveProbs[1]#South
        moveProbs[2]=initMoveProbs[0]#West
        moveProbs[3]=initMoveProbs[3]#East
        moveProbs[4]=initMoveProbs[4]#Stay
    elif(move=='E'):
        moveProbs[0]=initMoveProbs[1]#North
        moveProbs[1]=initMoveProbs[2]#South
        moveProbs[2]=initMoveProbs[3]#West
        moveProbs[3]=initMoveProbs[0]#East
        moveProbs[4]=initMoveProbs[4]#Stay

def getPriorMap():
    map_new = np.copy(empty_map).astype(np.float32)
    for pos in enumerate(map):
        if(map[pos] != 0 & map[pos] != -1):
            xi = map[pos+1]*moveProbs[3]+
            map[pos-1]*moveProbs[2]+
            map[pos+20]*moveProbs[0]+
            map[pos-20]*moveProbs[1]+
            map[pos]*moveProbs[4]
            l=getLikelihood(pos)
            map_new[pos]=l*xi
        else:
            map_new[pos]=map[pos]

#MOVE_IDS = ['N', 'S', 'W', 'E']
#Calculates the multi of the sensorData
def getLikelihood(position):
    works = 0
    #North
    for i in range(1,sensor[0])
        check = map[position-20*i]
        if(check==0 | check == -1):
            if(sensor[0]-i == 1):
                return 0.2
            return 0
    #SOUTH
    for i in range(1,sensor[1])
        check = map[position+20*i]
        if(check==0 | check == -1):
            if(sensor[1]-i == 1):
                return 0.2
            return 0
    #WEST
    for i in range(1,sensor[2])
        check = map[position-i]
        if(check==0 | check == -1):
            if(sensor[2]-i == 1):
                return 0.2
            return 0
    #EAST
    for i in range(1,sensor[3])
        check = map[position+i]
        if(check==0 | check == -1):
            if(sensor[3]-i == 1):
                return 0.2
            return 0
    return 0.8

def getEvidence():
    for pos in enumerate(map):
        if(map[pos] != 0 & map[pos] != -1):
            xi = map[pos+1]*0.25+
            map[pos-1]*0.25+
            map[pos+20]*0.25+
            map[pos-20]*0.25+
            map[pos]*0.25

def getBayesan():
    newMap = getPriorMap()



def localize():
    global moveProbs
    mapData = rospy.Subscriber('/map', OccupancyGrid, callback_map)
    sensorData = rospy.Subscriber('/scan', LaserScan, callback_sensor)
    moveData = rospy.Subscriber('/move', String, callback_move)

    geom = rospy.Publisher('/robot_pos', Point , queue_size=10)
    visual = rospy.Publisher('/visualization/robot_pos', Marker, queue_size=10)
    navi = rospy.Publisher('/map_updates', OccupancyGrid, queue_size=10)

    initMoveProbs = rospy.get_param('~robot_move_probabilities', [0.9, 0.04, 0.04, 0.0, 0.02])
    moveProbs=copy(initMoveProbs)
    rospy.init_node('localize', anonymous=True)
    rate = rospy.Rate(2)

    subrate = rospy.Rate(1)
    pubrate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        subrate.sleep()
        myAlgo()
        #rospy.signal_shutdown("Done")

if __name__ == '__main__':
    localize()