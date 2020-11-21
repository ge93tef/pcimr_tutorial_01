#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pcimr_simulation.srv import InitPos
from geometry_msgs.msg import Twist
import numpy as np

global pub_velo
global maxVelo
global minDist
global global_velo
global global_velo_linear
global maxDist
pub_velo = Twist()
pub_velo.linear.x=0.0
pub_velo.linear.y=0.0
pub_velo.linear.z=0.0
pub_velo.angular.x=0.0
pub_velo.angular.y=0.0
pub_velo.angular.z=0.0
global_velo = pub_velo
global_velo_linear = np.array([pub_velo.linear.x,pub_velo.linear.y ,pub_velo.linear.z]) #Initial velo as array

#Scan around me
def callback_Scan(data):
    global global_scan
    global_scan = np.array(data.ranges)

def callback_Velo(data):
    global_velo= [data.linear.x, data.linear.y, data.linear.z]  
    global_velo_linear = np.array([data.linear.x,data.linear.y ,data.linear.z]) 

#Any distance is beneath the min Distance = Stop everything
def emergencyStop():
    pub_velo.linear.x=0.0
    pub_velo.linear.y=0.0
    pub_velo.linear.z=0.0
    pub_velo.angular.x=0.0
    pub_velo.angular.y=0.0
    pub_velo.angular.z=0.0

def setMaxVelo():
    if(pub_velo.linear.x > maxVelo ):
        pub_velo.linear.x = maxVelo
    if(pub_velo.linear.y > maxVelo):
        pub_velo.linear.y = maxVelo
    if(pub_velo.linear.z > maxVelo):
        pub_velo.linear.z= maxVelo

def adjustSpeed():
    rangeDist = maxDist-minDist # Allowed  Range
    newVelo = maxVelo*(rangeDist/maxDist)#Calc new Velo
    #If any Velo is above new velo --> adjust
    if(pub_velo.linear.x > newVelo ):
        pub_velo.linear.x = newVelo
    if(pub_velo.linear.y > newVelo):
        pub_velo.linear.y = newVelo
    if(pub_velo.linear.z > newVelo):
        pub_velo.linear.z= newVelo


def checkWall():
    global pub_velo
    global maxVelo
    global minDist
    global global_velo
    global global_velo_linear
    global maxDist
    global global_scan
    maxVelo = 1.0 #max velocity
    minDist = 0.10 #min DIstance to object
    maxDist = 3.0 #Distance until the robot can drive with maxVelo
    scan = rospy.Subscriber('/scan', LaserScan, callback_Scan)
    velo_move = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    velo = rospy.Subscriber('cmd_vel', Twist, callback_Velo)

    rospy.init_node('checkWall', anonymous=True)
    rate = rospy.Rate(1)


    subrate = rospy.Rate(1)
    pubrate = rospy.Rate(1)

    while not rospy.is_shutdown():
        subrate.sleep()
        #Check if robot is already to close
        if((global_scan < minDist).any()):
            emergencyStop()
            velo_move.publish(pub_velo)
            print("Robot Stopped")
        #Check if any linear velocity reached max
        elif((global_velo_linear > maxVelo).any()):
            setMaxVelo()
            velo_move.publish(pub_velo)
        #Adjust speed relative to distance
        elif((global_scan < maxDist).any()):
            adjustSpeed()
            velo_move.publish(pub_velo)
        #velo_move.publish(pub_velo)
        #print("robot scan", global_scan)
        print("global_velo", global_velo)
        # rospy.signal_shutdown("Done")

if __name__ == '__main__':
    checkWall()