#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pcimr_simulation.srv import InitPos
from geometry_msgs.msg import Twist
import random
import numpy as np

#Actions (To be Change?)
#0: Stop
#1: Speed X
#2: Slow X
#3: Speed Y
#4: Slow Y

#Rewards:
#0: -100 Hit Wall
#1: +10 Get Faster
#2: -5 Get Slower
#3: -30 Stop
#4: +100 Finish Round

class learnToMove:

    def __init__(self):
        self.learnRate = rospy.get_param('~/learnRate', 0.85) #LearnRate
        self.epsilon = rospy.get_param('~/epsilon', 0.3) #Randomness
        self.gamma = rospy.get_param('~/gamma', 0.8) #forQCalc
        self.startVelocity = rospy.get_param('~/startVelocity', [0.2,0.0,0.0])
        self.actionValues = rospy.get_param('~/actionValues', [0.1,0.1,0.1,0.1])
        self.maxEpisodes = rospy.get_param('~/maxEpisodes', 5)
        self.newVelo = Twist()
        self.velo_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.move_sub = rospy.Subscriber('/input/cmd_vel', Twist, self.callbackMove)
    
    def callbackMove(self, Twist):
        self.newVelo = Twist

    def getSpaceState(self,scan):
        stateSpace = np.zeros((1,scan.shape[0]))
        tmp = np.copy(scan)
        stateSpace[0] = tmp
        return stateSpace

    def addNewState(state, stateSpace,Q):
        stateSpace=np.append(stateSpace,[state], axis=0) #Grow State Space
        Q=np.append(Q,[np.zeros(Q.shape[1])], axis=0) #Grow QTable
        return stateSpace, Q

    def getState(self,state, stateSpace, Q):
        x = 0
        for i in stateSpace:
            if((state==i).all()):
                return stateSpace[x],x, stateSpace, Q
            x=x+1
        stateSpace, Q = addNewState(state,stateSpace, Q)
        return state, stateSpace.shape[0]-1, stateSpace, Q


    def get_distance(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        self.scan = np.array(self.msg.ranges)
        self.actions = np.array(['Stop', 'SpeedX', 'SlowX', 'SpeedY', 'SlowY']) #See above
        self.stateSpace = self.getSpaceState(self.scan)
        self.QTable = np.zeros((self.stateSpace.shape[0], self.actions.shape[0]))
        self.rewards = np.array([-100,10,-5,-30,+100]) # See above

    def checkDistance(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        self.scan = np.array(self.msg.ranges)
        print(self.scan)
        return np.amin(self.scan)

    
    def makeAction(self, action, actionValues):
        reward = 0
        if(action=='Stop'):
            self.newVelo.linear.x = 0
            self.newVelo.linear.y = 0
            self.newVelo.linear.z = 0
            reward = reward + self.rewards[3] #Reward for Stop
        elif(action == 'SpeedX'):    
            self.newVelo.linear.x = self.newVelo.linear.x + actionValues[0]
            reward = reward + self.rewards[1] #Reward for Speed
        elif(action == 'SlowX'):
            self.newVelo.linear.x = self.newVelo.linear.x - actionValues[1]
            reward = reward + self.rewards[2] #Reward for Slow
        elif(action == 'SpeedY'):
            self.newVelo.linear.y = self.newVelo.linear.y + actionValues[2]
            reward = reward + self.rewards[1] #Reward for Speed
        elif(action == 'SlowY'):
            self.newVelo.linear.y = self.newVelo.linear.y + actionValues[3]
            reward = reward + self.rewards[2] #Reward for Slow
        self.velo_move.publish(self.newVelo)  #Publish
        minDist = self.checkDistance()
        if(minDist <= 0):
            reward = reward + self.rewards[0] #RewardForHitWall
        return reward, False

    def learn(self):
        steps = 0
        currentState = self.stateSpace[0]
        done = False
        while not done:
            done = False
            if random.uniform(0,1)<self.epsilon:
                actionId = random.randrange(self.actions.shape[0]) 
                chooseAction = self.actions[actionId]       
            else:
                currentState,stateId, self.stateSpace, self.QTable=self.getState(currentState, self.stateSpace,self.QTable)
                actionId = np.argmax(self.QTable[stateId, :])
                chooseAction = self.actions[actionId]  
            oldState=np.copy(currentState)
            myReward, done = self.makeAction(chooseAction,self.actionValues)
            currentState = self.scan
            currentState,stateId, self.stateSpace, self.QTable=self.getState(currentState, self.stateSpace, self.QTable)
            oldState,oldStateId,self.stateSpace, self.QTable=self.getState(oldState, self.stateSpace, self.QTable)
            self.QTable[oldStateId, actionId] = self.QTable[oldStateId, actionId] + self.learnRate * (myReward+self.gamma*np.max(self.QTable[stateId, :]) - self.QTable[oldStateId,actionId])
            steps = steps+1
            #print("Steps", steps)



    def run(self, rate: float = 30):
        print("Start")
        self.get_distance()
        while not rospy.is_shutdown():
            self.learn()

if __name__ == '__main__':
    rospy.init_node('learnToMove')
    learnToMove = learnToMove()
    learnToMove.run(rate=30)