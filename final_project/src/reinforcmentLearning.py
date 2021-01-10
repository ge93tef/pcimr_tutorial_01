#!/usr/bin/env python

import rospy
import rosservice
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pcimr_simulation.srv import InitPos
from geometry_msgs.msg import Twist
import random
from std_srvs.srv import Empty
import numpy as np

#Actions (To be Change?)
#0: Stay
#1: Speed X
#2: Slow X
#3: Speed Y
#4: Slow Y
#5: Stop

#Rewards:
#0: -100 Hit Wall
#1: +10 Get Faster
#2: -5 Get Slower
#3: -30 Stop
#4: +100 Finish Round

class learnToMove:

    def __init__(self):
        self.learnRate = rospy.get_param('~/learnRate', 0.85) #LearnRate
        self.epsilon = rospy.get_param('~/epsilon', 0.2) #Randomness
        self.gamma = rospy.get_param('~/gamma', 0.9) #forQCalc
        self.startVelocity = rospy.get_param('~/startVelocity', [0.1,0.0,0.0])
        self.actionValues = rospy.get_param('~/actionValues', [0.1,0.25,0.25,0.25])
        self.maxEpisodes = rospy.get_param('~/maxEpisodes', 5)
        self.amountBlocks = rospy.get_param('~/amountBlocks', 5)
        self.newVelo = Twist()
        self.velo_move = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.move_sub = rospy.Subscriber('/input/cmd_vel', Twist, self.callbackMove)
    
    def callbackMove(self, Twist):
        self.newVelo = Twist

    def getSpaceState(self,scan):
        stateSpace = np.zeros((1,self.amountBlocks+1))
        Q=np.zeros((stateSpace.shape[0], self.actions.shape[0]))
        state, stateSpace, Q = self.addNewState(scan, stateSpace, Q)
        return stateSpace

    def transformState(self,state):
        split=state.shape[0]//self.amountBlocks
        newState=np.zeros(self.amountBlocks+1)
        for i in range(0,self.amountBlocks-1):
            block = state[(i*split):(split*(i+1))]
            newState[i]=np.amin(block)
        newState[self.amountBlocks-1] = np.amin(state[(i+1)*split:])
        newState[-1]=self.newVelo.linear.x
        return newState

    def addNewState(self,state, stateSpace,Q):
        newState=self.transformState(state)
        stateSpace=np.append(stateSpace,[newState], axis=0) #Grow State Space
        Q=np.append(Q,[np.zeros(Q.shape[1])], axis=0) #Grow QTable
        return newState, stateSpace, Q

    def getState(self,state, stateSpace, Q):
        x = 0
        if(state.shape[0] != self.amountBlocks):
            state=self.transformState(state)
        for i in stateSpace:
            if((state==i).all()):
                return stateSpace[x],x, stateSpace, Q
            x=x+1
        state, stateSpace, Q = self.addNewState(state,stateSpace, Q)
        return state, stateSpace.shape[0]-1, stateSpace, Q


    def get_distance(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        self.scan = np.round(np.array(self.msg.ranges), 1)
        self.actions = np.array(['Forward','SpeedX','MoveLeft', 'MoveRight']) #See above
        self.stateSpace = self.getSpaceState(self.scan)
        self.QTable = np.zeros((2, self.actions.shape[0] ))
        self.rewards = np.array([-30,0.001,-5,-10,5, 0.02]) # See above
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_simulation()

    def checkDistance(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        self.scan = np.round(np.array(self.msg.ranges), 1)
        return np.amin(self.scan[61:184])

    
    def makeAction(self, action, actionValues):
        reward = 0
        #print(action)
        if(action == 'Forward'):
            self.newVelo.angular.z = 0.0
        elif(action == 'SpeedX'):    
            self.newVelo.linear.x = self.newVelo.linear.x + actionValues[0]
            if(self.newVelo.linear.x  > 0.5):
                self.newVelo.linear.x = 0.5
            else:
                reward = reward + self.rewards[1] #Reward for Speed
        elif(action == 'SlowX'):
            self.newVelo.linear.x = self.newVelo.linear.x - actionValues[1]
            if(self.newVelo.linear.x <= 0):
                self.newVelo.linear.x = 0
                reward = reward + self.rewards[3]
        elif(action == 'MoveLeft'):
            self.newVelo.angular.z= -0.40
            self.newVelo.linear.x = 0
            #reward = reward + self.rewards[1] #Reward for Speed
        elif(action == 'MoveRight'):
            #reward = reward + self.rewards[2] #Reward for Slow
            self.newVelo.angular.z = 0.40
            self.newVelo.linear.x = 0
        if(self.newVelo.linear.x > 0):
            reward = reward + self.rewards[1] #Reward for Speed
        self.velo_move.publish(self.newVelo)  #Publish
        minDist = self.checkDistance()
        if(minDist <= 0.45):
            reward = reward + self.rewards[0] #RewardForHitWall
        if(minDist > 0.45):
            reward = reward + self.rewards[4]
        if(minDist <= 0.14):
            reward = reward + 80*(self.rewards[0])
        if(self.scan[120]>0.7):
            reward = reward + 0.1
        return reward, False

    def learn(self):
        steps = 0
        episode = 0
        overallReward = 0
        currentState = self.stateSpace[1]
        done = False
        print("Episode", episode)
        while not done:
            done = False
            if(overallReward < -10000):
                episode = episode +1
                print("Episode", episode)
                print('Steps', steps)
                self.reset_simulation()
                overallReward = 0
                currentState = self.stateSpace[1]
                self.newVelo.linear.x = 0
                self.newVelo.angular.z = 0
                steps = 0
            if random.uniform(0,1)<self.epsilon:
                actionId = random.randrange(self.actions.shape[0]) 
                chooseAction = self.actions[actionId]       
            else:
                currentState,stateId, self.stateSpace, self.QTable=self.getState(currentState, self.stateSpace,self.QTable)
                actionId = np.argmax(self.QTable[stateId, :])
                chooseAction = self.actions[actionId]  
            oldState=np.copy(currentState)
            myReward, done = self.makeAction(chooseAction,self.actionValues)
            #myReward = myReward + steps * self.rewards[5] ##Longer run = more points
            currentState,stateId, self.stateSpace, self.QTable=self.getState(self.scan , self.stateSpace, self.QTable)
            oldState,oldStateId,self.stateSpace, self.QTable=self.getState(oldState, self.stateSpace, self.QTable)
            self.QTable[oldStateId, actionId] = self.QTable[oldStateId, actionId] + self.learnRate * (myReward+self.gamma*np.max(self.QTable[stateId, :]) - self.QTable[oldStateId,actionId])
            steps = steps+1
            overallReward = overallReward + myReward



    def run(self, rate: float = 30):
        print("Start")
        self.get_distance()
        while not rospy.is_shutdown():
            self.learn()

if __name__ == '__main__':
    rospy.init_node('learnToMove')
    learnToMove = learnToMove()
    learnToMove.run(rate=30)