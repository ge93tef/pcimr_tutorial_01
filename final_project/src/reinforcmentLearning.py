#!/usr/bin/env python

import rospy
import rosservice
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pcimr_simulation.srv import InitPos
from geometry_msgs.msg import Twist,Pose2D
import random
from stdr_msgs.srv import MoveRobot
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
        self.learnRate = rospy.get_param('~/learnRate', 0.2) #LearnRate
        self.epsilon = rospy.get_param('~/epsilon', 0.9) #Randomness
        self.gamma = rospy.get_param('~/gamma', 0.9) #forQCalc
        self.actionValues = rospy.get_param('~/actionValues', [0.2,0.3,0.4, 0.05]) #Speed0,Speed1,Speed2,SpeedTurn
        self.maxEpisodes = rospy.get_param('~/maxEpisodes', 5)
        self.amountBlocks = rospy.get_param('~/amountBlocks', 7)
        self.newVelo = Twist()
        self.velo_move = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=100)
        self.move_sub = rospy.Subscriber('/robot0/cmd_vel', Twist, self.callbackMove)
        self.lastActionZ = np.array([0])#Z
        self.lastScan = np.zeros((258))
        self.episode =0
        self.load = True
    
    def callbackMove(self, Twist):
        self.newVelo = Twist

    def getSpaceState(self,scan):
        stateSpace = np.zeros((1,self.amountBlocks))
        #stateSpace = np.zeros((1,self.scan.shape[0]))
        Q=np.zeros((stateSpace.shape[0], self.actions.shape[0]))
        state, stateSpace, Q = self.addNewState(scan, stateSpace, Q)
        return stateSpace

    def transformState(self,state):
        #split=state.shape[0]//self.amountBlocks
       # 
       # for i in range(0,self.amountBlocks-1):
        #    amountHalf = (split*(i+1)-(i*split))//2
       #    newState[i]  = state[(i*split)+amountHalf]
       # amountHalf = (state.shape[0]-((i+1)*split))//2
       # newState[self.amountBlocks-1] = state[(i+1)*split+amountHalf]
        #bins = np.array([0.0,0.2,0.4,0.6,0.8,1.0,1.2,1.4,1.6,1.8,2.0,2.2,2.4,2.6,2.8,3.0,3.2,3.4,3.6,3.8,4.0,4.2,4.4,4.6,4.8,5.0,5.2,5.4,5.6])
        bins = np.arange(0.0,5.8,0.2)
        if(state.shape[0]==7):
            return state
        newState=np.zeros(7)
        newState[0] = state[0]
        newState[1] = state[15]
        newState[2] = state[65]
        newState[3] = state[129]
        newState[4] = state[179]
        newState[5] = state[229]
        newState[6] = state[244]

        binnedState = np.digitize(newState,bins, right=False)
        count = 0
        for i in binnedState:
            newState[count] = bins[i-1]
            count = count +1
        return newState

    def addNewState(self,state, stateSpace,Q):
        newState=self.transformState(state)
        stateSpace=np.round(np.append(stateSpace,[newState], axis=0),1) #Grow State Space
        Q=np.append(Q,[np.zeros(Q.shape[1])], axis=0) #Grow QTable
        return newState, stateSpace, Q

    def getState(self,state, stateSpace, Q):
        x = 0
        if(state.shape[0] != self.amountBlocks):      
            state=self.transformState(state)
        state = np.round(state,1)
        for i in stateSpace:
            if((state==i).all()):
                return stateSpace[x],x, stateSpace, Q
            x=x+1
        state, stateSpace, Q = self.addNewState(state,stateSpace, Q)
        return state, stateSpace.shape[0]-1, stateSpace, Q


    def get_distance(self):
        self.msg = rospy.wait_for_message("robot0/laser_0", LaserScan)
        self.scan = np.round(np.array(self.msg.ranges), 1)
        self.actions = np.array(['Speed0','LeftTurn', 'RightTurn']) #See above
        if(self.load == False):
            self.stateSpace = self.getSpaceState(self.scan)
            self.QTable = np.zeros((self.stateSpace.shape[0], self.actions.shape[0] ))
            print(self.QTable.shape)
            print(self.stateSpace.shape)
        else:
            self.stateSpace = np.loadtxt('StateSpace_Disc7.out', delimiter=';')
            self.QTable = np.loadtxt('QTable_Disc7.out', delimiter=';')
            print(self.QTable[2])
        self.rewards = np.array([0.3,0.1,0.4,0.8, 0.99, 0.2, 200]) # Drive to Wall, DiffGotBetter, Get Away from Wall, Driving, 90DegDiff, DistanceFrontGrew, Lost
        #self.reset_simulation = rospy.ServiceProxy('/stdr/reset_simulation', Empty)
        newPose = Pose2D()
        newPose.x = 0
        newPose.y = 0
        newPose.theta = 0
        self.reset_simulation = rospy.ServiceProxy('robot0/replace', MoveRobot)
        #self.reset_simulation(newPose)

    def checkDistance(self):
        self.msg = rospy.wait_for_message("robot0/laser_0", LaserScan)
        self.scan = np.round(np.array(self.msg.ranges), 1)
        

    
    def makeAction(self, action, actionValues):
        reward = 0
        lost = False
        #print(action)
        if(action == 'Speed0'):
            self.newVelo.angular.z = 0.0
            self.newVelo.linear.x = actionValues[0]
            reward = reward + 5
        elif(action == 'LeftTurn'):
            self.newVelo.angular.z= 0.20
            reward = reward + 1
            self.newVelo.linear.x = actionValues[3]
        elif(action == 'RightTurn'):
            self.newVelo.angular.z = -0.20
            reward = reward + 1
            self.newVelo.linear.x = actionValues[3]
        self.lastScan = np.copy(self.scan)
        self.velo_move.publish(self.newVelo)  #Publish
        self.checkDistance() #checkDistanceForRewards
        left = self.scan[15]
        right = self.scan[229]
        if(left > 5):
            left = 5
        if(right > 5):
            right = 5
        if(np.amin(self.scan)<=0.20):
            print('Lost Game: Collision')
            reward = -200
            lost=True
        else:     
            reward = reward + (0.3 - np.abs(left-right))*50
           # if(self.newVelo.linear.x>0.05):
               # reward = reward + 5
            #if(np.amin(self.scan)<=0.50):
                    #reward = reward - 5.5
        return reward, lost

    def learn(self):
        self.steps = 0
        overallReward = 0
        currentState = self.stateSpace[1]
        done = False
        lost = False
        print("Episode", self.episode)
        print("Epsilon", self.epsilon)
        while not done:
            #r = rospy.Rate(3000)
            done = False
            
            oldState,oldStateId, self.stateSpace, Q=self.getState(currentState, self.stateSpace,self.QTable)
            
            if(lost):
                self.episode = self.episode +1
                print("Episode", self.episode)
                print('Steps', self.steps)
                if(self.episode %1 == 0):
                    print("Table", self.QTable.shape)
                    print("State", self.stateSpace.shape)
                    title = 'QTable_Disc7'+'.out'
                    np.savetxt(title, self.QTable, delimiter=';',fmt='%.8f')   # X is an array
                    title = 'StateSpace_Disc7'+'.out'
                    np.savetxt(title, self.stateSpace, delimiter=';',fmt='%.1f')   # X is an array
                #self.reset_simulation() 
                currentState = self.stateSpace[1]
                self.newVelo.linear.x = 0
                self.newVelo.angular.z = 0
                lost=False
                self.epsilon = self.epsilon*0.9987
                if(self.epsilon < 0.05):
                    self.epsilon = 0.05
                #if(self.episode >= 5000):
                    #print('Test', overallReward)
                    self.epsilon = 0.0
                    #self.learnRate = 0.0
                overallReward = 0
                print("Epsilon", self.epsilon)
                self.steps = 0
            if random.uniform(0,1)<self.epsilon:
                actionId = random.randrange(self.actions.shape[0]) 
                chooseAction = self.actions[actionId]      
            else:
                currentState,stateId, self.stateSpace, self.QTable=self.getState(currentState, self.stateSpace,self.QTable)
                if(np.count_nonzero(currentState) == 0):
                    actionId = random.randrange(self.actions.shape[0])
                else:
                    actionId = np.argmax(self.QTable[stateId, :])
                chooseAction = self.actions[actionId]  
            myReward, lost = self.makeAction(chooseAction,self.actionValues)
            currentState,stateId, self.stateSpace, self.QTable=self.getState(self.scan , self.stateSpace, self.QTable)
            self.QTable[oldStateId, actionId] = self.QTable[oldStateId, actionId] + self.learnRate * (myReward+self.gamma*np.max(self.QTable[stateId, :]) - self.QTable[oldStateId,actionId])
            self.steps = self.steps+1
            overallReward = overallReward + myReward
            #if(self.steps>1500):
                #lost = = True



    def run(self, rate: float = 30000):
        print("Start")
        r = rospy.Rate(rate)
        self.get_distance()
        self.learn()
        while not rospy.is_shutdown():
            print("Start")

if __name__ == '__main__':
    rospy.init_node('learnToMove')
    learnToMove = learnToMove()
    learnToMove.run(rate=1000)