#!/usr/bin/env python
'''
 Author: M. Al-shaboti
 Description: This is auction-based task allocation approach experiment using Turtlebot2
'''
import rospy
from move_base_msgs.msg import actionlib, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist


from CommModule import Sender, Receiver
from commsg import comm_msg_t

import time, threading, random, math

Kd = 0.7
Kq = 0.3

MAX_TRAVEL_DIST = 10 # meter
MAX_RESOURCES = 60 # units


threadLock = threading.Lock()
class TurtlebotNode (threading.Thread):

    def __init__(self, name, lock, x, y, q):
        threading.Thread.__init__(self)
        #robots name
        self.name = name
        self.threadLock = lock
        # if in auctioneer case, Dictionary for all robots and their bids
        self.bidsDic = {}             
        self.auctionEndTime = 0
        # if in biders case
        self.currTaskIndex = -1
        self.pendingTaskList = []
        #robot information 
        self.Resouces = MAX_RESOURCES
        self.bidR = self.Resouces 
        self.Energy = 0
        self.bidE = self.Energy
        self.x = x
        self.y = y
        self.Quality = q

    def run(self):
        try:
            #start ROS
            rospy.init_node(self.name, anonymous=False)
            #what to do if shut down (e.g. ctrl + C or failure)
            rospy.on_shutdown(self.shutdown)    
            # Publisher to manually control the robot (e.g. to stop it)
            self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
            #tell the action client that we want to spin a thread by default
            self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            rospy.loginfo("wait for the action server to come up")
            #allow up to 5 seconds for the action server to come up
            self.move_base.wait_for_server(rospy.Duration(5))
            rospy.loginfo("Connected to move base server")        
            # A variable to hold the initial pose of the robot to be set by 
            # the user in RViz        
            initial_pose = PoseWithCovarianceStamped()
            # Get the initial pose from the user
            rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
            rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
            self.last_location = Pose()
            rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)  
            # wiat for initial pose
            while initial_pose.header.stamp == "":
                rospy.sleep(1)
            
            rospy.loginfo("Starting navigation test")              
            #end ROS

            # Create new receiver thread
            self.receiver = Receiver(self.name, threadLock)
            self.receiver.start()        

            while not rospy.is_shutdown():
                self.checkReceive()

                if self.auctionEndTime != 0 and self.auctionEndTime < current_sec_time():
                    self.auctionEndTime = 0
                    winner = max(self.bidsDic, key=self.bidsDic.get);
                    if winner == self.name:
                        self.winTask(self.auctionMsg)
                    else:
                        self.winMsg = comm_msg_t()
                        self.winMsg.receiver = winner
                        self.sendMsg("winMsg")
                    print self.bidsDic


        except Exception, e:
            print(self.name, " node terminated.", e)


    def checkReceive(self):
        threadLock.acquire() 
        if self.receiver.isReceived:
            self.receiver.isReceived = False   
            print self.name," received ", self.receiver.msg.msg_type  

            if self.receiver.msg.msg_type == "TaskAnn":
                self.auctionMsg = self.receiver.msg

                self.auctionEndTime = current_sec_time() + 3 # 3 seconds
                mybid = self.computeMyBid()
                
                if mybid != 1:
                    self.bidsDic[self.name] = mybid 
                self.sendMsg("auctionMsg")

            elif self.receiver.msg.msg_type == "auctionMsg":      
                self.auctionMsg = self.receiver.msg
                mybid = self.computeMyBid()
                
                if mybid != 1:     
                    self.bidsMsg = self.receiver.msg
                    self.bidsMsg.bids = mybid;
                    self.sendMsg("bidsMsg")

            elif self.receiver.msg.msg_type == "bidsMsg":  
                self.bidsDic[self.receiver.msg.sender] = self.receiver.msg.bids
            elif self.receiver.msg.msg_type == "winMsg":
                self.winTask(self.bidsMsg)

        threadLock.release()
    def sendMsg(self, msgtype):

        if msgtype == "auctionMsg":
            self.auctionMsg.receiver = "multicast"
            self.auctionMsg.sender = self.name
            self.auctionMsg.msg_type = msgtype;            
            Sender(self.auctionMsg)

        elif msgtype == "bidsMsg":
            self.bidsMsg.receiver = self.bidsMsg.sender
            self.bidsMsg.sender = self.name            
            self.bidsMsg.msg_type = msgtype;
            Sender(self.bidsMsg)
        elif msgtype == "winMsg":
            self.winMsg.sender = self.name
            self.winMsg.msg_type = msgtype;
            Sender(self.winMsg);

    def computeMyBid(self):
        if (self.bidR - self.auctionMsg.r >= 0):
            #Get the minimum distance to the biding task 
            minD = math.sqrt( (self.y - self.auctionMsg.position[1])**2 + (self.x - self.auctionMsg.position[0])**2 )
            for i in range(0,len(self.pendingTaskList)):
                x = self.pendingTaskList[i].x
                y = self.pendingTaskList[i].y
                temp = math.sqrt( (self.y - y)**2 + (self.x - x)**2 )
                if temp < minD:
                    minD = temp
            normD = float(minD)/float(MAX_TRAVEL_DIST);
            normQ = abs(self.Quality - self.auctionMsg.q)/8.0
            mybid = - Kd *normD - Kq * normQ
            print self.name, " bids is ", mybid

            return mybid

        return 1;
    def winTask(self, winMsg):
        print self.name , " is the winner!"
        newTask = self.TaskInfo(winMsg.position[0],winMsg.position[1],winMsg.r, \
            winMsg.q,winMsg.exec_time,winMsg.seq_no)
        self.pendingTaskList.append(newTask)
        print "Task saved !", newTask

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    class TaskInfo(object):
        """For save all tasks"""
        def __init__(self, x, y, r, q, exeTime,seqNo, isDone=0):            
            self.x = x
            self.y = y
            self.r = r
            self.q = q
            self.exeTime = exeTime
            self.seqNo = seqNo
            self.isDone = isDone 
        def __str__(self):
            return str(self.x) + ", " + str(self.y) + ", " + str(self.r) + ", " + str(self.q) + \
            ", " + str(self.exeTime) + ", " + str(self.seqNo) + ", " + str(self.isDone)



current_milli_time = lambda: int(round(time.time() * 1000))
current_sec_time = lambda: int(round(time.time()))

if __name__ == '__main__':
    try:
        #name,lock,x,y,q
        turtlebot1 = TurtlebotNode("turtlebot1",threadLock,0,0,5)
        turtlebot1.start();
        turtlebot1.join();
    except Exception, e:
        print "Turtlebot has been terminated " ,e