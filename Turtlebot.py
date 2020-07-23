#!/usr/bin/env python

#  http://ros-by-example.googlecode.com/svn/trunk/rbx_vol_1/rbx1_nav/nodes/nav_test.py
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

from CommModule import Sender, Receiver
from commsg import comm_msg_t

import time, threading, random, math

Kd = 0.7
Kq = 0.3

MAX_TRAVEL_DIST = 10 # meter
MAX_RESOURCES = 60 # units


class TurtlebotNode():
    def __init__(self, name, x, y, q, rcv):
        self.receiver = rcv
        #robots name
        self.name = name
        #self.threadLock = lock
        # if in auctioneer case, Dictionary for all robots and their bids
        self.bidsDic = {}             
        self.auctionEndTime = 0
        # if in biders case
        self.currTaskIndex = -1
        self.currTaskBeginTime = -1
        self.isCurrTaskTimeExec = False
        self.pendingTaskList = []
        #robot information 
        self.Resouces = MAX_RESOURCES
        self.bidR = self.Resouces 
        self.x = x
        self.y = y
        self.Quality = q    

        #statistics
        self.exeTask = 0
        self.totalTraveledDist = 0 
        self.lastExeTaskIndex = -1
        #Go to goal vaiables
        self.goalTimeout = 60 # 1 minute
        self.isInGotoGoal = False


        rospy.init_node(self.name, anonymous=True)        
        rospy.on_shutdown(self.shutdown)
        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)        
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)
        
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting 60s for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server... OK")
        
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()      
        
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('\initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)

        rospy.loginfo("initialpose has been received... ")    
        rospy.loginfo("Starting task allocation experiment")
        self.receiver.start()

        #----------------------------------------main loop----------------------------
        try:
            while not rospy.is_shutdown():
                rospy.sleep(1)
                self.checkReceive()

                #1 if auctioneer
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

                #2 Check for success or failure to to goal
                if self.isInGotoGoal:
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Goal succeeded!")
                        self.isInGotoGoal = False
                        self.isCurrTaskTimeExec = True
                        self.currTaskBeginTime = rospy.get_rostime().secs
                    else:
                        currTime = rospy.get_rostime().secs
                        if currTime >  (self.initialGoalTime + self.goalTimeout):
                            self.move_base.cancel_goal()
                            rospy.loginfo("Timed out achieving goal")
                            self.isInGotoGoal = False   

                # if in executing current task                 
                elif self.isCurrTaskTimeExec:
                    endTaskTime = self.currTaskBeginTime + self.pendingTaskList[self.currTaskIndex].exeTime
                    if  rospy.get_rostime().secs < endTaskTime:
                        self.lastExeTaskIndex = self.currTaskIndex
                        self.pendingTaskList[self.currTaskIndex].isDone = 1  
                        self.isCurrTaskTimeExec = False
                        self.exeTask += 1   
                        # send task done msg
                        taskDoneMsg = comm_msg_t()
                        taskDoneMsg.sender = self.name
                        taskDoneMsg.receiver = "supervisor"
                        taskDoneMsg.seq_no = self.pendingTaskList[self.currTaskIndex].seqNo;
                        taskDoneMsg.msg_type = "TDone"     
                        Sender(taskDoneMsg)                    

                #2 Execute next task from pendingTaskList
                elif len(self.pendingTaskList) > self.exeTask:
                    print "inside if before getNextTaskIndex"
                    self.currTaskIndex = self.getNextTaskIndex()
                    rospy.loginfo("currTaskIndex is" + str(self.currTaskIndex))
                    self.gotoGoal()

        except Exception, e:
            print(self.name, " node terminated. ", e)
        #---------------------------------end main loop-----------------------------------------------

    def getNextTaskIndex(self):
        nextTaskIndex = -1;
        if self.lastExeTaskIndex == -1:
            currX = self.x
            currY = self.y
        else:
            currX = self.pendingTaskList[self.lastExeTaskIndex].x
            currY = self.pendingTaskList[self.lastExeTaskIndex].y

        minD = 9999999        
        i = 0;
        while i < len(self.pendingTaskList):
            if self.pendingTaskList[i].isDone == 0:
                tempMin = math.sqrt( (currY - self.pendingTaskList[i].y)**2 + (currX - self.pendingTaskList[i].x)**2 ) 
                if tempMin < minD: 
                    minD = tempMin
                    nextTaskIndex = i;
            i+=1       
        return nextTaskIndex

    def gotoGoal(self):
        # Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = Pose(Point(self.pendingTaskList[self.currTaskIndex].x, \
           self.pendingTaskList[self.currTaskIndex].y, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        # Let the user know where the robot is going next
        rospy.loginfo("Going to: " + str(self.pendingTaskList[self.currTaskIndex]))
        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)
        self.initialGoalTime = rospy.get_rostime().secs
        self.isInGotoGoal = True
        # Allow 5 minutes to get there
#        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 


    def checkReceive(self):
        #threadLock.acquire() 
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
            minD = math.sqrt( (self.y - self.auctionMsg.position[1])**2 + \
             (self.x - self.auctionMsg.position[0])**2 )
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
        self.bidR -= winMsg.r
        print newTask
        print "Task has been added to the task pending list !"

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
        rospy.loginfo("update initial_pose")
        rospy.loginfo(self.initial_pose)

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
      
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        receiver = Receiver('TurtleRobot1',  threading.Lock())
        #receiver.start()  
        rurtleRobot1 = TurtlebotNode("TurtleRobot1",1.69,2.7,5, receiver)
        #rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
