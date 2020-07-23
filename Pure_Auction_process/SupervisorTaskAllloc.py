'''
 Author: M. Al-shaboti
 Description: This is auction-based task allocation approach experiment using Turtlebot2 
'''
# TDone, TaskAnn
from CommModule import Sender, Receiver
from utility import Randomize, Point, current_sec_time, current_milli_time

import time,threading
from commsg import comm_msg_t

#define global experiment variables, like noTasks, exeMean, taskLocationList, etc
noTasks = 1;
exeMean = 20;
minExec = 10; # to not have 0 sec exeTime 
interArrMean = 1;
minInterArr = 5; # to give a time for robots to assigne the previous task
taskSeqNo = 0
# To sync access shared resources
threadLock = threading.Lock()

class Supervisor (threading.Thread):

    def __init__(self, name, lock):
        threading.Thread.__init__(self)
        self.name = name
        self.threadLock = lock
        self.generateAllRands()
        self.pointList = [Point(1.07,2.92),Point(0.58,4.13),Point(1.89,4.43),Point(2.16,3.55),Point(1.5,3.6)]

    def run(self):
     try:                
        # Create new receiver thread
        self.receiver = Receiver(self.name, threadLock)
        self.receiver.start()
        lastTime = current_sec_time();
        taskSeqNo = 0

        while True:
            # if yes, then announce next task
            if (taskSeqNo < noTasks) and (current_sec_time() > self.interAvlTimeList[taskSeqNo] + lastTime): 
                taskAnnmsg = comm_msg_t()
                taskAnnmsg.sender = self.name
                taskAnnmsg.receiver = "turtlebot1"
                taskAnnmsg.seq_no = taskSeqNo;
                taskAnnmsg.msg_type = "TaskAnn"
                taskAnnmsg.r = self.ResourceList[taskSeqNo]
                taskAnnmsg.q = self.qualityList[taskSeqNo]
                taskAnnmsg.exec_time = self.exeTimeList[taskSeqNo]

                taskAnnmsg.position = (self.pointList[taskSeqNo].x, self.pointList[taskSeqNo].y)               
                Sender(taskAnnmsg)
                taskSeqNo +=1
                lastTime = current_sec_time()


            #print current_sec_time(), self.interAvlTimeList[taskSeqNo-1] + lastTime
            self.checkReceive();

        self.receiver.join();    
     except Exception, e:
        print("Supervisor node terminated.", e)

    def checkReceive(self):
        threadLock.acquire() 

        if self.receiver.isReceived:
            self.receiver.isReceived = False
            if self.receiver.msg.msg_type == "TDone":
                print "Robot ", self.receiver.msg.sender, " has performed task ", self.receiver.msg.taskSeqNo
            
            
        threadLock.release()

        
    def generateAllRands(self):
        self.ResourceList = Randomize().getUniformList(1,11,noTasks)
        self.qualityList = Randomize().getUniformList(1,10,noTasks)
        self.exeTimeList = Randomize().getExponList(exeMean,noTasks);
        self.exeTimeList = [x+minExec for x in self.exeTimeList]
        self.interAvlTimeList = Randomize().getExponList(interArrMean,noTasks);
        self.interAvlTimeList = [x+minInterArr for x in self.interAvlTimeList]
        #static from map 
        self.pointList = [Point(1,2), Point(3,3)]
        
 
if __name__ == '__main__':
    try:
        threadLock = threading.Lock()
        supervisor = Supervisor("supervisor",threadLock)
        supervisor.start();
        supervisor.join();

    except Exception, e:
        print "Supervisor has been terminated " + e