'''
 Author: M. Al-Shaboti
 This module for handling multicast communication betweeen nodes.
'''
import lcm,time,threading
from commsg import comm_msg_t


class Receiver (threading.Thread):

    def __init__(self, name, lock):
        threading.Thread.__init__(self)
        self.name = name
        self.threadLock = lock

        self.isReceived = False;
        try:
            self.listener = lcm.LCM()
            self.subscription = self.listener.subscribe("TATopic", self.msgReceive)             
        except Exception, e:
            print(e)

    def run(self):
        print "Starting receiver thread of " + self.name
        try:
            while True:
                self.listener.handle() 

        except Exception,e:
            print(e)
        self.listener.unsubscribe(self.subscription)

    def msgReceive(self,channel, data):
        
        # Get lock to synchronize threads, block any other thread from using shared res, untile I finish
        self.threadLock.acquire() 
        #accessing shared resources
        try:
            self.msg = comm_msg_t.decode(data)
            if (self.msg.receiver == self.name or self.msg.receiver == "multicast") and (self.msg.sender != self.name or self.msg.msg_type =="winMsg"):
                self.isReceived = True;
        except Exception, e:
            pass    
        
        # Free, shared res, lock to release next thread
        self.threadLock.release()      

# Simple class for sending message
class Sender(object):
    """docstring for Sender"""
    def __init__(self, msg):
        super(Sender, self).__init__()        
        lc = lcm.LCM()
        lc.publish("TATopic", msg.encode())      


if __name__ == '__main__':

    robotName = "turtlebot2"
    try:        
        threadLock = threading.Lock()
        # Create new receiver thread
        receiver = Receiver("turtlebot1", threadLock)
        receiver.start()
        counter = 1;

        while True:
            time.sleep(1);
            print "main thread still alive"
            threadLock.acquire() 
            if receiver.isReceived:
                receiver.isReceived = False
                if receiver.msg.receiver == robotName:
                   print " I am the receiver " + receiver.msg.receiver

            threadLock.release()                

            if counter > 0:
                msg = comm_msg_t()
                msg.sender = robotName
                msg.receiver = "supervisor" #turtlebot1"
                msg.position = (1, 2, 3)
               
                Sender(msg)
                counter-=1        

        # wait for all child threads to die    
        receiver.join()        
    except:
        print("Receiver node terminated.")
