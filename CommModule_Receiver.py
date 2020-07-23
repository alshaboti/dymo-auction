'''
 Author: M. Al-Shaboti
 This module for handling multicast communication betweeen nodes.
'''
import time,threading

import lcm
from commsg import comm_msg_t

robotName = "turtlebot1"

class Receiver (threading.Thread):

    def __init__(self, name, lock):
        threading.Thread.__init__(self)
        self.name = name
        self.threadLock = lock

        self.isReceived = False;
        try:
            self.listener = lcm.LCM()
            self.subscription = self.listener.subscribe("EXAMPLE", self.msgReceive)             
        except Exception, e:
            print(e)

    def run(self):
        print "Starting " + self.name
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
        lc.publish("EXAMPLE", msg.encode())

if __name__ == '__main__':
    try:        
        threadLock = threading.Lock()
        # Create new receiver thread
        receiver = Receiver("Receiver-Thread",threadLock)
        receiver.start()

        while True:
            time.sleep(1);
            print "main thread still alive"
            threadLock.acquire() 

            if receiver.isReceived:
                receiver.isReceived = False
                if receiver.msg.receiver == robotName:
                    print " I am the receiver " + receiver.msg.receiver
                    msg = comm_msg_t()
                    msg.sender = robotName
                    msg.receiver = receiver.msg.sender
                    msg.position = (1, 2)
                    Sender(msg)


            threadLock.release()
        # wait for all child threads to die    
        receiver.join()
    except:
        print("Receiver node terminated.")
