'''
 Author: M. Al-Shaboti
 This is a utility module 
'''
import random, time


class Randomize(object):
    """docstring for Randomize"""
    def __init__(self):
        super(Randomize, self).__init__()
    def getUniformList(self, a,b,size):
        uniformList = [0] * size 
        counter = 0
        while counter < size:
            uniformList[counter] = int(random.uniform(a, b))
            counter +=1
        return uniformList

    def getExponList(self, mu,size):
        exopnlist = [0] * size        
        counter = 0
        while counter < size:
            exopnlist[counter] = int(random.expovariate(1/float(mu)));
            counter +=1
            #print exopnlist[counter-1]
        return exopnlist

class Point(object):
    """to descripe a 2d point x,y"""
    def __init__(self, x,y):
        super(Point, self).__init__()
        self.x = x
        self.y = y
    def __str__(self):
        return str(self.x) + "," + str(self.y)
        

current_milli_time = lambda: int(round(time.time() * 1000))
current_sec_time = lambda: int(round(time.time()))

#QualityList
if __name__ == '__main__':
    try:
        noTasks = 12;
        ResourceList = Randomize().getUniformList(1,11,noTasks)
        qualityList = Randomize().getUniformList(1,10,noTasks)
        exeTimeList = Randomize().getExponList(20,noTasks);
        interAvlTimeList = Randomize().getExponList(40,noTasks);
        #minimum execution time 10 seconds
        exeTimeList = [x+10 for x in exeTimeList]
        interAvlTimeList = [x+10 for x in interAvlTimeList]
        pointList = [Point(1,2), Point(23,3)]


        print ResourceList
        print qualityList
        print exeTimeList
        print interAvlTimeList
        print pointList[0]
    except Exception, e:
        print("Receiver node terminated.", e)

