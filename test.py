#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist

class GoToPose():
    def __init__(self):
        #publish advertizement before init_node()
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        rospy.init_node('test', anonymous=True)
        initial_pose = PoseWithCovarianceStamped() 
        initial_pose.pose.pose = Pose(Point(1.6,2.7, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = rospy.Time.now()
        print initial_pose.pose.pose.position.x
        while True:
            rospy.sleep(1);       
            self.pub.publish(initial_pose);


if __name__ == '__main__':
    try:
        GoToPose()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")

