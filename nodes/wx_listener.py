#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_hydra')
import rospy
from std_msgs.msg import *

class Listener:
    def __init__(self):
        
        rospy.Subscriber("wx_controller", String, self.callback)
        rospy.init_node('wx_controller_listener', anonymous=True)
        rospy.spin()

    def callback(self, data):
        print data.data
    

if __name__ == '__main__':
    listener = Listener()
