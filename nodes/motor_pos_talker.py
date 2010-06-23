#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
from ros_flydra.msg import *
from geometry_msgs.msg import Point
import time
import numpy as np
pi = np.pi

def talker():
    pub = rospy.Publisher('ptf_3d', Point)
    rospy.init_node('talker')
    start_time = time.time()
    
    # random start position values
    x0 = np.random.randn()
    y0 = np.random.randn()
    z0 = np.random.randn()

    # random amplitude values
    xamp = np.random.randn()
    yamp = np.random.randn()
    zamp = np.random.randn()
        
    while not rospy.is_shutdown():       
        theta = time.time() % (2*pi)
        x = xamp*np.cos( theta ) + x0
        y = yamp*np.sin( theta ) + y0
        z = zamp*np.sin( theta/2.3 ) + z0
        msg = Point(x,y,z)
        pub.publish(msg)
        rospy.sleep(0.005)
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

