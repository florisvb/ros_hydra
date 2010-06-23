#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_hydra')
import rospy
from std_msgs.msg import *
from ros_flydra.msg import *
import time

import sys
sys.path.append("/home/floris/src/floris")

from optparse import OptionParser

class MotorCom:
    
    def __init__(self, motor):
    
        self.motor = motor
    
        ################################################
        # ROS stuff
        
        # ros subscribers
        sub = motor + '_ctrl'
        rospy.Subscriber(sub, Float64, self.ctrl_callback)

        # ros publishers
        pub = motor + '_pos'
        self.pub = rospy.Publisher(pub, Float64)

        node = motor + '_atmel_com'
        rospy.init_node(node, anonymous=True)
        #################################################
        
        #################################################
        # init motor
        if motor == 'pan':
            sernum='0.0.A'
            gr=7.2
            clkdir_mult=1
            ind_per_rev=12800
            vel_max=30000
            vel_min=0
        if motor == 'tilt':
            sernum='0.0.B'
            gr=7.2
            clkdir_mult=1
            ind_per_rev=12800
            vel_max=30000
            vel_min=0
        if motor == 'focus':
            sernum='0.0.C'
            gr=2.6
            clkdir_mult=1
            ind_per_rev=12800
            vel_max=30000
            vel_min=0
        self.m = stepper_motors(sernum=sernum, gr=gr, clkdir_mult=clkdir_mult, ind_per_rev=ind_per_rev, vel_max=vel_max, vel_min=vel_min)
        #################################################
        
        # latest motor states  
        self.pos = 0
        self.vel = 0
        
        # motor control characteristics 
        if self.motor == 'pan':
            self.damping = 0.02
            self.gain = 15
        if self.motor == 'tilt':
            self.damping = 0.02
            self.gain = 15
        if self.motor == 'focus':
            self.damping = 0.02
            self.gain = 15
        
        self.direction = 0

    def ctrl_callback(self, data):
        m_des_pos = data.data
        m_current_pos = self.m.getpos()
        self.pub.publish(Float64(m_current_pos))
        
        # controller:
        vel_des = self.gain*(m_des_pos-m_current_pos) 
                
        # proposed acceleration:
        accel = (vel_des - self.vel)
        
        # set new desired velocity
        self.vel = self.vel + (vel_des - self.vel)*np.exp(-1*np.abs(accel)*self.damping)
        
        change_direction = 1
        if np.sign(self.vel) == self.direction:
            change_direction = 0
        self.direction = np.sign(self.vel)
    
        #print motorid, m_control, m_current_vel, vel_des, accel, 'latency: ', latency
        self.m.setvel(self.vel, change_direction = change_direction)


if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--motor", type="str", dest="motor", default='pan',
                        help="motor identifier, ie. pan, tilt, or focus")
    
    motorcom = MotorCom(options.motor)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
