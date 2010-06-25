#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_hydra')
import rospy
from std_msgs.msg import *
from ros_flydra.msg import *
import time

import numpy as np

import sys
sys.path.append("/home/floris/src/floris")


from optparse import OptionParser

class MotorCom:
    
    def __init__(self, motor, dummy=False):
    
        self.motor = motor
        self.dummy = dummy
        
        
        
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
        if self.dummy is False:
            import stepper_motors
            self.m = stepper_motors(sernum=sernum, gr=gr, clkdir_mult=clkdir_mult, ind_per_rev=ind_per_rev, vel_max=vel_max, vel_min=vel_min)
        #################################################
        
        # latest motor states  
        self.pos = 0
        self.vel = 0
        
        # motor control characteristics 
        if self.motor == 'pan':
            self.damping = 0.02
            self.gain = 15
            self.limit_lo = -1
            self.limit_hi = 1
        if self.motor == 'tilt':
            self.damping = 0.02
            self.gain = 15
            self.limit_lo = -1
            self.limit_hi = 1
        if self.motor == 'focus':
            self.damping = 0.02
            self.gain = 15
            self.limit_lo = 0
            self.limit_hi = 10
        
        self.direction = 0
        
        
        ################################################
        # ROS stuff
        
        node = motor + '_atmel_com'
        rospy.init_node(node, anonymous=True)
        self.last_time = rospy.get_time()
        
        # ros subscribers
        sub = motor + '_ctrl'
        rospy.Subscriber(sub, Float64, self.ctrl_callback)

        # ros publishers
        pub = motor + '_pos'
        self.pub = rospy.Publisher(pub, Float64)

        
        #################################################
        
        
        rospy.spin()

    def ctrl_callback(self, data):
        m_des_pos = data.data
        if m_des_pos < self.limit_lo: m_des_pos = self.limit_lo
        if m_des_pos > self.limit_hi: m_des_pos = self.limit_hi
             
        #print self.pos
        
        if self.dummy is True:
            self.pub.publish(Float64(self.pos))
            # controller:
            vel_des = self.gain*(m_des_pos-self.pos) 
            print self.pos, m_des_pos, vel_des
                
            # proposed acceleration:
            accel = (vel_des - self.vel)
            
            # set new desired velocity
            self.vel = self.vel + (vel_des - self.vel)*np.exp(-1*np.abs(accel)*self.damping)

            now = rospy.get_time()
            self.pos = self.vel*(now-self.last_time) + self.pos            
            self.last_time = now
            
        
        if self.dummy is False:
            self.pos = self.m.getpos()
            self.pub.publish(Float64(self.pos))
            
            # controller:
            vel_des = self.gain*(m_des_pos-self.pos) 
                    
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
    parser.add_option("--dummy", action="store_true", dest="dummy", default=False,
                        help="with dummy = True, will not attempt to talk to controller, but will return false motor values")
    (options, args) = parser.parse_args()
    
    motorcom = MotorCom(options.motor, dummy=options.dummy)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
