#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_hydra')
import rospy
from std_msgs.msg import *
from ros_flydra.msg import *
from ros_hydra.msg import motorcom, motorctrl, motorlimits
from joystick_ps3.msg import ps3values
import time

import numpy as np

import sys
sys.path.append("/home/floris/src/floris")


from optparse import OptionParser

class MotorCom:
    
    def __init__(self, motor, dummy=False, zerostart=False):
    
        self.motor = motor
        self.dummy = dummy
        self.STOP_CTRL = False
        self.ps3values_playstation = False
        self.latency = 0.01
        self.m_des_vel_filtered = 0.
        self.m_des_pos_filtered = 0.
        self.zerostart = zerostart
        
        #################################################
        # init motor
        if motor == 'pan':
            sernum='0.0.B'
            gr=7.2
            clkdir_mult=1
            ind_per_rev=12800
            vel_max=30000
            vel_min=0
        if motor == 'tilt':
            sernum='0.0.A'
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
            self.m = stepper_motors.StepperMotor(sernum=sernum, gr=gr, clkdir_mult=clkdir_mult, ind_per_rev=ind_per_rev, vel_max=vel_max, vel_min=vel_min)
        #################################################
        
        # latest motor states  
        self.pos = 0
        self.vel = 0
        
        self.parameterupdate()
        if self.motor == 'pan':
            self.limit_lo = -1
            self.limit_hi = 1
            self.limit_buffer = 0.2
        if self.motor == 'tilt':
            self.limit_lo = -1
            self.limit_hi = 1
            self.limit_buffer = 0.2
        if self.motor == 'focus':
            self.limit_lo = -1000
            self.limit_hi = 1000
            self.limit_buffer = 0.2
        
        self.direction = 0
        
        
        ################################################
        # ROS stuff
        
        node = motor + '_atmel_com'
        rospy.init_node(node, anonymous=True)
        self.last_time = rospy.get_time()
        
        # ros subscribers
        sub = motor + '_ctrl'
        rospy.Subscriber(sub, motorctrl, self.ctrl_callback)
        rospy.Subscriber("ps3_interpreter", ps3values, self.ps3_callback)
        rospy.Subscriber("parameterupdate", Bool, self.parameterupdate)

        # ros publishers
        pub = motor + '_pos'
        pub_limits = motor + '_limits'
        self.pub = rospy.Publisher(pub, motorcom)
        self.pub_limits = rospy.Publisher(pub_limits, motorlimits)
        
        #################################################
        
        
        rospy.spin()
        
    def parameterupdate(self, data=True):
        # motor control characteristics 
        if not rospy.has_param('ptgain'):
            print 'setting default parameters'
            rospy.set_param('ptgain', 5)
            rospy.set_param('ptdamping', 0.5)
            rospy.set_param('focusgain', 5)
            rospy.set_param('focusdamping', 0.5)
            rospy.set_param('ptaccel', 0.5)
            rospy.set_param('ptminaccel','.01')
            rospy.set_param('focusaccel', 0.8)
        
        if self.motor == 'pan' or self.motor == 'tilt':
            self.damping = rospy.get_param('ptdamping')
            self.gain = rospy.get_param('ptgain')
            self.max_accel = rospy.get_param('ptaccel')
            self.min_accel = rospy.get_param('ptminaccel')
        if self.motor == 'focus':
            self.damping = rospy.get_param('focusdamping')
            self.gain = rospy.get_param('focusgain')
            self.max_accel = rospy.get_param('focusaccel')
            #self.min_accel = rospy.get_param('focusminaccel')
            
    def ps3_callback(self, ps3values):
        if self.dummy is False:
            if ps3values.R1 is False and ps3values.L1 is False: 
                # look for down press followed by up: so command is sent only once
                if ps3values.playstation is True:
                    self.ps3values_playstation = True
                if ps3values.playstation is False and self.ps3values_playstation is True:
                    self.ps3values_playstation = False
                    print 'finding zeros... '
                    self.STOP_CTRL = True
                    
                    if self.zerostart is True:
                        self.m.zerostart()
                        self.limit_lo = 0.1
                        self.limit_hi = 6
                        time.sleep(1)
                    else:
                        if self.motor != 'focus':
                            self.limit_lo, self.limit_hi = self.m.findzeros()
                            print 'limits: '
                            print self.limit_lo, self.limit_hi
                        if self.motor == 'focus':
                            self.m.infinity(duration=2,speed=-10)
                            self.limit_lo = 0.1
                            self.limit_hi = 4
                            time.sleep(1)
                        
                    self.limit_lo = self.limit_lo + self.limit_buffer
                    self.limit_hi = self.limit_hi - self.limit_buffer
                    print '*'*80
                    print 'limit_hi: ', self.limit_hi
                    print 'limit_lo: ', self.limit_lo
                    self.vel=0
                    self.STOP_CTRL = False
                
        # publish limit lo/hi
        self.pub_limits.publish(motorlimits(self.limit_lo, self.limit_hi))

    def ctrl_callback(self, data):
    
        if self.STOP_CTRL is True:
            return
    
        if self.STOP_CTRL is False:
            
            m_des_pos = data.pos
            m_des_vel = data.vel
            if 1:
                if m_des_pos < self.limit_lo: 
                    print 'HIT LO LIMIT, STOPPING!'
                    m_des_pos = self.limit_lo
                if m_des_pos > self.limit_hi: 
                    print 'HIT HI LIMIT, STOPPING!'
                    m_des_pos = self.limit_hi
                 
            #print self.pos
            
            if self.dummy is True:
                time0 = rospy.get_rostime()
                #print self.pos, m_des_pos
                self.pub.publish(motorcom(self.pos, self.latency, time0))
                # controller:
                vel_des = self.gain*(m_des_pos-self.pos) 
                    
                # proposed acceleration:
                accel = (vel_des - self.vel)
                
                # set new desired velocity
                self.vel = self.vel + (vel_des - self.vel)*np.exp(-1*np.abs(accel)*self.damping)

                now = rospy.get_rostime()
                self.pos = self.vel*(now-self.last_time) + self.pos            
                self.last_time = now
                self.latency = now-time0
                
            
            if self.dummy is False:
                
                # try input shaping. look at Teel's work
            
                time0 = rospy.get_rostime()
                dt = 0.03
                self.pos = self.m.getpos()
                self.pub.publish(motorcom(self.pos, self.latency, time0))
                
                # discrete low pass filter on desired position (wikipedia)
                self.alpha = dt / (self.damping + dt)
                self.m_des_pos_filtered = self.alpha*m_des_pos + (1.-self.alpha)*self.m_des_pos_filtered
                self.m_des_vel_filtered = self.alpha*m_des_vel + (1.-self.alpha)*self.m_des_vel_filtered
                
                
                # controller:
                #vel_des = self.gain*(m_des_pos-self.pos) + m_des_vel
                vel_des = self.gain*(self.m_des_pos_filtered-self.pos) + self.m_des_vel_filtered
                     
                # proposed acceleration:
                accel = (vel_des - self.vel)
                #print self.pos, m_des_pos, vel_des, accel
                
                ## old damping method ##
                # set new desired velocity
                # damping should be less than one if acceleration is very large, =1 is small
                #damp_factor = np.exp(np.abs(accel))*self.damping-self.damping
                #if damp_factor > 1.0:
                #    damp_factor = 0.8
                #
                
                if 1:
                    if np.abs(accel) > self.max_accel:# and (m_des_pos-self.pos) > 2.*np.pi/180.:
                        self.vel += self.max_accel*np.sign(accel)
                    else:
                        self.vel = vel_des
                
                self.vel = vel_des
                print self.m_des_pos_filtered-self.pos, vel_des, m_des_vel
                
                #self.vel = self.vel + (vel_des - self.vel)*(1-damp_factor)
                
                if self.pos < self.limit_lo: 
                    if self.vel < 0:
                        print 'stopping!'
                        self.vel = 0                    
                if self.pos > self.limit_hi: 
                    if self.vel > 0:
                        print 'stopping!'
                        self.vel = 0
                                    
                change_direction = 1
                if np.sign(self.vel) == self.direction:
                    change_direction = 0
                self.direction = np.sign(self.vel)
            
                #print motorid, m_control, m_current_vel, vel_des, accel, 'latency: ', latency
                self.m.setvel(self.vel, change_direction = change_direction)
                #print m_des_pos, self.pos, self.vel, m_des_vel
                self.lasttime = rospy.get_rostime()
                self.latency = self.lasttime.secs-time0.secs
                

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--motor", type="str", dest="motor", default='pan',
                        help="motor identifier, ie. pan, tilt, or focus")
    parser.add_option("--dummy", action="store_true", dest="dummy", default=False,
                        help="with dummy = True, will not attempt to talk to controller, but will return false motor values")
    parser.add_option("--zerostart", action="store_true", dest="zerostart", default=False,
                        help="zerostart uses stepper_motors.zerostart for zeroing routine")
    (options, args) = parser.parse_args()
    
    motorcom = MotorCom(options.motor, dummy=options.dummy, zerostart=options.zerostart)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
