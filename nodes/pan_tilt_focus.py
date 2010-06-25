#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_hydra')
import rospy
from std_msgs.msg import *
from ros_flydra.msg import *
from joystick_ps3.msg import ps3values
from geometry_msgs.msg import Point
import time
import numpy as np
import numpy as numpy

import sys
sys.path.append("/home/floris/src/floris")
import camera_math


class PTMotor:
    def __init__(self, motor):
    
        self.motor = motor
        # latest motor states  
        self.pos = 0
        self.vel = 0
        self.pos_offset = 0
        self.ps3_gain = 0.01
        
class FocusMotor:
    def __init__(self):
    
        # latest motor states  
        self.pos = 0
        self.vel = 0
        self.pos_offset = 0
        self.ps3_gain = 0.01
        
        # motor calibration values
        self.coeffs = [1, 0, 0]
        self.camera_center = [0,0,0]
        
    def calc_focus(self, distc):
        focus_pos = self.coeffs[0] / (self.coeffs[1] + distc) + self.coeffs[2]
        return focus_pos
        
    def calc_distc_from_focus(self, focus_pos):
        distc = self.coeffs[0] / (focus_pos - self.coeffs[2]) - self.coeffs[1]
        return distc

class PanTiltFocusControl:

    def __init__(self, dummy = True):
    
        # flydra object stuff
        self.pref_obj_id = None      
        self.pref_obj_position = None
        self.pref_obj_velocity = None
        self.pref_obj_latency = None            
        
        # calibration info
        self.calibration_raw_6d = None
        self.dummy = dummy
        self.calibrate()
        
        # motors:
        self.pan = PTMotor('pan')
        self.tilt = PTMotor('tilt')
        self.focus = FocusMotor()
    
        ################################################
        # ROS stuff
        rospy.init_node('pantiltfocus_controller', anonymous=True)
        
        # ros subscribers
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.flydra_callback)
        rospy.Subscriber("flydra_pref_obj_id", UInt32, self.obj_id_callback)
        rospy.Subscriber("pan_pos", Float64, self.pan_pos_callback)
        rospy.Subscriber("tilt_pos", Float64, self.tilt_pos_callback)
        rospy.Subscriber("focus_pos", Float64, self.focus_pos_callback)
        rospy.Subscriber("ps3_interpreter", ps3values, self.ps3_callback)
        
        
        # ros publishers
        self.pub_pan_ctrl = rospy.Publisher("pan_ctrl", Float64)
        self.pub_tilt_ctrl = rospy.Publisher("tilt_ctrl", Float64)
        self.pub_focus_ctrl = rospy.Publisher("focus_ctrl", Float64)
        self.pub_ptf_3d = rospy.Publisher("ptf_3d", Point)
        
        
        #################################################
        
        
        
        
    #################  CALLBACKS  ####################################
        
    def flydra_callback(self, super_packet):

        now = time.time()
        for packet in super_packet.packets:
            self.pref_obj_latency =  now-packet.acquire_stamp.to_seconds()
            for obj in packet.objects:
                if obj.obj_id == self.pref_obj_id:
                    position = np.array([obj.position.x, obj.position.y, obj.position.z])
                    velocity = np.array([obj.velocity.x, obj.velocity.y, obj.velocity.z])
                    self.pref_obj_position = position
                    self.pref_obj_velocity = velocity
                    print obj.obj_id
                    self.generate_control()
        
    def obj_id_callback(self, data):
        if data.data != 0:
            new_pref_obj_id = data.data
        elif data.data == 0:
            new_pref_obj_id = None
        if self.pref_obj_id is not None:
            if new_pref_obj_id is None:
                self.reset()
        self.pref_obj_id = new_pref_obj_id
        
    def pan_pos_callback(self, data):
        self.pan.pos = data.data
    def tilt_pos_callback(self, data):
        self.tilt.pos = data.data
    def focus_pos_callback(self, data):
        self.focus.pos = data.data
        
    def ps3_callback(self,ps3values):
    
        # L2 + left joystick: move motors
        if ps3values.L2 < 0.9:
            self.tilt.pos_offset = self.tilt.pos_offset + ps3values.joyleft_y*self.tilt.ps3_gain
            self.pan.pos_offset = self.pan.pos_offset + ps3values.joyleft_x*self.pan.ps3_gain
        
            if self.tilt.pos_offset > 1: self.tilt.pos_offset = 1
            if self.tilt.pos_offset < -1: self.tilt.pos_offset = -1
            
            if self.pan.pos_offset > 1: self.pan.pos_offset = 1
            if self.pan.pos_offset < -1: self.pan.pos_offset = -1
            
            if ps3values.start is True:
                self.tilt.pos_offset = 0
                self.pan.pos_offset = 0

        # R2 + right joystick: focus
        if ps3values.R2 < 0.9:
            self.focus.pos_offset = self.focus.pos_offset + ps3values.joyright_y*self.focus.ps3_gain
            if self.focus.pos_offset > 1: self.focus.pos_offset = 1
            if self.focus.pos_offset < -1: self.focus.pos_offset = -1
            
            print 'focusing!!', ps3values.joyright_y
            
            if ps3values.start is True:
                self.focus.pos_offset = 0
        
        self.generate_control()
        
    #################  PTF CALIBRATION  ####################################
    
    def calibrate(self):
        # need to check that we have enough points
        # do hydra cal stuff using self.calibration_raw_6d
        if self.dummy:
            self.Mhat = np.hstack((np.eye(3),np.array([[1],[1],[1]])))
            self.Mhat3x3inv = np.linalg.inv(self.Mhat[:,0:3]) # this is the left 3x3 section of Mhat, inverted
            self.Mhat3x1 = self.Mhat[:,3] # this is the rightmost vertical vector of Mhat
            
        if not self.dummy:
            self.Mhat = camera_math.getMhat(self.calibration_raw_6d)
            
        self.camera_center = camera_math.center(self.Mhat).T
        return
        
    def save_calibration_data(self):
        # i think it's [motors, fly_position]
        if self.pref_obj_id is None:
            print 'no active object id... no data saved'
            return 0
        new_cal_data = np.array([self.pan.pos, self.tilt.pos, self.focus.pos, 
                                 self.pref_obj_id_position[0], self.pref_obj_id_position[1], self.pref_obj_id_position[2]])
        if self.calibration_raw_6d is None:
            self.calibration_raw_6d = new_cal_data
        else:
            self.calibration_raw_6d = np.vstack((self.calibration_raw_6d,new_cal_data))
            
        if self.calibration_raw_6d.shape[0] >= 3:
            self.calibrate()
            
    def calc_distc(self,pos_3d):
        distc = scipy.linalg.norm( pos_3d-self.camera_center ) # (might need to check orientation of vectors)
        return distc
            
    def load_calibration_data(self, filename):
        fname = (filename)
        fd = open( fname, mode='r')
        print 'loading calibration... '
        self.calibration_raw_6d = pickle.load(fd)
        self.calibrate()
        
    def save_calibration_data_to_file(self, filename):
        print 'saving calibration to file: ', filename
        fname = (filename)  
        fd = open( fname, mode='w' )
        pickle.dump(self.calibration_raw_6d, fd)
        return 1
        
    
        
    def to_motor_coords(self, obj_pos):
        # takes 3D object as input, returns the three corresponding motor positions
        # back out desired motor positions
        obj_pos = np.array(obj_pos)
        #print obj_pos.shape, self.Mhat.shape
        q = np.dot(self.Mhat,obj_pos.T)
        r = q[0] # camera coord x
        s = q[1] # camera coord y
        t = q[2] # camera coord z
        v = s/t
        u = r/t 
        #distc = self.calc_distc(obj_pos)
        distc = np.linalg.norm(q) # equivalent to above function call
        
        pan_pos = np.arctan2(u,1) # focal length of 1, arbitrary
        tilt_pos = np.arctan2(v,1)
        focus_pos = self.focus.calc_focus(distc)
        motor_coords = [pan_pos, tilt_pos, focus_pos]
        #print motor_coords
        
        return motor_coords
        
    def to_world_coords(self, pan_pos, tilt_pos, focus_pos):
        # takes three motor positions, and returns 3D point
        # for t:
        # find intersection of ray (r,s,t=anything) and sphere distc=distc, around camera center self.camera_center
        
        distc = self.focus.calc_distc_from_focus(focus_pos)
        
        u = np.tan(pan_pos)
        v = np.tan(tilt_pos)
        t = distc / np.sqrt(1+u**2+v**2)
        s = v*t
        r = u*t
        q = np.array([r,s,t])
        
        pos_3d = np.dot(self.Mhat3x3inv, q-self.Mhat3x1)
        self.pub_ptf_3d.publish(Point(pos_3d[0], pos_3d[1], pos_3d[2]))
        
        return pos_3d
            
    #################  CONTROL  ####################################
        
    def generate_control(self):
        
        # predict fly position:
        if self.pref_obj_id is not None:
            print self.pref_obj_id, self.pref_obj_position, self.pref_obj_velocity, self.pref_obj_latency
            self.predicted_obj_pos = self.pref_obj_position + self.pref_obj_velocity*self.pref_obj_latency
            obj_pos = np.hstack((self.predicted_obj_pos, [1]))
        else:
            obj_pos = np.array([0,0,0,1])
        m_offset = np.array([self.pan.pos_offset, self.tilt.pos_offset, self.focus.pos_offset])
        m_des_pos = self.to_motor_coords(obj_pos) + m_offset

        #print self.to_motor_coords(obj_pos)
        # send out control signal
        if 1:
            print
            print '*'*80
            print m_des_pos[0]
            self.pub_pan_ctrl.publish(Float64(m_des_pos[0]))
            self.pub_tilt_ctrl.publish(Float64(m_des_pos[1]))
            self.pub_focus_ctrl.publish(Float64(m_des_pos[2]))
            
        # publish best knowledge of points - should be a steady stream if the ps3 controller is running, otherwise should be in the pan, tilt, focus, position callbacks, but currently that would lead to three callbacks, which is ugly.
        self.pub_ptf_3d.publish(Point(self.pan.pos, self.tilt.pos, self.focus.pos))
                
        
    def reset(self):
        # flydra object stuff
        self.pref_obj_id = None      
        self.pref_obj_position = None
        self.pref_obj_velocity = None
        self.pref_obj_latency = None
        
        
        
    def run(self):
        rospy.spin()
        
if __name__ == '__main__':
    ptf_ctrl = PanTiltFocusControl()
    ptf_ctrl.run()
        
