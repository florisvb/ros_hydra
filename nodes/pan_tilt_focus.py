#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_hydra')
import rospy
from std_msgs.msg import *
from ros_flydra.msg import *
from geometry_msgs.msg import Point
import time
import numpy as np


class MotorControl:
    def __init__(self, motor):
    
        self.motor = motor
        # latest motor states  
        self.pos = 0
        self.vel = 0
        self.pos_offset = 0
        

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
        self.pan = MotorControl('pan')
        self.tilt = MotorControl('tilt')
        self.focus = MotorControl('focus')
    
        ################################################
        # ROS stuff
        
        # ros subscribers
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.flydra_callback)
        rospy.Subscriber("flydra_pref_obj_id", UInt32, self.obj_id_callback)
        rospy.Subscriber("pan_pos", Float64, self.pan_pos_callback)
        rospy.Subscriber("tilt_pos", Float64, self.tilt_pos_callback)
        rospy.Subscriber("focus_pos", Float64, self.focus_pos_callback)
        rospy.Subscriber("wx_controller", String, self.wx_controller_callback)
        
        
        # ros publishers
        self.pub_pan_ctrl = rospy.Publisher("pan_ctrl", Float64)
        self.pub_tilt_ctrl = rospy.Publisher("tilt_ctrl", Float64)
        self.pub_focus_ctrl = rospy.Publisher("focus_ctrl", Float64)
        self.pub_ptf_3d = rospy.Publisher("ptf_3d", Point)
        
        rospy.init_node('pantiltfocus_controller', anonymous=True)
        #################################################
        
        
        
        
    #################  CALLBACKS  ####################################
        
    def flydra_callback(self, super_packet):
        print 'super packet, ', len(super_packet.packets), ' packets'
        now = time.time()
        for packet in super_packet.packets:
            print
            print '*'*80
            self.pref_obj_latency =  now-packet.acquire_stamp.to_seconds()
            for obj in packet.objects:
                if obj.obj_id == self.pref_obj_id:
                    position = [obj.position.x, obj.position.y, obj.position.z]
                    velocity = [obj.velocity.x, obj.velocity.y, obj.velocity.z]
                    print
                    print position
                    print velocity
                    self.pref_obj_position = position
                    self.pref_obj_velocity = velocity
        self.generate_control()
        
    def obj_id_callback(self, data):
        new_pref_obj_id = data.data
        
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
        
    def wx_controller_callback(self,data):
        
        nudge = 0.005
        
        # up arrow
        if data.data == 'up':
            self.tilt.pos_offset = self.tilt.pos_offset + nudge
            
        # down arrow
        if data.data == 'down':
            self.tilt.pos_offset = self.tilt.pos_offset - nudge
            
        # right arrow
        if data.data == 'right':
            self.pan.pos_offset = self.pan.pos_offset + nudge
            
        # left arrow
        if data.data == 'left':
            self.pan.pos_offset = self.pan.pos_offset - nudge
            
        # clear
        if data.data == 'clear':
            self.tilt.pos_offset = 0
            self.pan.pos_offset = 0
        
        self.generate_control()
        
    #################  PTF CALIBRATION  ####################################
    
    def calibrate(self):
        # need to check that we have enough points
        # do hydra cal stuff using self.calibration_raw_6d
        if self.dummy:
            self.Mhat = np.hstack((np.eye(3),np.array([[1],[1],[1]])))
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
        pickle.dump(dataset, fd)
        return 1
            
    def to_motor_coords(self, obj_pos):
        # takes 3D object as input, returns the three corresponding motor positions
        # back out desired motor positions
        obj_pos = np.array(obj_pos)
        #print obj_pos.shape, self.Mhat.shape
        q = np.dot(self.Mhat,obj_pos.T)
        r = q[0]
        s = q[1]
        t = q[2]
        v = s/t
        u = r/t 
        pan_pos = np.arctan2(u,1)
        tilt_pos = np.arctan2(v,1)
        focus_pos = 10 # focus function
        
        motor_coords = [pan_pos, tilt_pos, focus_pos]
        #print motor_coords
        
        return motor_coords
        
    def to_world_coords(self, pan_pos, tilt_pos, focus_pos):
        # takes three motor positions, and returns 3D point
        if self.dummy:
            pos_3d = [pan_pos-1, tilt_pos-1, focus_pos-1]
            
        self.pub_ptf_3d.publish(Point(pos_3d[0], pos_3d[1], pos_3d[2]))
            
            
    #################  CONTROL  ####################################
        
    def generate_control(self):
        
        # predict fly position:
        if self.pref_obj_id is not None:
            self.predicted_obj_pos = self.pref_obj_position + self.pref_obj_velocity*self.pref_obj_latency
            obj_pos = np.hstack((self.predicted_obj_pos, [1]))
        else:
            obj_pos = [1,1,1,1]
        m_offset = np.array([self.pan.pos_offset, self.tilt.pos_offset, self.focus.pos_offset])
        m_des_pos = self.to_motor_coords(obj_pos) + m_offset

        #print self.to_motor_coords(obj_pos)
        # send out control signal
        if 1:
            self.pub_pan_ctrl.publish(Float64(m_des_pos[0]))
            self.pub_tilt_ctrl.publish(Float64(m_des_pos[1]))
            self.pub_focus_ctrl.publish(Float64(m_des_pos[2]))
                
        
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
        
