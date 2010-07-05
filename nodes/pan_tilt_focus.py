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
import pickle

import matplotlib.pyplot as plt
import scipy

import sys
sys.path.append("/home/floris/src/floris")
import camera_math

from optparse import OptionParser

def as_column(x):
    x = np.asarray(x)
    if len(x.shape) == 1:
        x = np.reshape(x, (x.shape[0],1) )
    return x

class PTMotor:
    def __init__(self, motor):
    
        self.motor = motor
        # latest motor states  
        self.pos = 0
        self.vel = 0
        self.pos_offset = 0
        if self.motor == 'tilt': self.ps3_gain = -0.02
        if self.motor == 'pan': self.ps3_gain = -0.02 
        self.home = 0
        
class FocusMotor:
    def __init__(self):
    
        # latest motor states  
        self.pos = 0
        self.vel = 0
        self.pos_offset = 0
        self.ps3_gain = 0.05
        self.home = 0
        
        # motor calibration values
        self.coeffs = [1, 0.01]
        
    def test_calc_focus(self, distc):
        focus_pos = self.coeffs[0] / (self.coeffs[1] + distc) + self.coeffs[2]
        #print focus_pos, distc, self.coeffs
        return focus_pos
        
    def test_calc_distc_from_focus(self, focus_pos):
        distc = self.coeffs[0] / (focus_pos - self.coeffs[2]) - self.coeffs[1]
        return distc
        
    def test_fmin_func(self, coeffs):
        focus = coeffs[0] / (self.distc+coeffs[1]) + coeffs[2]
        err = focus - self.data[:,2]
        abs_err = np.abs(err)
        err_sum = np.sum(abs_err)
        return err_sum
        
    def test_calibrate(self,data, camera_center, plot=1, fig = None):
        print 'calibrating'
        # data structure: [m1,m2,focus,x,y,z]
        self.data = data
        
        # calculate distances
        print 'calculating distances'
        self.distc = np.zeros(self.data.shape[0])
        for i in range(self.data.shape[0]):
            self.distc[i] = scipy.linalg.norm( self.data[i,3:6]-camera_center )
            
        focus = self.data[:,2]
        
        # fit a/(x) + c
        coeffs = np.polyfit(self.distc**(-1), focus, 1)
        ## now try fmin fit using coeffs as seed ##
        seed = np.zeros(3)
        seed[0] = coeffs[0]
        seed[2] = coeffs[1]
        
        tmp = scipy.optimize.fmin( self.fmin_func, seed, full_output = 1, disp=0)
        self.coeffs = tmp[0]
        print 'coefficients: ', self.coeffs
                
        if plot == 1:
            if fig is None:    
                fig = plt.figure(1)
            plt.scatter(self.distc,focus)
            xi = np.linspace(min(self.distc),max(self.distc),50)
            yi = [self.calc_focus(x) for x in xi]
            
            plt.title('Calibration data for Pan Tilt Focus')
            plt.xlabel('distance to camera center, m')
            plt.ylabel('focus motor setting, radians')
            plt.plot(xi,yi)
            #plt.show()
            print 'plotted focus'
        
        return 1
        
    def calc_focus(self, distc):
        focus_pos = distc*self.coeffs[0] + self.coeffs[1]
        #print focus_pos, distc, self.coeffs
        return focus_pos
        
    def calc_distc_from_focus(self, focus_pos):
        distc = (focus_pos - self.coeffs[1]) / self.coeffs[0]
        return distc
        
    def calibrate(self,data, camera_center, plot=1, fig = None):
        print 'calibrating'
        # data structure: [m1,m2,focus,x,y,z]
        self.data = data
        
        # calculate distances
        print 'calculating distances'
        self.distc = np.zeros(self.data.shape[0])
        for i in range(self.data.shape[0]):
            self.distc[i] = scipy.linalg.norm( self.data[i,3:6] - camera_center)
            
        focus = self.data[:,2]
        
        # fit d*(x) + c
        self.coeffs = np.polyfit(self.distc, focus, 1)

        print 'test coefficients: ', self.coeffs
                
        if plot == 1:
            if fig is None:    
                fig = plt.figure(2)
            plt.scatter(self.distc,focus)
            xi = np.linspace(min(self.distc),max(self.distc),50)
            yi = [self.calc_focus(x) for x in xi]
            
            plt.title('TEST Calibration data for Pan Tilt Focus')
            plt.xlabel('distance to camera center, m')
            plt.ylabel('focus motor setting, radians')
            plt.plot(xi,yi)
            plt.show()
            print 'plotted focus'
        
        return 1
        
class PanTiltFocusControl:

    def __init__(self, dummy = True, calibration_filename=None):
    
        # flydra object stuff
        self.pref_obj_id = None      
        self.pref_obj_position = None
        self.pref_obj_velocity = None
        self.pref_obj_latency = None            
        
        # calibration info
        self.calibration_raw_6d = None
        self.dummy = dummy
        
        self.ps3values_select = False
        self.ps3values_start = False
        self.ps3values_R1 = False
        
        # motors:
        self.pan = PTMotor('pan')
        self.tilt = PTMotor('tilt')
        self.focus = FocusMotor()
        
        ################################################
        # ROS stuff
        rospy.init_node('pantiltfocus_controller', anonymous=True)
        
        # ros publishers
        self.pub_pan_ctrl = rospy.Publisher("pan_ctrl", Float64)
        self.pub_tilt_ctrl = rospy.Publisher("tilt_ctrl", Float64)
        self.pub_focus_ctrl = rospy.Publisher("focus_ctrl", Float64)
        self.pub_ptf_3d = rospy.Publisher("ptf_3d", Point)
        self.pub_camera_center = rospy.Publisher("camera_center", Point)
        self.pub_ptf_home = rospy.Publisher("ptf_home", Point)
        
        # ros subscribers
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.flydra_callback)
        rospy.Subscriber("flydra_pref_obj_id", UInt32, self.obj_id_callback)
        rospy.Subscriber("pan_pos", Float64, self.pan_pos_callback)
        rospy.Subscriber("tilt_pos", Float64, self.tilt_pos_callback)
        rospy.Subscriber("focus_pos", Float64, self.focus_pos_callback)
        rospy.Subscriber("ps3_interpreter", ps3values, self.ps3_callback)
        
        
        
        #################################################
        
        if calibration_filename is not None:
            self.load_calibration_data(filename=calibration_filename)
        self.calibrate()
        
    #################  CALLBACKS  ####################################
        
    def flydra_callback(self, super_packet):

        now = time.time()
        for packet in super_packet.packets:
            self.pref_obj_latency =  now-packet.acquire_stamp.to_seconds()
            for obj in packet.objects:
                if obj.obj_id == self.pref_obj_id:
                    position = np.array([obj.position.x, obj.position.y, obj.position.z])
                    velocity = np.array([obj.velocity.x, obj.velocity.y, obj.velocity.z])
                    self.pref_obj_position = np.array(position)
                    self.pref_obj_velocity = np.array(velocity)
                    #print obj.obj_id
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
        
            L2gain = 2 - (ps3values.L2+1)
        
            self.tilt.pos_offset = self.tilt.pos_offset + ps3values.joyleft_y*self.tilt.ps3_gain*L2gain
            self.pan.pos_offset = self.pan.pos_offset + ps3values.joyleft_x*self.pan.ps3_gain*L2gain
        
            #if self.tilt.pos_offset > 1: self.tilt.pos_offset = 1
            #if self.tilt.pos_offset < -1: self.tilt.pos_offset = -1
            
            #if self.pan.pos_offset > 1: self.pan.pos_offset = 1
            #if self.pan.pos_offset < -1: self.pan.pos_offset = -1
            
            if ps3values.start is True:
                self.tilt.pos_offset = 0
                self.pan.pos_offset = 0

            # right joystick: focus
            self.focus.pos_offset = self.focus.pos_offset + ps3values.joyright_y*self.focus.ps3_gain*L2gain
            #if self.focus.pos_offset > 1: self.focus.pos_offset = 1
            #if self.focus.pos_offset < -1: self.focus.pos_offset = -1
            
            if ps3values.start is True:
                self.focus.pos_offset = 0
                
        # R2: move motor home position
        if ps3values.R2 < 0.9:
            self.tilt.home = self.tilt.home + ps3values.joyleft_y*self.tilt.ps3_gain
            self.pan.home = self.pan.home + ps3values.joyleft_x*self.pan.ps3_gain
        
            if ps3values.start is True:
                self.tilt.home = 0
                self.pan.home = 0

            # right joystick: focus
            self.focus.home = self.focus.home + ps3values.joyright_y*self.focus.ps3_gain
            
            if ps3values.start is True:
                self.focus.home = 0
                
            home_3d = self.to_world_coords(self.pan.home, self.tilt.home, self.focus.home, pub=False)
            self.pub_ptf_home.publish(Point(home_3d[0], home_3d[1], home_3d[2]))
            
        # L1: calibration
        if ps3values.L1 is True:
            
            if ps3values.select is True:
                self.ps3values_select = True
            if ps3values.select is False and self.ps3values_select is True:
                self.ps3values_select = False
                self.save_calibration_data()
                
            if ps3values.start is True:
                self.ps3values_start = True
            if ps3values.start is False and self.ps3values_start is True:
                self.ps3values_start = False
                self.calibrate()
                
            if ps3values.R1 is True:
                self.ps3values_R1 = True
            if ps3values.R1 is False and self.ps3values_R1 is True:
                self.ps3values_R1 = False
                self.save_calibration_data_to_file()
                
        self.generate_control()
        
    #################  PTF CALIBRATION  ####################################
    
    def calibrate(self):
        # need to check that we have enough points
        # do hydra cal stuff using self.calibration_raw_6d
        if self.dummy:
            self.Mhat = np.hstack((np.eye(3),np.array([[1],[1],[1]])))
        if not self.dummy:
            self.Mhat = camera_math.getMhat(self.calibration_raw_6d)
            
        self.Mhatinv = np.linalg.pinv(self.Mhat)
        #self.Mhat3x3inv = np.linalg.inv(self.Mhat[:,0:3]) # this is the left 3x3 section of Mhat, inverted
        #self.Mhat3x1 = self.Mhat[:,3] # this is the rightmost vertical vector of Mhat
            
        self.camera_center = np.array(camera_math.center(self.Mhat).T[0])
        if not self.dummy:
            self.focus.calibrate(self.calibration_raw_6d, self.camera_center)
            #self.focus.test_calibrate(self.calibration_raw_6d, self.camera_center)
        print 'camera_center: ', self.camera_center
        self.pub_camera_center.publish(Point(self.camera_center[0], self.camera_center[1], self.camera_center[2]))
        return
        
    def save_calibration_data(self):
        # i think it's [motors, fly_position]
        if self.pref_obj_id is None:
            print 'no active object id... no data saved'
            return 0
        new_cal_data = np.array([[self.pan.pos, self.tilt.pos, self.focus.pos, 
                                 self.pref_obj_position[0], self.pref_obj_position[1], self.pref_obj_position[2]]])
        print 'adding new calibration data: ', new_cal_data
        if self.calibration_raw_6d is None:
            self.calibration_raw_6d = new_cal_data
        else:
            self.calibration_raw_6d = np.vstack((self.calibration_raw_6d,new_cal_data))
            
        print
        print self.calibration_raw_6d
        print self.calibration_raw_6d.shape
        
        if self.calibration_raw_6d.shape[0] >= 6:
            self.dummy = False
            self.calibrate()
            
    def calc_distc(self,pos_3d):
        #print pos_3d, self.camera_center
        distc = scipy.linalg.norm( pos_3d[0:3]-self.camera_center ) # (might need to check orientation of vectors)
        return distc
            
    def load_calibration_data(self, filename=None):
        if filename is None:
            print 'NEED FILENAME'
        fname = (filename)
        fd = open( fname, mode='r')
        print
        print 'loading calibration... '
        self.calibration_raw_6d = pickle.load(fd)
        self.calibrate()
        
    def save_calibration_data_to_file(self, filename=None):
        if filename is None:
            filename = time.strftime("ptf_calibration_%Y%m%d_%H%M%S",time.localtime())
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
        distc = self.calc_distc(obj_pos)
        #distc = np.linalg.norm(q) # equivalent to above function call
        #print 'pos_3d: ', obj_pos
        #print 'distc: ', r,s,t
        pan_pos = np.arctan2(u,1) # focal length of 1, arbitrary
        tilt_pos = np.arctan2(v,1)
        focus_pos = self.focus.calc_focus(distc)
        motor_coords = [pan_pos, tilt_pos, focus_pos]
        #print motor_coords
        
        return motor_coords
        
    def to_world_coords(self, pan_pos, tilt_pos, focus_pos, pub=True):
        # takes three motor positions, and returns 3D point
        # for t:
        # find intersection of ray (r,s,t=anything) and sphere distc=distc, around camera center self.camera_center
        
        distc = self.focus.calc_distc_from_focus(focus_pos)
        
        
        
        
        u = np.tan(pan_pos)
        v = np.tan(tilt_pos)
        
        
        c1 = self.camera_center

        x2d = (u,v,1.0)
        c2 = numpy.dot(self.Mhatinv, as_column(x2d))[:,0]
        c2 = c2[:3]/c2[3]

        direction = c2-c1
        direction = direction/numpy.sqrt(numpy.sum(direction**2))
        pos_3d = c1+direction*distc
        #print pos_3d
        #print pos_3d
        if pub is True:
            #print pan_pos, tilt_pos, focus_pos, pos_3d
            self.pub_ptf_3d.publish(Point(pos_3d[0], pos_3d[1], pos_3d[2]))
        
        return pos_3d
            
    #################  CONTROL  ####################################
        
    def generate_control(self):
        
        # predict fly position:
        if self.pref_obj_id is not None and self.dummy is False:
            #print self.pref_obj_id, self.pref_obj_position, self.pref_obj_velocity, self.pref_obj_latency
            #try:
            self.predicted_obj_pos = self.pref_obj_position + self.pref_obj_velocity*self.pref_obj_latency
            obj_pos = np.hstack((self.predicted_obj_pos, [1]))
            motor_pos = self.to_motor_coords(obj_pos)
            #print obj_pos, '*'
            #except:
                #self.reset()
        if self.pref_obj_id is None or self.dummy is True:
            motor_pos = np.array([self.pan.home,self.tilt.home,self.focus.home])
        m_offset = np.array([self.pan.pos_offset, self.tilt.pos_offset, self.focus.pos_offset])
        m_des_pos = motor_pos + m_offset
        #print '*'*80
        #print m_des_pos
        

        #print self.to_motor_coords(obj_pos)
        # send out control signal
        if 1:
            self.pub_pan_ctrl.publish(Float64(m_des_pos[0]))
            self.pub_tilt_ctrl.publish(Float64(m_des_pos[1]))
            self.pub_focus_ctrl.publish(Float64(m_des_pos[2]))
            
        # publish best knowledge of points - should be a steady stream if the ps3 controller is running, otherwise should be in the pan, tilt, focus, position callbacks, but currently that would lead to three callbacks, which is ugly.
        #self.pub_ptf_3d.publish(Point(self.pan.pos, self.tilt.pos, self.focus.pos))
        self.to_world_coords(self.pan.pos, self.tilt.pos, self.focus.pos)
        
    def reset(self):
        # flydra object stuff
        self.pref_obj_id = None      
        self.pref_obj_position = None
        self.pref_obj_velocity = None
        self.pref_obj_latency = None
        
        
        
    def run(self):
        rospy.spin()
        
if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--calibration-filename", type="string", dest="calibration_filename", default=None,
                        help="camera calibration filename, raw 6d points collected using this same program")
    parser.add_option("--dummy", action='store_true', dest="dummy", default=False,
                        help="run using a dummy calibration, for testing")
    (options, args) = parser.parse_args()
    if options.calibration_filename is not None:
        options.dummy = False

    ptf_ctrl = PanTiltFocusControl(calibration_filename=options.calibration_filename, dummy=options.dummy)
    ptf_ctrl.run()
        
