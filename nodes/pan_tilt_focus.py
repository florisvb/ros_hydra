#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_hydra')
import rospy
from std_msgs.msg import *
from ros_flydra.msg import *
from ros_hydra.msg import motorcom, motorctrl, motorlimits
from joystick_ps3.msg import ps3values
from geometry_msgs.msg import Point
import time
import numpy as np
import numpy as numpy
import pickle
import copy


import cv

import matplotlib.pyplot as plt
import scipy

import sys
sys.path.append("/home/floris/src/floris")
import camera_math
import cvNumpy
import ptf_camera_classes

from optparse import OptionParser

def as_column(x):
    x = np.asarray(x)
    if len(x.shape) == 1:
        x = np.reshape(x, (x.shape[0],1) )
    return x

class PanTiltFocusControl:

    def __init__(self, dummy=True, calibration=None):
        
        # flydra object stuff
        self.pref_obj_id = None      
        self.pref_obj_position = None
        self.pref_obj_velocity = None
        self.pref_obj_latency = None            
        
        # calibration info
        self.calibration_raw_6d = None
        if calibration is None:
            self.dummy = True
        else:
            self.dummy = False
            
        self.ps3values_select = False
        self.ps3values_start = False
        self.ps3values_R1 = False
        self.ps3values = None
        
        self.parameterupdate()
        
        # motors:
        self.pan = ptf_camera_classes.PTMotor('pan')
        self.tilt = ptf_camera_classes.PTMotor('tilt')
        self.focus = ptf_camera_classes.FocusMotor()
        
        ################################################
        # ROS stuff
        rospy.init_node('pantiltfocus_controller', anonymous=True)
        
        # ros publishers
        self.pub_pan_ctrl = rospy.Publisher("pan_ctrl", motorctrl)
        self.pub_tilt_ctrl = rospy.Publisher("tilt_ctrl", motorctrl)
        self.pub_focus_ctrl = rospy.Publisher("focus_ctrl", motorctrl)
        self.pub_ptf_3d = rospy.Publisher("ptf_3d", Point)
        self.pub_camera_center = rospy.Publisher("camera_center", Point)
        self.pub_ptf_home = rospy.Publisher("ptf_home", Point)
        
        
        
        
        if calibration is not None:
            self.load_calibration_data(filename=calibration)
            if np.shape(self.calibration_raw_6d)[0] <= 4:
                self.dummy = True
        self.calibrate()
        
        
        # ros subscribers
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.flydra_callback)
        rospy.Subscriber("flydra_pref_obj_id", UInt32, self.obj_id_callback)
        rospy.Subscriber("pan_pos", motorcom, self.pan_pos_callback)
        rospy.Subscriber("tilt_pos", motorcom, self.tilt_pos_callback)
        rospy.Subscriber("focus_pos", motorcom, self.focus_pos_callback)
        rospy.Subscriber("pan_limits", motorlimits, self.pan_limits_callback)
        rospy.Subscriber("tilt_limits", motorlimits, self.tilt_limits_callback)
        rospy.Subscriber("focus_limits", motorlimits, self.focus_limits_callback)
        rospy.Subscriber("ps3_interpreter", ps3values, self.ps3_callback)
        rospy.Subscriber("parameterupdate", Bool, self.parameterupdate)
        
        
        
        #################################################
        
    #################  CALLBACKS  ####################################
    
    def parameterupdate(self, data=True):
        if not rospy.has_param('motoradjust_pan'):
            rospy.set_param('motoradjust_pan', 0)
        self.motoradjust_pan = rospy.get_param('motoradjust_pan')
        
        if not rospy.has_param('motoradjust_tilt'):
            rospy.set_param('motoradjust_tilt', 0)
        self.motoradjust_tilt = rospy.get_param('motoradjust_tilt')
        print 'parameter update', self.motoradjust_pan
        
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
        self.pan.pos = data.pos
        self.pan.latency = data.latency
    def tilt_pos_callback(self, data):
        self.tilt.pos = data.pos
        self.tilt.latency = data.latency
    def focus_pos_callback(self, data):
        self.focus.pos = data.pos
        self.focus.latency = data.latency
        
    def pan_limits_callback(self, data):
        self.pan.limit_lo = data.limit_lo
        self.pan.limit_hi = data.limit_hi
    def tilt_limits_callback(self, data):
        self.tilt.limit_lo = data.limit_lo
        self.tilt.limit_hi = data.limit_hi
    def focus_limits_callback(self, data):
        self.focus.limit_lo = data.limit_lo
        self.focus.limit_hi = data.limit_hi
        
    def ps3_callback(self,ps3values):
    
        # L2 + left joystick: move motors
        if ps3values.L2 < 0.9:
        
            L2gain = 2 - (ps3values.L2+1)
        
            self.tilt.pos_offset = self.tilt.pos_offset + ps3values.joyleft_y*self.tilt.ps3_gain*L2gain
            self.pan.pos_offset = self.pan.pos_offset + ps3values.joyleft_x*self.pan.ps3_gain*L2gain
            
            if ps3values.start is True:
                self.tilt.pos_offset = 0
                self.pan.pos_offset = 0

            # right joystick: focus
            self.focus.pos_offset = self.focus.pos_offset + ps3values.joyright_y*self.focus.ps3_gain*L2gain
            
            if ps3values.start is True:
                self.focus.pos_offset = 0
                
            if self.ps3values is not None:
                    
                # button presses
                nudge_pan = 0.05*np.sign(self.pan.ps3_gain)*np.pi/180.0
                nudge_tilt = -0.05*np.sign(self.tilt.ps3_gain)*np.pi/180.0
                nudge_focus = 0.2*np.sign(self.focus.ps3_gain)*np.pi/180.0
                if ps3values.up is False:
                    if self.ps3values.up is True:
                        self.tilt.pos_offset += nudge_tilt
                if ps3values.down is False:
                    if self.ps3values.down is True:
                        self.tilt.pos_offset -= nudge_tilt
                        
                if ps3values.right is False:
                    if self.ps3values.right is True:
                        self.pan.pos_offset += nudge_pan
                if ps3values.left is False:
                    if self.ps3values.left is True:
                        self.pan.pos_offset -= nudge_pan
                        
                if ps3values.triangle is False:
                    if self.ps3values.triangle is True:
                        self.focus.pos_offset += nudge_focus
                if ps3values.x is False:
                    if self.ps3values.x is True:
                        self.focus.pos_offset -= nudge_focus
                        
            # scanner
            #if self.ps3values.R2 < 0.9:
            #    if ps3values.R2 > 0.9:
            #        self.scan(increment=self.scanner_interval, pause=self.scanner_pause)
                
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
            
        # L1: calibration PT
        if ps3values.L1 is True:
            
            if ps3values.select is True:
                self.ps3values_select = True
            if ps3values.select is False and self.ps3values_select is True:
                self.ps3values_select = False
                self.save_calibration_data(calibrate=False)
                
            if ps3values.start is True:
                self.ps3values_start = True
            if ps3values.start is False and self.ps3values_start is True:
                self.ps3values_start = False
                self.dummy = False
                self.calibrate_pt(self.calibration_raw_6d)
                
            if ps3values.playstation is False and self.ps3values.playstation is True:
                self.save_calibration_data_to_file()
                
                
        self.ps3values = ps3values
        self.generate_control()
        
    #################  PTF CALIBRATION  ####################################
    
    def calibrate(self):
        # need to check that we have enough points
        # do hydra cal stuff using self.calibration_raw_6d
        if self.dummy:
            self.Mhat = np.hstack((np.eye(3),np.array([[1],[1],[1]])))
            self.Mhatinv = np.linalg.pinv(self.Mhat)
            self.camera_center = [0,0,0]
        
        if not self.dummy:
        
            # initially: calibrate with all data:
            #self.calibrate_pt(self.calibration_raw_6d)
            if self.calibration_raw_6d is not None:
                
                if 0:
                    self.calibrate_pt()
                    self.calibrate_focus()
                if 1:
                    errors = self.calibrate_pt_distorted()
                    print 'errors: ', errors
                    self.calibrate_focus()
                    
            #original_avg_2d_reprojection_error, original_avg_focus_reprojection_error = self.calc_reprojection_error(self.calibration_raw_6d)
            
            if 0:
                
                # loop to eliminate bad points
                n_data_pts = np.shape(self.calibration_raw_6d)[0]
                bad_pts = []
                for i in range(n_data_pts):
                    data_tmp = copy.copy(self.calibration_raw_6d)
                    data_tmp = np.delete(data_tmp, np.s_[i], axis=0) 
                    self.calibrate_pt(data_tmp)
                    self.focus.calibrate(data=data_tmp, camera_center=self.camera_center)
                    avg_2d_reprojection_error, avg_focus_reprojection_error = self.calc_reprojection_error(data_tmp)
                    err_diff = avg_2d_reprojection_error - original_avg_2d_reprojection_error
                    print 'reprojection errors: ', err_diff 
                    if err_diff < 1:
                        bad_pts.append(i)
                        
                # now redo calibration with only good points:
                n_good_pts = n_data_pts - len(bad_pts)
                self.calibration_filtered = np.zeros([n_good_pts,6])
                good_pts_counter = 0
                for i in range(n_data_pts):
                    if i not in bad_pts:
                        self.calibration_filtered[good_pts_counter,:] = self.calibration_raw_6d[i,:]
                        good_pts_counter += 1
                self.calibrate_pt(self.calibration_filtered)
                self.focus.calibrate(data=self.calibration_filtered, camera_center=self.camera_center, plot=True)
                new_2d_reprojection_error, new_focus_reprojection_error = self.calc_reprojection_error(self.calibration_filtered)   
                
                print
                print '2d reprojection error using all data: ', original_avg_2d_reprojection_error
                print '2d reprojection error using some data: ', new_2d_reprojection_error
                print 'focus reprojection error using all data: ', original_avg_focus_reprojection_error
                print 'focus reprojection error using some data: ', new_focus_reprojection_error
                print
                print 'removed x pts: ', len(bad_pts)
                
            #plt.show()
        
        #self.Mhat3x3inv = np.linalg.inv(self.Mhat[:,0:3]) # this is the left 3x3 section of Mhat, inverted
        #self.Mhat3x1 = self.Mhat[:,3] # this is the rightmost vertical vector of Mhat
        
        print 'camera_center: ', self.camera_center
        self.pub_camera_center.publish(Point(self.camera_center[0], self.camera_center[1], self.camera_center[2]))
        return
        
    def calibrate_pt_distorted(self):
    
        # do initial guess
        self.calibrate_pt()
        print 'initial camera_center: ', self.camera_center
        data = self.calibration_raw_6d
        
        def fmin_func(distortions):
            self.pan.center_displacement = distortions[0]
            self.pan.angular_displacement = distortions[1]
            self.tilt.center_displacement = distortions[2]
            self.tilt.angular_displacement = distortions[3]
            data_undistorted = copy.copy(data)
            for i in range(data_undistorted.shape[0]):
                data_undistorted[i,0:2] = self.undistort_ptf(data[i,3:6], data[i,0], data[i,1])
            errs = self.calibrate_pt(data_undistorted)
            print 'error: ', errs
            return errs
        
        # run loop
        distortions = [self.pan.center_displacement, self.pan.angular_displacement, self.tilt.center_displacement, self.tilt.angular_displacement]
        tmp = scipy.optimize.fmin( fmin_func, distortions, full_output = 1, disp=0)
        distortions = tmp[0]
        fmin_func(distortions)
        
        print distortions
        print self.camera_center
        
        return tmp[1]
        
        
    def calibrate_pt(self, data=None, focal_length=1.):
        if data is None:
            data = self.calibration_raw_6d
        points3D = cvNumpy.array_to_mat(np.asarray( data[:,3:6] ))
        points2D = cvNumpy.array_to_mat(np.asarray( np.tan( data[:,0:2] ) ))
        K = cvNumpy.array_to_mat(np.eye(3))
        K[0,0] = focal_length
        K[1,1] = focal_length
        rvec = cvNumpy.array_to_mat(np.zeros(3))
        tvec = cvNumpy.array_to_mat(np.zeros(3))
        distCoeffs = cvNumpy.array_to_mat(np.zeros(4))
        cv.FindExtrinsicCameraParams2(points3D, points2D, K, distCoeffs, rvec, tvec)
        # "align_points" ? in flydra
        # convert rotation vector to rotation matrix
        rmat = cvNumpy.array_to_mat(np.zeros([3,3]))
        cv.Rodrigues2( rvec, rmat )
        
        # generate P matrix
        self.R = cvNumpy.mat_to_array(rmat)
        self.t = cvNumpy.mat_to_array(tvec)
        self.K = cvNumpy.mat_to_array(K)
        self.Rt = np.hstack((self.R,self.t.T))
        self.P = np.dot( self.K, self.Rt )
        self.Mhat = self.P
        self.camera_center = camera_math.center(self.P)[:,0]
        self.distCoeffs = cvNumpy.mat_to_array(distCoeffs)
        
        # get reprojection errors
        reprojected_points2D = cvNumpy.array_to_mat(np.asarray( ( data[:,0:2] ) ))
        cv.ProjectPoints2(points3D, rvec, tvec, K, distCoeffs, reprojected_points2D)
        
        points2D_arr = cvNumpy.mat_to_array(points2D)
        reprojected_points2D_arr = cvNumpy.mat_to_array(reprojected_points2D)
        
        errors = [ np.linalg.norm(points2D_arr[i,:] - reprojected_points2D_arr[i,:]) for i in range(points2D_arr.shape[0]) ]
        
        # inverse Mhat:
        self.Mhatinv = np.linalg.pinv(self.Mhat)
        
        return np.mean(errors)
        
    def calibrate_pt_dlt(self, data=None):
        if data is None:
            data = self.calibration_raw_6d
        data3d = data[:,3:6]
        data2d = data[:,0:2]
        Pmat, residuals = camera_math.DLT(data3d, data2d)
        self.Mhat = Pmat
        self.Mhatinv = np.linalg.pinv(self.Mhat)
        self.camera_center = camera_math.center(Pmat)
        
        print 'mhat: '
        print self.Mhat
        print 'camera center: '
        print self.camera_center
        print 'residuals'
        print residuals
        print 'decomp'
        camera_math.decomp(self.Mhat, SHOW=1)
        return
        
    def calibrate_focus(self, data=None):
        if data is None:
            data = self.calibration_raw_6d
        data3d = data[:,3:6]
        data2d = data[:,0:2]
        self.focus.calibrate(data=data, camera_center=self.camera_center, plot=True)
        return
        
    def calc_reprojection_error(self,data):
        n_data_pts = np.shape(data)[0]
        reprojection_error_2d = np.zeros(n_data_pts)
        focus_reprojection_error = np.zeros(n_data_pts)
        reprojection_error_3d = np.zeros(n_data_pts)
        for i in range(n_data_pts):
            obj = data[i,3:6]
            #print 'object: ', obj
            pan_pos, tilt_pos, focus_pos = self.to_motor_coords(obj)
            reprojection_error_2d[i] = np.abs(pan_pos - data[i,0]) + np.abs(tilt_pos - data[i,1])
            focus_reprojection_error[i] = np.abs(focus_pos - data[i,2]) 
            reprojection_error_3d[i] = reprojection_error_2d[i] + focus_reprojection_error[i]
            
        avg_2d_reprojection_error = np.mean(reprojection_error_2d)
        avg_focus_reprojection_error = np.mean(focus_reprojection_error)
        return avg_2d_reprojection_error, avg_focus_reprojection_error 
        
    def fmin_calibration_func(self,displacements):
        
        # initial Mhat / camera center guess:
        Mhat, residuals = camera_math.getMhat(self.calibration_raw_6d)
        displaced_calibration = copy.copy(self.calibration_raw_6d)
        diff_Mhat = 100
        
        while diff_Mhat > 0.0005:
            camera_center = np.array(camera_math.center(Mhat).T[0])
            for i in range(displaced_calibration.shape[0]):
                # for distance correction term, the pan portion uses the planar radial distance, the tilt portion uses the full distance
                dist = np.linalg.norm(displaced_calibration[i,3:6] - camera_center)
                planar_dist = np.cos(self.calibration_raw_6d[i,1])*dist
                dist_arr = [planar_dist, dist]
                for j in range(2):
                    displaced_calibration[i,j] = displaced_calibration[i,j] + np.pi/2 - np.arccos(displacements[j]/dist_arr[j])
            new_Mhat, new_residuals = camera_math.getMhat(displaced_calibration)
            diff_Mhat = np.sum( np.abs(new_Mhat - Mhat) )
            Mhat = new_Mhat
            self.Mhat = Mhat
            self.camera_center = camera_center
            residuals = new_residuals
        print residuals, self.camera_center
        
        return residuals
        
    def calc_actual_dist_to_object(self, pos_3d):
        distc = self.calc_distc(pos_3d)
        actual_dist_to_object = np.sqrt(distc**2 + self.pan.center_displacement**2 + self.tilt.center_displacement**2)
        return actual_dist_to_object
        
    def undistort_ptf(self, obj_pos, pan_pos, tilt_pos):
        #obj_pos_in_ptf_coords = self.to_ptf_coords(obj_pos, pan_pos, tilt_pos)
        #distc = obj_pos_in_ptf_coords[2]
        distc = self.calc_distc(obj_pos)
        pan_pos_undistorted = np.arcsin( self.pan.center_displacement / distc ) + self.pan.angular_displacement + pan_pos
        tilt_pos_undistorted = np.arcsin( self.tilt.center_displacement / distc ) + self.tilt.angular_displacement + tilt_pos
        undistorted = [pan_pos_undistorted, tilt_pos_undistorted]
        return undistorted
        
    def distort_ptf(self, obj_pos, pan_pos, tilt_pos):
        distc = self.calc_distc(obj_pos)
        pan_pos_distorted = pan_pos - (np.arcsin( self.pan.center_displacement / distc ) + self.pan.angular_displacement)
        tilt_pos_distorted = tilt_pos - (np.arcsin( self.tilt.center_displacement / distc ) + self.tilt.angular_displacement)
        distorted = [pan_pos_distorted, tilt_pos_distorted]
        return distorted
        
        
    def save_calibration_data(self, calibrate=True):
        # [motors, fly_position]
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
        
        if calibrate:
            if self.calibration_raw_6d.shape[0] >= 4:
                self.dummy = False
                self.calibrate()
                
    def calc_distc(self,pos_3d):
        #print pos_3d, self.camera_center
        distc = scipy.linalg.norm( pos_3d[0:3]-self.camera_center ) # (might need to check orientation of vectors)
        return distc
            
    def load_calibration_data(self, filename=None, motor='pt'):
        if filename is None:
            print 'NEED FILENAME'
        fname = (filename)
        fd = open( fname, mode='r')
        print
        print 'loading calibration... '
        if motor == 'pt':
            self.calibration_raw_6d = pickle.load(fd)
        elif motor == 'focus': 
            self.focus_calibration = pickle.load(fd)
            
    def save_calibration_data_to_file(self, filename=None):
        if filename is None:
            filename = time.strftime("ptf_calibration_%Y%m%d_%H%M%S",time.localtime())
        print 'saving calibration to file: ', filename
        fname = (filename)  
        fd = open( fname, mode='w' )
        pickle.dump(self.calibration_raw_6d, fd)
        return 1

    def scan(self, increment=5.0*np.pi/180.0, pause=0.1, save_data=True):
        self.dummy = True
        
        # go to the upper left corner
        self.pan.home = 0
        self.tilt.home = 0
        self.focus.home = 0
        self.pan.pos_offset = self.pan.limit_lo+increment
        self.tilt.pos_offset = self.tilt.limit_hi-increment
        self.focus.pos_offset = 1
        print self.pan.pos_offset, self.tilt.pos_offset, self.focus.pos_offset
        self.drive(1)
                
        if 1:
            # scan across a pan line:
            def pan_scan_loop(increment, direction, pause, save_data):
                self.pan.pos_offset += increment*direction
                self.drive(pause)
                if save_data:
                    if self.pref_obj_id is None:
                        print 'no obj id selected... will wait for 0.5 sec'
                        self.drive(0.5)
                    self.save_calibration_data(calibrate=False)
            
            def pan_scan(increment, direction, pause, save_data):
                print 'pan increment'
                if direction > 0:
                    while self.pan.pos_offset <= self.pan.limit_hi:
                        pan_scan_loop(increment, direction, pause, save_data)
                if direction < 0:
                    while self.pan.pos_offset >= self.pan.limit_lo:
                        pan_scan_loop(increment, direction, pause, save_data)
                        
            direction = 1
            while self.tilt.pos_offset <= self.tilt.limit_hi and self.tilt.pos_offset > self.tilt.limit_lo:
                self.tilt.pos_offset -= increment
                print 'tilt increment'
                self.generate_control()
                pan_scan(increment, direction, pause, save_data)
                direction *= -1
    
    def drive(self, duration, rate=100):
        # run the control routine for 'duration' seconds at 'rate' in Hz
        time0 = rospy.get_time()
        r = rospy.Rate(rate) # rate, hz
        while rospy.get_time()-time0 < duration:
            self.generate_control()
            r.sleep()
        
    def to_motor_coords(self, obj_pos, offset=[0,0,0]):
        # takes 3D object as input, returns the three corresponding motor positions
        # back out desired motor positions
        if len(obj_pos) == 3:
            obj_pos = np.hstack((obj_pos, [1]))
        obj_pos = np.asarray(obj_pos)
        #print obj_pos, self.Mhat.shape
        q = np.dot(self.Mhat,obj_pos.T)
        r = q[0] # camera coord x
        s = q[1] # camera coord y
        t = q[2] # camera coord z
        v = s/t
        u = r/t 
        distc = self.calc_distc(obj_pos)
        #distc = self.calc_actual_dist_to_object(obj_pos)
        #distc = np.linalg.norm(q) # equivalent to above function call
        #print 'pos_3d: ', obj_pos
        #print 'distc: ', r,s,t
        #print self.motoradjust_pan*distc
        pan_pos = np.arctan2(u+offset[0],1) # focal length of 1, arbitrary
        tilt_pos = np.arctan2(v+offset[1],1)
        
        pt_pos = self.distort_ptf(obj_pos, pan_pos, tilt_pos)
        pan_pos = pt_pos[0]
        tilt_pos = pt_pos[1]
        
        focus_pos = self.focus.calc_focus(obj_pos) + offset[2]
        motor_coords = [pan_pos, tilt_pos, focus_pos]
        #print motor_coords
        
        return motor_coords
        
    def to_ptf_coords(self, obj_pos, pan_pos=None, tilt_pos=None):
    
        #if len(obj_pos) == 3:
        #    obj_pos = np.hstack((obj_pos, [1]))

        if pan_pos is None:
            pan_pos = self.pan.pos
        if tilt_pos is None: 
            tilt_pos = self.tilt.pos            

        obj_in_static_ptf_coords = np.dot(self.R,obj_pos)
        print obj_in_static_ptf_coords
        
        ## WRONG
        Rn_pan = camera_math.build_rot_mat([0,1,0], pan_pos)
        Rn_tilt = camera_math.build_rot_mat([1,0,0], tilt_pos)
        Rn = np.dot(Rn_pan, Rn_tilt)
        
        obj_in_current_ptf_coords = np.dot(Rn, obj_in_static_ptf_coords) + self.t
        
        return obj_in_current_ptf_coords
        
    def to_motor_coords_vel(self, obj_pos, obj_vel):
        
        dt = 1. # arbitrary, and irrelevant (gets cancelled out)
        
        obj_pos_1 = obj_pos
        obj_pos_2 = obj_pos+obj_vel*dt
        
        motor_pos_1 = np.array(self.to_motor_coords(obj_pos_1))
        motor_pos_2 = np.array(self.to_motor_coords(obj_pos_2))
        
        motor_vel = (motor_pos_2 - motor_pos_1) / dt
        
        return motor_vel
        
    def to_world_coords(self, pan_pos, tilt_pos, focus_pos, pub=False):
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
        #print 'pos 3d: ', pos_3d
        #print pos_3d
        #print pos_3d
        if pub is True:
            print pan_pos, tilt_pos, focus_pos, pos_3d
            self.pub_ptf_3d.publish(Point(pos_3d[0], pos_3d[1], pos_3d[2]))
        
        return pos_3d
            
    #################  CONTROL  ####################################
        
    def generate_control(self):
    
        offset = np.array([self.pan.pos_offset, self.tilt.pos_offset, self.focus.pos_offset])
        
        # predict fly position:
        if self.pref_obj_id is not None and self.dummy is False:
            #print self.pref_obj_id, self.pref_obj_position, self.pref_obj_velocity, self.pref_obj_latency
            #try:
            motor_latency = np.min([self.pan.latency, self.tilt.latency, self.focus.latency]) 
            self.predicted_obj_pos = self.pref_obj_position + self.pref_obj_velocity*(self.pref_obj_latency+motor_latency)
            obj_pos = np.hstack((self.predicted_obj_pos, [1]))
            motor_pos = self.to_motor_coords(obj_pos, offset=offset)
            m_des_vel = self.to_motor_coords_vel(self.predicted_obj_pos, self.pref_obj_velocity)
                
                
        if self.pref_obj_id is None and self.dummy is False:
            #m_offset = self.to_motor_coords([0,0,0], offset=offset)
            motor_pos = np.array([self.pan.home,self.tilt.home,self.focus.home]) + offset
            m_des_vel = [0,0,0]
            
        if self.dummy is True:
            motor_pos = np.array([self.pan.home,self.tilt.home,self.focus.home]) + offset
            m_des_vel = [0,0,0]
            
        
        #print 'motor pos: ', motor_pos
        #print 'motor offset: ', m_offset
        m_des_pos = motor_pos
        #print '*'*80
        #print m_des_pos
        

        #print self.to_motor_coords(obj_pos)
        # send out control signal
        if 1:
            self.pub_pan_ctrl.publish(motorctrl(m_des_pos[0], m_des_vel[0]))
            self.pub_tilt_ctrl.publish(motorctrl(m_des_pos[1], m_des_vel[1]))
            self.pub_focus_ctrl.publish(motorctrl(m_des_pos[2], m_des_vel[2]))
            
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
    parser.add_option("--calibration", type="string", dest="calibration", default=None,
                        help="camera calibration filename, raw 6d points collected using this same program")
    parser.add_option("--dummy", action='store_true', dest="dummy", default=False,
                        help="run using a dummy calibration, for testing")
    (options, args) = parser.parse_args()

    ptf_ctrl = PanTiltFocusControl(calibration=options.calibration, dummy=options.dummy)
    ptf_ctrl.run()
        
