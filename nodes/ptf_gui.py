#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_hydra')
import rospy
from std_msgs.msg import *
from ros_flydra.msg import *
from ros_hydra.msg import motorcom, motorctrl, motorlimits
from joystick_ps3.msg import ps3values
import time
import wx

import numpy as np

import sys
sys.path.append("/home/floris/src/floris")


from optparse import OptionParser

class FloatSlider(wx.Slider):
    def GetValue(self):
        return (float(wx.Slider.GetValue(self)))/self.GetMax()
        
class SliderPackage:
    def __init__(self, frame, display_name='slider', defaultval=0, minval=0, maxval=1, resolution=1000, display=True, pos=(30,30), length=100, orientation='horizontal'):
        # only supports horizontal sliders for now
        if orientation == 'horizontal':
            orient = wx.SL_HORIZONTAL
        elif orientation == 'vertical':
            orient = wx.SL_VERTICAL
            
        self.minval = minval
        self.maxval = maxval
        self.defaultval = defaultval
        x = pos[0]
        y = pos[1]
        left_origin = x
        x_slider = x+15+len(str(self.minval))*5
        defaultval_ticks = int(float(self.defaultval-self.minval)/float(self.maxval)*float(resolution))
        self.minlabel = wx.StaticText(frame, -1, str(self.minval) , (left_origin, y))
        self.maxlabel = wx.StaticText(frame, -1, str(self.maxval) , (x_slider+length+5, y))
        self.vallabel = wx.StaticText(frame, -1, str(defaultval) , (left_origin + 6*len(display_name)+25, y-18))
        s = display_name + ': '
        self.name = wx.StaticText(frame, -1, s, (left_origin, y-18))
        self.slider = FloatSlider(frame, -1, defaultval_ticks, 0, resolution, (x_slider,y), (length,-1), orient)
        
    def getval(self):
        rawval = self.slider.GetValue()
        val = rawval*(self.maxval-self.minval)+self.minval
        self.vallabel.SetLabel(str(val))
        return val
        
# TODO:
# 1. add a way to save and load the parameters       
# negative numbers and sliderpackage dont work!

class Frame(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, title='Events Test')
        panel = wx.Panel(self)
        
        #self.radioList = ['blue', 'red', 'yellow', 'orange', 'green', 'purple', 'navy blue', 'black', 'gray']
        #rb = wx.RadioBox(self, label="What color would you like ?", pos=(100, 100), choices=self.radioList,  majorDimension=3,
                         #style=wx.RA_SPECIFY_COLS)
        #self.Bind(wx.EVT_RADIOBOX, self.EvtRadioBox, rb)

        
        # motor control parameters
        motor_control_x_pos = 30
        self.ptgain_slider = SliderPackage(panel, 'pan/tilt gains', 10, 0, 100, pos=(motor_control_x_pos,50), length=200)
        self.ptdamping_slider = SliderPackage(panel, 'pan/tilt damping', 0.25, 0, 1, pos=(motor_control_x_pos,150), length=200)
        self.ptaccel_slider = SliderPackage(panel, 'pan/tilt max accel', 0.12, 0, 1, pos=(motor_control_x_pos,100), length=200)
        #self.ptminaccel_slider = SliderPackage(panel, 'pan/tilt min accel', .01, 0, 0.01, pos=(motor_control_x_pos,150), length=200)
        self.focusgain_slider = SliderPackage(panel, 'focus gains', 5, 0, 10, pos=(motor_control_x_pos,200), length=200)
        self.focusdamping_slider = SliderPackage(panel, 'focus damping', 0.5, 0, 1, pos=(motor_control_x_pos,300), length=200)
        self.focusaccel_slider = SliderPackage(panel, 'focus max accel', 0.5, 0, 1, pos=(motor_control_x_pos,250), length=200)
        #self.focusminaccel_slider = SliderPackage(panel, 'focus min accel', .1, 0, .1, pos=(motor_control_x_pos,300), length=200)
        
        # scanning parameters
        self.motoradjust_pan_slider = SliderPackage(panel, 'motoradjust pan', 0, -.05, .05, pos=(motor_control_x_pos,350), length=200)
        self.motoradjust_tilt_slider = SliderPackage(panel, 'motoradjust tilt', 0, -.05, .05, pos=(motor_control_x_pos,400), length=200)
        
        
        
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)
        
        

        self.Show(True)
        
        
        ################################################
        # ROS stuff
        rospy.init_node('ptf_gui', anonymous=True)
        # ros publishers
        self.pub = rospy.Publisher('parameterupdate', Bool)
        #################################################
        
        # send the parameters to ROS on startup
        self.sliderUpdate()
                
        
    def EvtRadioBox(self, event):
        
        print 'button pushed!'
            
    def sliderUpdate(self, event=None):
        self.ptgain = self.ptgain_slider.getval()
        self.ptdamping = self.ptdamping_slider.getval()
        self.ptaccel = self.ptaccel_slider.getval()
        #self.ptminaccel = self.ptminaccel_slider.getval()
        self.focusgain = self.focusgain_slider.getval()
        self.focusdamping = self.focusdamping_slider.getval()
        self.focusaccel = self.focusaccel_slider.getval()
        #self.focusminaccel = self.focusminaccel_slider.getval()
        self.motoradjust_pan = self.motoradjust_pan_slider.getval()
        self.motoradjust_tilt = self.motoradjust_tilt_slider.getval()
        
        rospy.set_param('ptgain', self.ptgain)
        rospy.set_param('ptdamping', self.ptdamping)
        rospy.set_param('ptaccel', self.ptaccel)
        #rospy.set_param('ptminaccel', self.ptminaccel)
        rospy.set_param('focusgain', self.focusgain)
        rospy.set_param('focusdamping', self.focusdamping)
        rospy.set_param('focusaccel', self.focusaccel)
        #rospy.set_param('focusminaccel', self.focusminaccel)
        rospy.set_param('motoradjust_pan', self.motoradjust_pan)
        rospy.set_param('motoradjust_tilt', self.motoradjust_tilt)
        print self.motoradjust_pan
        self.pub.publish(Bool(True))
        
if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--motor", type="str", dest="motor", default='pan',
                        help="motor identifier, ie. pan, tilt, or focus")
    parser.add_option("--dummy", action="store_true", dest="dummy", default=False,
                        help="with dummy = True, will not attempt to talk to controller, but will return false motor values")
    (options, args) = parser.parse_args()

    app = wx.App()
    f = Frame()
    app.MainLoop()
    
    
    
    
    
