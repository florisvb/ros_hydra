#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_hydra')
import rospy
from std_msgs.msg import *

import wx

class Frame(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, title='Events Test')
        panel = wx.Panel(self)
    
        self.pub = rospy.Publisher('wx_controller', String)
        rospy.init_node('wx_controller')
    
        
        panel.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)
        panel.Bind(wx.EVT_LEFT_DOWN, self.Mouse)
        self.Show(True)
        
    def OnKeyDown(self, event):
        # left arrow: 314
        # right arrow: 316
        # up arrow: 315
        # down arrow: 317
        # c: 67
        print 'keyboard!'
        
        key = event.GetKeyCode()
        if key == 314:
            msg = String('left')
        if key == 316:
            msg = String('right')
        if key == 315:
            msg = String('up')
        if key == 317:
            msg = String('down')
        if key == 67:
            msg = String('clear')
        self.pub.publish( msg )
        event.Skip()
    def Mouse(self, event):
        print 'mouse!'
        self.pub.publish(String('mouse!'))
        event.Skip()
        
if __name__ == '__main__':
    app = wx.App()
    f = Frame()
    app.MainLoop()
