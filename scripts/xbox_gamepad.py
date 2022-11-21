#!/usr/bin/env python3
#  Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Simple example showing how to get gamepad events."""
import time
import threading
import numpy as np
import inputs
from inputs import devices
from inputs import get_gamepad
import sys
import rospy
from geometry_msgs.msg import Twist


B0  = 1
B1  = 1
B2  = 1
B3  = 1
B4  = 1
B5  = 1
B6  = 1
B7  = 1
B8  = 1
B9  = 1
B10 = 1
B11 = 1

class Gamepad:
    def __init__(self):
        gamepads = [device.name for device in devices if type(device) is inputs.GamePad]
        if gamepads == []:
            raise Exception('No gamepad detected.')

        for gamepad in gamepads:
            if 'ESM' in gamepad or 'USB GAMEPAD' in gamepad:
                break
            else:
                raise Exception('Gamepad in incorrect mode.')

        self.axesMap = {
            'ABS_X':'LEFT_X',
            'ABS_Y':'LEFT_Y',
            'ABS_Z':'RIGHT_X',
            'ABS_RZ':'RIGHT_Y',
            'ABS_GAS':'RT',
            'ABS_BRAKE':'LT',
        }

        self.buttonMap = {
            'BTN_SOUTH':'A',
            'BTN_EAST':'B',
            'BTN_NORTH':'X',
            'BTN_WEST':'Y',
            'BTN_TL':'LB',
            'BTN_TR':'RB',
            'BTN_TL2':'LT',
            'BTN_TR2':'RT',
            'BTN_THUMBL':'L_JOY',
            'BTN_THUMBR':'R_JOY',
            'BTN_MODE':'MODE',
            'BTN_START': 'START',
            'BTN_SELECT': 'BACK',
        }

        self.buttons = {}
        self.axes = {}
        self.hat = [0,0]
        self.states = { 'axes': self.axes,
                        'buttons': self.buttons,
                        'hat': self.hat
                    }

        for button in self.buttonMap.keys():
            self.buttons[self.buttonMap[button]] = 0

        for axis in self.axesMap.keys():
            if axis == "ABS_GAS" or axis == "ABS_BRAKE":
                self.axes[self.axesMap[axis]] = 0
            else:
                self.axes[self.axesMap[axis]] = 128

        # self.stateUpdater()
        self.stateUpdaterThread = threading.Thread(target=self.stateUpdater)
        self.stateUpdaterThread.start()

    def _getStates(self):
        events = get_gamepad()
        for event in events:
            # print(event.ev_type, event.code, event.state)
            if event.ev_type == "Absolute":
                if 'ABS_HAT' in event.code:
                    if 'ABS_HAT0X' == event.code:
                        self.hat[0] = event.state
                    elif 'ABS_HAT0Y' == event.code:
                        self.hat[1] = -event.state
                else:
                    self.axes[self.axesMap[event.code]] = event.state
            elif event.ev_type == "Key":
                self.buttons[self.buttonMap[event.code]] = event.state
            else:
                pass

        self.states = { 'axes': self.axes,
                        'buttons': self.buttons,
                        'hat': self.hat
                    }

        return self.states

    def stateUpdater(self):
        while True:
            self._getStates()

    def getStates(self):
        return self.states

    def getDataAssoc(self):
        data_assoc = dict()
        data_assoc['axes'] = dict()
        data_assoc['axes']['LEFT_X'] = -round(((-2/255)*self.axes['LEFT_X'])+1, 2)
        data_assoc['axes']['LEFT_Y'] = round(((-2/255)*self.axes['LEFT_Y'])+1, 2)
        data_assoc['axes']['RIGHT_X'] = -round(((-2/255)*self.axes['RIGHT_X'])+1, 2)
        data_assoc['axes']['RIGHT_Y'] = round(((-2/255)*self.axes['RIGHT_Y'])+1, 2)
        data_assoc['axes']['LT'] = ((1/255)*self.axes['LT'])
        data_assoc['axes']['RT'] = ((1/255)*self.axes['RT'])
        data_assoc['buttons'] = self.buttons
        data_assoc['hat'] = self.hat
        return data_assoc

    def getData(self):
        axes = np.array([((-2/255)*self.axes['LEFT_X'])+1,
                         ((-2/255)*self.axes['LEFT_Y'])+1,
                         ((-2/255)*self.axes['RIGHT_X'])+1,
                         ((-2/255)*self.axes['RIGHT_Y'])+1,
                         ((1/255)*self.axes['LT']),
                         ((1/255)*self.axes['RT'])]) # store all axes in an array

        buttons = np.array([self.buttons['Y'],              # B0
                            self.buttons['B'],              # B1
                            self.buttons['A'],              # B2
                            self.buttons['X'],              # B3
                            self.buttons['LB'],             # B4
                            self.buttons['RB'],             # B5
                            self.buttons['LT'],             # B6
                            self.buttons['RT'],             # B7
                            self.buttons['BACK'],           # B8
                            self.buttons['START'],          # B9
                            self.buttons['L_JOY'],          # B10
                            self.buttons['R_JOY']]          # B11
                            )                               # store all buttons in array

        gp_data = np.hstack((axes, buttons))                # this array will have 16 elements

        return(gp_data)

def gamepad_run():
    rospy.init_node('d3_gamepad', anonymous=True) # ROS node name will start with 'd3_gamepad' and be suffixed with some random numbers 
    rospy.loginfo(sys.path)
    rate = rospy.Rate(10) # 10 Hz refresh from input

    max_lin_vel = rospy.get_param("~max_linear_velocity", 1.0)
    rospy.loginfo("max_linear_velocity: %f", max_lin_vel)
    max_ang_vel = rospy.get_param("~max_angular_velocity", 1.0)
    rospy.loginfo("max_angular_velocity: %f", max_ang_vel)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Publish a Twist to the cmd_vel topic, only queue 10 messages if our Subscribers are not receiving them fast enough

    gp = Gamepad()

    while not rospy.is_shutdown():
        gp_data = gp.getDataAssoc()
        rospy.loginfo(gp_data)
        vel_msg = Twist()
        # Data comes normalized -1,1. Convert to range specified by max_*_vel parameters.
        vel_msg.linear.x = gp_data['axes']['LEFT_Y'] * max_lin_vel
        # Invert angular for controller input to match robots coordinate system
        vel_msg.angular.z = gp_data['axes']['LEFT_X'] * max_ang_vel * -1.0
        vel_msg.linear.y = gp_data['buttons']['BACK'] #EStop Stop
        vel_msg.linear.z = gp_data['buttons']['START'] #EStop Resume

        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        gamepad_run()
    except rospy.ROSInterruptException:
        pass
