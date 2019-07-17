#!/usr/bin/env python

import evdev
from evdev import InputDevice, categorize, ecodes, util, events
import pprint
import rospy
from control.msg import *


TARGET_DEVICE_NAME = "Sony Computer Entertainment Wireless Controller"

ABS_INPUT_RANGE = 255.0
STEERING = 'ABS_X'
BRAKE = 'ABS_RX'
THROTTLE = 'ABS_RY'

COMMAND_RANGE = 100.0
UPDATE_RATE = 50.0 # 50 Hz

ecodes_dict = {
    304: 'SQUARE',
    305: 'CROSS',
    306: 'CIRCLE',
    307: 'TRIANGLE',
    308: 'LEFT_SHOULDER',
    309: 'RIGHT_SHOULDER',
    310: 'LEFT_TRIGGER',
    311: 'RIGHT_TRIGGER',
    312: 'SHARE',
    313: 'OPTIONS',
    314: 'LEFT_JOYSTICK',
    315: 'RIGHT_JOYSTICK',
    316: 'PS_BUTTON',
    317: 'TOUCH_PAD'
    }

class joystick_input:
    def __init__(self, steering_offset=0.0):
        self.brake = 0.0
        self.throttle = 0.0
        self.steering = 0.0
        self.steering_offset = steering_offset

    def set_brake(self, val):
        self.brake = val

    def set_throttle(self, val):
        self.throttle = val

    def set_steering(self, val):
        self.steering = val

    def get_command(self):
        throttle_cmd = (self.throttle - self.brake) / ABS_INPUT_RANGE * COMMAND_RANGE
        steering_cmd = (self.steering - 0.5 * ABS_INPUT_RANGE) / ABS_INPUT_RANGE * COMMAND_RANGE * 2.0 + self.steering_offset
        steering_cmd = -min(COMMAND_RANGE, max(-COMMAND_RANGE, steering_cmd))
        return throttle_cmd, steering_cmd


if __name__=="__main__":
    rospy.init_node('joystick')
    pub = rospy.Publisher('vehicle_control', command, queue_size=10)
    rate = rospy.Rate(UPDATE_RATE)
    joy_input = joystick_input(steering_offset=1.17)

    pp = pprint.PrettyPrinter(indent=2)

    devices= [evdev.InputDevice(path) for path in evdev.list_devices()]
    
    target_dev = None

    while target_dev is None:
        for device in devices:
            if device.name == TARGET_DEVICE_NAME:
                target_dev = device
                print "Target device found!"

            rate.sleep()

    while not rospy.is_shutdown():

        event = target_dev.read_one()

        while event is not None:

            #if event.type == ecodes.EV_KEY:
                #data = categorize(event)
                #print 'press down' if data.keystate else 'press up'
                #print event.code
                #print util.resolve_ecodes(ecodes_dict, [event.code])[0][0]
            if event.type == ecodes.EV_ABS:
                data = categorize(event)
                name = ecodes.bytype[data.event.type][data.event.code]

                if name == STEERING:
                    joy_input.set_steering(data.event.value)
                elif name == THROTTLE:
                    joy_input.set_throttle(data.event.value)
                elif name == BRAKE:
                    joy_input.set_brake(data.event.value)
            
            event = target_dev.read_one()

        throttle_cmd, steering_cmd = joy_input.get_command()
        pub.publish(throttle_cmd, steering_cmd)

        rate.sleep()


